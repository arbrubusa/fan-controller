#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_EMC2101.h>

// ==== WIFI + MQTT CONFIG ====
const char* ssid       = "<YOUR SSID>";
const char* password   = "<YOUR PASSWORD>";
const char* mqtt_server= "<MQTT IP>"; // Mosquitto

WiFiClient espClient;
PubSubClient client(espClient);

// ==== PCA9546 Address ====
#define PCA9546_ADDR 0x70

// ==== EMC2101 ====
Adafruit_EMC2101 emc_in;
Adafruit_EMC2101 emc_out;
#define EMC_ADDR  0x4C   // EMC2101 default address

// ==== TEMPERATURE CURVE WITH HYSTERESIS ====
float temp_low_up   = 27.0;   // rise to duty_mid when >= 27°C
float temp_low_down = 25.0;   // drop to duty_low when <= 25°C
float temp_high_up  = 32.0;   // rise to duty_high when >= 32°C
float temp_high_down= 30.0;   // drop to duty_mid when <= 30°C

int duty_low   = 30;     // %
int duty_mid   = 65;     // %
int duty_high  = 90;     // %

float ratio_out= 0.88;   // outlet = 88% of inlet

// ==== TEMPORAL CONTROL (30 seconds) ====
unsigned long temp_change_time = 0;
int current_duty = 30;           // current duty cycle (starts at low)
int target_duty = 30;            // target duty cycle
const unsigned long CHANGE_DELAY = 30000; // 30 seconds

// ==== FAILSAFE MODE ====
unsigned long lastMqtt = 0;
const unsigned long timeoutMqtt = 15000; // 15 sec without broker = fans 100%
bool failsafe_active = false;
bool just_exited_failsafe = false;
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 60000; // reconnect attempt every 60s

// ==== PCA9546 Channel Selection ====
void selectChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9546_ADDR);
  Wire.write(1 << channel);  // channel = 0..3
  Wire.endTransmission();
}

// ==== MQTT Callback ====
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i=0;i<length;i++) msg += (char)payload[i];
  if (String(topic) == "config/fan/curve") {
    // You can implement JSON parsing here to remotely change the curve
  }
}

void reconnectMQTT() {
  if (!client.connected()) {
    Serial.println("Attempting MQTT reconnection...");
    if (client.connect("ESP32-fan")) {
      client.subscribe("config/fan/curve");
      Serial.println("MQTT reconnected!");
      lastMqtt = millis(); // update timestamp
    } else {
      Serial.printf("MQTT reconnection failed (rc=%d)\n", client.state());
    }
  }
}

// ==== Calculate target duty based on temperature with hysteresis ====
int calculateTargetDuty(float temp) {
  // If at duty_low (30%)
  if (current_duty == duty_low) {
    if (temp >= temp_low_up) return duty_mid;  // 27°C → rise to 65%
    return duty_low;  // maintain 30%
  }
  
  // If at duty_mid (65%)
  if (current_duty == duty_mid) {
    if (temp <= temp_low_down) return duty_low;   // 25°C → drop to 30%
    if (temp >= temp_high_up) return duty_high;   // 32°C → rise to 90%
    return duty_mid;  // maintain 65%
  }
  
  // If at duty_high (90%)
  if (current_duty == duty_high) {
    if (temp <= temp_high_down) return duty_mid;  // 30°C → drop to 65%
    return duty_high;  // maintain 90%
  }
  
  return current_duty; // fallback
}

void setup() {
  Serial.begin(115200);
  Wire.begin(22,20); // SDA=22, SCL=20 (ESP32 Feather V2)

  // WiFi
  WiFi.begin(ssid,password);
  while (WiFi.status()!=WL_CONNECTED){ delay(500); Serial.print(".");}
  Serial.println("\nWiFi connected!");

  // MQTT
  client.setServer(mqtt_server,1883);
  client.setCallback(callback);

  // Initialize EMC2101 channel 0 (inlet)
  selectChannel(0);
  if (!emc_in.begin(EMC_ADDR, &Wire)) {
    Serial.println("ERROR: EMC2101 not found on channel 0");
  } else {
    Serial.println("EMC2101 inlet OK");
  }
  emc_in.enableTachInput(true);
  emc_in.LUTEnabled(false);
  emc_in.invertFanSpeed(false);
  emc_in.configPWMClock(true,true);
  emc_in.setPWMDivisor(1);

  // Initialize EMC2101 channel 1 (outlet)
  selectChannel(1);
  if (!emc_out.begin(EMC_ADDR, &Wire)) {
    Serial.println("ERROR: EMC2101 not found on channel 1");
  } else {
    Serial.println("EMC2101 outlet OK");
  }
  emc_out.enableTachInput(true);
  emc_out.LUTEnabled(false);
  emc_out.invertFanSpeed(false);
  emc_out.configPWMClock(true,true);
  emc_out.setPWMDivisor(1);

  // Start with low duty
  selectChannel(0);
  emc_in.setDutyCycle(current_duty);
  selectChannel(1);
  emc_out.setDutyCycle((int)(current_duty * ratio_out));
}

void loop() {
  // Try to reconnect only if not in failsafe
  if (!client.connected() && !failsafe_active) {
    reconnectMQTT();
  } else if (client.connected()) {
    lastMqtt = millis();
  }
  client.loop();

  // === Temperature Reading + Fan Control ===

  // Select channel 0 → Inlet (with calibration offset)
  selectChannel(0);
  float t_in = emc_in.getInternalTemperature() - 6.0; // calibration
  int rpm_in = emc_in.getFanRPM();

  // Select channel 1 → Outlet
  selectChannel(1);
  float t_out = emc_out.getInternalTemperature();
  int rpm_out = emc_out.getFanRPM();

  // === FAILSAFE WITH AUTO RECONNECTION ===
  if (millis() - lastMqtt > timeoutMqtt) {
    // Entered failsafe
    if (!failsafe_active) {
      Serial.println("ENTERING FAILSAFE: Fans 100% (MQTT timeout)");
      failsafe_active = true;
      lastReconnectAttempt = millis();
    }
    
    // Force fans to 100%
    current_duty = 100;
    target_duty = 100;
    
    // Try to reconnect every 60 seconds
    if (millis() - lastReconnectAttempt >= reconnectInterval) {
      Serial.println("Attempting MQTT reconnection (failsafe active)...");
      reconnectMQTT();
      lastReconnectAttempt = millis();
    }
    
  } else {
    // MQTT is OK - exit failsafe if it was active
    if (failsafe_active) {
      Serial.println("EXITING FAILSAFE: MQTT reconnected");
      Serial.printf("DEBUG: t_in=%.1f, temp_low_up=%.1f, temp_high_up=%.1f\n", t_in, temp_low_up, temp_high_up);
      
      failsafe_active = false;
      just_exited_failsafe = true;
      
      // Determine duty based only on current temperature
      if (t_in < temp_low_up) {
        target_duty = duty_low;
        current_duty = duty_low;
        Serial.printf("DEBUG: t_in < temp_low_up → target_duty = %d\n", duty_low);
      } else if (t_in >= temp_high_up) {
        target_duty = duty_high;
        current_duty = duty_high;
        Serial.printf("DEBUG: t_in >= temp_high_up → target_duty = %d\n", duty_high);
      } else {
        target_duty = duty_mid;
        current_duty = duty_mid;
        Serial.printf("DEBUG: intermediate temp → target_duty = %d\n", duty_mid);
      }
      
      temp_change_time = millis();
      Serial.printf("New target after failsafe: target=%d%%, current=%d%% (applied immediately)\n", target_duty, current_duty);
    }
  }

  // Calculate desired duty based on hysteresis (ONLY IF NOT IN FAILSAFE AND NOT JUST EXITED)
  if (!failsafe_active && !just_exited_failsafe) {
    int new_target_duty = calculateTargetDuty(t_in);

    // If target changed, start time counter
    if (new_target_duty != target_duty) {
      target_duty = new_target_duty;
      temp_change_time = millis();
      Serial.printf("New target: %d%% (waiting 30s...)\n", target_duty);
    }

    // If 30 seconds passed with same target, apply the change
    if (target_duty != current_duty && (millis() - temp_change_time >= CHANGE_DELAY)) {
      Serial.printf("30s elapsed! Changing from %d%% to %d%%\n", current_duty, target_duty);
      current_duty = target_duty;
      Serial.printf("Applying change: %d%%\n", current_duty);
    }
  }
  
  // Reset flag after first loop
  if (just_exited_failsafe) {
    just_exited_failsafe = false;
  }

  int duty_out = (int)(current_duty * ratio_out);

  // Apply duty to both controllers
  selectChannel(0);
  emc_in.setDutyCycle(current_duty);
  selectChannel(1);
  emc_out.setDutyCycle(duty_out);

  // === Debug and MQTT Publishing ===
  char buf[64];

  sprintf(buf,"%d", rpm_in);
  bool pub1 = client.publish("fans/inlet/rpm", buf);
  Serial.printf("RPM inlet: %s (ok: %d)\n", buf, pub1);

  sprintf(buf,"%d", rpm_out);
  bool pub2 = client.publish("fans/outlet/rpm", buf);
  Serial.printf("RPM outlet: %s (ok: %d)\n", buf, pub2);

  // TEMPERATURE - WITH DEBUG
  sprintf(buf,"%.1f", t_in);
  bool pub3 = client.publish("temp/inlet", buf);
  Serial.printf("TEMP inlet: %s (ok: %d)\n", buf, pub3);

  sprintf(buf,"%.1f", t_out);
  bool pub4 = client.publish("temp/outlet", buf);
  Serial.printf("TEMP outlet: %s (ok: %d)\n", buf, pub4);

  sprintf(buf,"%d", current_duty);
  bool pub5 = client.publish("fans/inlet/duty", buf);
  Serial.printf("Duty inlet: %s (ok: %d)\n", buf, pub5);

  sprintf(buf,"%d", duty_out);
  bool pub6 = client.publish("fans/outlet/duty", buf);
  Serial.printf("Duty outlet: %s (ok: %d)\n", buf, pub6);

  // General status
  Serial.printf("MQTT Status: %d | WiFi: %d | Failsafe: %d\n", client.connected(), WiFi.status(), failsafe_active);

  // Serial log
  unsigned long time_remaining = 0;
  if (target_duty != current_duty) {
    time_remaining = (CHANGE_DELAY - (millis() - temp_change_time)) / 1000;
  }
  
  Serial.printf("T_in=%.1f°C, RPM_in=%d, duty=%d%% | T_out=%.1f°C, RPM_out=%d, duty_out=%d%% | Target=%d%% (-%lus)\n",
                t_in, rpm_in, current_duty, t_out, rpm_out, duty_out, target_duty, time_remaining);
  Serial.println("─────────────────────────────────────────────────────────────");

  delay(2000);
}
