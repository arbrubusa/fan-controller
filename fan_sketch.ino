#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_EMC2101.h>

// ==== CONFIG WIFI + MQTT ====
const char* ssid       = "<YOUR SSID>";
const char* password   = "<YOUR PASSWORD>";
const char* mqtt_server= "<MQTT IP>"; // IP do Proxmox (Mosquitto)

WiFiClient espClient;
PubSubClient client(espClient);

// ==== Endereço PCA9546 ====
#define PCA9546_ADDR 0x70

// ==== EMC2101 ====
Adafruit_EMC2101 emc_in;
Adafruit_EMC2101 emc_out;
#define EMC_ADDR  0x4C   // default do EMC2101

// ==== CONFIG CURVA COM HISTERESE ====
float temp_low_up   = 27.0;   // sobe para duty_mid quando >= 27°C
float temp_low_down = 25.0;   // desce para duty_low quando <= 25°C
float temp_high_up  = 32.0;   // sobe para duty_high quando >= 32°C
float temp_high_down= 30.0;   // desce para duty_mid quando <= 30°C

int duty_low   = 30;     // %
int duty_mid   = 65;     // %
int duty_high  = 90;     // %

float ratio_out= 0.88;   // saída = 88% da entrada

// ==== CONTROLE TEMPORAL (30 segundos) ====
unsigned long temp_change_time = 0;
int current_duty = 30;           // duty atual (inicia em low)
int target_duty = 30;            // duty desejado
const unsigned long CHANGE_DELAY = 30000; // 30 segundos

// ==== SAFE MODE ====
unsigned long lastMqtt = 0;
const unsigned long timeoutMqtt = 15000; // 15 seg sem broker = fans 100%

// ==== Seleção de canal PCA9546 ====
void selectChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9546_ADDR);
  Wire.write(1 << channel);  // channel = 0..3
  Wire.endTransmission();
}

// ==== MQTT callback ====
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i=0;i<length;i++) msg += (char)payload[i];
  if (String(topic) == "config/fan/curve") {
  }
}

void reconnectMQTT() {
  while (!client.connected()) {
    if (client.connect("ESP32-fan")) {
      client.subscribe("config/fan/curve");
    } else {
      delay(2000);
    }
  }
}

// ==== Determina duty baseado na temperatura com histerese ====
int calculateTargetDuty(float temp) {
  // Se está em duty_low (30%)
  if (current_duty == duty_low) {
    if (temp >= temp_low_up) return duty_mid;  // 27°C → sobe para 65%
    return duty_low;  // mantém 30%
  }
  
  // Se está em duty_mid (65%)
  if (current_duty == duty_mid) {
    if (temp <= temp_low_down) return duty_low;   // 25°C → desce para 30%
    if (temp >= temp_high_up) return duty_high;   // 32°C → sobe para 90%
    return duty_mid;  // mantém 65%
  }
  
  // Se está em duty_high (90%)
  if (current_duty == duty_high) {
    if (temp <= temp_high_down) return duty_mid;  // 30°C → desce para 65%
    return duty_high;  // mantém 90%
  }
  
  return current_duty; // fallback
}

void setup() {
  Serial.begin(115200);
  Wire.begin(22,20); // SDA=22, SCL=20 (ESP32 Feather V2)

  // WiFi
  WiFi.begin(ssid,password);
  while (WiFi.status()!=WL_CONNECTED){ delay(500); Serial.print(".");}
  Serial.println("\nWiFi conectado!");

  // MQTT
  client.setServer(mqtt_server,1883);
  client.setCallback(callback);

  // Inicializa EMC2101 canal 0 (entrada)
  selectChannel(0);
  if (!emc_in.begin(EMC_ADDR, &Wire)) {
    Serial.println("ERRO: não encontrou EMC2101 no canal 0");
  } else {
    Serial.println("EMC2101 entrada OK");
  }
  emc_in.enableTachInput(true);
  emc_in.LUTEnabled(false);
  emc_in.invertFanSpeed(false);
  emc_in.configPWMClock(true,true);
  emc_in.setPWMDivisor(1);

  // Inicializa EMC2101 canal 1 (saída)
  selectChannel(1);
  if (!emc_out.begin(EMC_ADDR, &Wire)) {
    Serial.println("ERRO: não encontrou EMC2101 no canal 1");
  } else {
    Serial.println("EMC2101 saída OK");
  }
  emc_out.enableTachInput(true);
  emc_out.LUTEnabled(false);
  emc_out.invertFanSpeed(false);
  emc_out.configPWMClock(true,true);
  emc_out.setPWMDivisor(1);

  // Inicia com duty baixo
  selectChannel(0);
  emc_in.setDutyCycle(current_duty);
  selectChannel(1);
  emc_out.setDutyCycle((int)(current_duty * ratio_out));
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  } else {
    lastMqtt = millis();
  }
  client.loop();

  // === Leitura Temp + Controle Fans ===

  // Seleciona canal 0 → Entrada (com offset de calibração)
  selectChannel(0);
  float t_in = emc_in.getInternalTemperature() - 6.0; // calibração
  int rpm_in = emc_in.getFanRPM();

  // Seleciona canal 1 → Saída
  selectChannel(1);
  float t_out = emc_out.getInternalTemperature();
  int rpm_out = emc_out.getFanRPM();

  // Calcula duty desejado baseado na histerese
  int new_target_duty = calculateTargetDuty(t_in);

  // Se mudou o target, inicia contagem de tempo
  if (new_target_duty != target_duty) {
    target_duty = new_target_duty;
    temp_change_time = millis();
    Serial.printf("Nova meta: %d%% (aguardando 30s...)\n", target_duty);
  }

  // Se passou 30 segundos com o mesmo target, aplica a mudança
  if (target_duty != current_duty && (millis() - temp_change_time >= CHANGE_DELAY)) {
    current_duty = target_duty;
    Serial.printf("Aplicando mudança: %d%%\n", current_duty);
  }

  int duty_out = (int)(current_duty * ratio_out);

  // FAILSAFE
  if (millis()-lastMqtt > timeoutMqtt) {
    current_duty = 100;
    duty_out = 100;
    Serial.println("Failsafe: Fans 100% (MQTT timeout)");
  }

  // Aplica duty nos 2 controladores
  selectChannel(0);
  emc_in.setDutyCycle(current_duty);
  selectChannel(1);
  emc_out.setDutyCycle(duty_out);

  // === Debug e publicação MQTT ===
  char buf[64];

  sprintf(buf,"%d", rpm_in);
  bool pub1 = client.publish("fans/inlet/rpm", buf);
  Serial.printf("RPM inlet: %s (ok: %d)\n", buf, pub1);

  sprintf(buf,"%d", rpm_out);
  bool pub2 = client.publish("fans/outlet/rpm", buf);
  Serial.printf("RPM outlet: %s (ok: %d)\n", buf, pub2);

  // TEMPERATURA - COM DEBUG
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

  // Status geral
  Serial.printf("MQTT Status: %d | WiFi: %d\n", client.connected(), WiFi.status());

  // Log Serial
  unsigned long time_remaining = 0;
  if (target_duty != current_duty) {
    time_remaining = (CHANGE_DELAY - (millis() - temp_change_time)) / 1000;
  }
  
  Serial.printf("T_in=%.1f°C, RPM_in=%d, duty=%d%% | T_out=%.1f°C, RPM_out=%d, duty_out=%d%% | Target=%d%% (-%lus)\n",
                t_in, rpm_in, current_duty, t_out, rpm_out, duty_out, target_duty, time_remaining);

  delay(2000);
}
