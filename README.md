# fan-controller
Small IOT setup using ESP32 to control the fans in my homelab rack.

You can find here the most important scripts and files to recreate the project in you own homelab.

## Docker structure
This is the directory structure I created to run the software stack:

```bash
tree .
.
├── docker-compose.yml
├── grafana
│ └── data
├── influxdb
│ ├── config
│ └── data
├── mosquitto
│ ├── config
│ ├── data
│ └── log
└── nodered
└── data
```

