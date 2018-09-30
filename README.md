
# Watering System
ESP32 based system controlled by and reporting to MQTT broker. Utilizes Espressif ESP-IDF.
Two analogue pressure sensors are used, one to monitor the water level in the feeder tank, one to monitor pump output pressure.
The pump pressure is regulated by a 100Hz PID controller loop - pump speed control is achieved using a 20kHz PWM signal.
As a diaphram pump is used, the measured pressure has considerable ripple. This is filtered by a 2nd order butterworth IIR filter before entering the PID
Six solenoid valves direct the water to the chosen watering circuit. Only one circuit is active at a time - requests to water multiple circuits are queued and performed in succession.

# UI
A rough and ready UI has been crafted for the excellent mqttdashboard app: https://play.google.com/store/apps/details?id=com.thn.iotmqttdashboard
The layout can be found in dashboard.json - customize and publish this to an MQTT topic of your choice, i.e. metrics/exchange and use the import/export function of the app to import from that topic

## Building
The AWS IoT module used for MQTT insists on using TLS whether you like it or not, so you will need to generate a key and cert to allow it to talk to your broker. In my case I used Mosquitto and had a frustrating time getting it to work until I found that I needed to force TLS v1.2 on the client
 
This has been derived from an ESP-IDF sample app and I haven't taken the time to curate the options, but in essence, do

	make menuconfig

to set the options including serial port for flashing, SSID, password and TLS certs then 

	make

to build and/or

	make flash

to build+flash to the ESP32
