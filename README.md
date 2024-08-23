This project is a comprehensive IoT-based security and environmental monitoring system using a Raspberry Pi. The system integrates multiple sensors and devices, including a flame sensor, motion sensor (PIR), temperature and humidity sensor (DHT22), RFID reader, and a servo motor. The Raspberry Pi communicates with an MQTT broker to send and receive data, enabling remote monitoring and control.

Project Features:
Flame Detection: Detects the presence of fire and triggers a buzzer and an MQTT message.
Motion Detection: Identifies motion using a PIR sensor and activates an LED, buzzer, and MQTT notification.
Temperature & Humidity Monitoring: Monitors environmental conditions using a DHT22 sensor and sends data to the MQTT broker.
RFID Access Control: Verifies RFID badges and controls a servo motor for access.
MQTT Communication: All sensor data is sent to an MQTT broker for remote monitoring and potential automation.
