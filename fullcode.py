import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt
import subprocess
from gpiozero import LED, Buzzer
from signal import pause
from gpiozero import MotionSensor
from time import sleep
import Adafruit_DHT 
from mfrc522 import SimpleMFRC522
# MQTT broker configuration
mqtt_broker = "Put_Broker_Here"
mqtt_port = 1883
mqtt_topic_flame = "iot/flame_sensor"
mqtt_topic_motion = "iot/motion_sensor"
mqtt_topic_temp = "iot/temp"
mqtt_topic_hum = "iot/hum"
mqtt_topic_rfid = "iot/rfid"
mqtt_username = ""
mqtt_password = ""

# Wi-Fi configuration
wifi_ssid = "*******"
wifi_password = "******"

# Connect to Wi-Fi


# Create MQTT client instance
mqtt_client = mqtt.Client()

# Set MQTT broker credentials
#mqtt_client.username_pw_set(mqtt_username)

# Callback function for MQTT client connection
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker")
    # Subscribe to MQTT topics
    mqtt_client.subscribe(mqtt_topic_flame)
    mqtt_client.subscribe(mqtt_topic_motion)
    mqtt_client.subscribe(mqtt_topic_temp)
    mqtt_client.subscribe(mqtt_topic_hum)

# Callback function for received MQTT messages
def on_message(client, userdata, msg):
    message = msg.payload.decode()
    print(f"Received message: {message}")

# Connect to the MQTT broker
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
mqtt_client.connect(mqtt_broker, mqtt_port)

# Initialize GPIO pins
GPIO.setmode(GPIO.BCM)
flame_pins = [5, 6, 13, 19, 26]
buzzer_pin = 23
pir = MotionSensor(18)
led_red = LED(14)
led_green = LED(15)
DHT_SENSOR = Adafruit_DHT.DHT22
DHT_PIN = 17
servo_pin = 19  # GPIO pin for servo motor control
GPIO.setup(servo_pin, GPIO.OUT)
servo = GPIO.PWM(servo_pin, 50)  # PWM frequency for SG90 servo motor (50Hz)
servo.start(0)  # Start the servo motor at 0% duty cycle

GPIO.setwarnings(False)  # Suppress GPIO warning

for flame_pin in flame_pins:
    GPIO.setup(flame_pin, GPIO.IN)
GPIO.setup(buzzer_pin, GPIO.OUT)

# Create Buzzer object
buzzer = Buzzer(buzzer_pin)
#client.loop_forever()
# Function to read temperature and humidity from DHT22 sensor
def read_dht22():
    humidity, temperature = Adafruit_DHT.read_retry(DHT_SENSOR, DHT_PIN)

    if humidity is not None and temperature is not None:
        print("Temperature={0:0.1f}C Humidity={1:0.1f}%".format(temperature, humidity))
        if temperature > 25:
            led_red.on()
        else:
            led_red.off()
        mqtt_client.publish(mqtt_topic_temp, str(temperature))
        mqtt_client.publish(mqtt_topic_hum, str(humidity))
    else:
        print("Failed to retrieve data from DHT22 sensor")

# Function to handle motion detection
def motion_detected():
    print("Motion detected")
    led_green.off()
    led_red.on()
    buzzer.on()
    time.sleep(3)
    buzzer.off()
    time.sleep(0.1)
    mqtt_client.publish(mqtt_topic_motion, "Motion Detected")


def no_motion_detected():
    print("No motion detected")
    led_green.on()
    led_red.off()
    buzzer.off()
    mqtt_client.publish(mqtt_topic_motion, "No Motion Detected")

# Function to handle flame detection
def check_flame():
    flame_detected = False
    for i, flame_pin in enumerate(flame_pins):
        if GPIO.input(flame_pin):
            flame_detected = True
            break
    
    if flame_detected:
        GPIO.output(buzzer_pin, True)
        print("Flame detected!")
        mqtt_client.publish(mqtt_topic_flame, "Flame Detected")
    else:
        GPIO.output(buzzer_pin, False)
        print("Flame not detected!")
        mqtt_client.publish(mqtt_topic_flame, "Flame Not Detected")
def read_rfid():
    try:
        reader = SimpleMFRC522()
        print('Approchez votre badge')
        id, text = reader.read()
        print('Vous avez passé le badge avec l\'id:', id)
        
        return id
    except KeyboardInterrupt:
        GPIO.cleanup()
        return None
    
def check_compatibility(badge_id, stored_ids):
    global last_motion_time
    str_badge_id = str(badge_id)
    if str_badge_id in stored_ids:
        ok=True
        print('Badge is compatible')
        servo.ChangeDutyCycle(9.5)  # Rotate the servo motor to a specific angle (example: 90 degrees)
        time.sleep(1)  # Wait for the servo motor to reach the desired angle
        servo.ChangeDutyCycle(2.5)  # Rotate the servo motor back to 0 degrees
        time.sleep(1)  # Wait for the servo motor to reach the desired angle
        servo.ChangeDutyCycle(0)  # Stop the servo motor
        last_motion_time = time.time()
        mqtt_client.publish(mqtt_topic_rfid, str(badge_id))
        return True # Publish RFID data via MQTT
    else:
        print('Badge is not compatible')
        return False 
        
stored_badge_ids = ['909975656284']
# Main loop
try:
    global last_motion_time
    while True:  
        read_dht22()
        check_flame() 
        badge_id = read_rfid()
        rfid_detected =check_compatibility(badge_id, stored_badge_ids)
        motion =  pir.wait_for_motion()
        if not motion :
                no_motion_detected()  
        else :
                if time.time()-last_motion_time<20:
                    no_motion_detected()
                else :
                    last_motion_time
                    motion_detected()
            
        if rfid_detected :        
        if time.time()-last_motion_time >20 and :
            if not motion :
                no_motion_detected()  
            else :
                motion_detected()
        

        else :
            if rfid_detected :
                last_motion_time = time.time()
                if  time.time() - last_motion_time<20: 
                    print('hani f west time.time')
                else:
                    motion_detected()

       
           
            
        
        time.sleep(3)  # Delay between iterations

except KeyboardInterrupt:
    GPIO.cleanup()  # Clean up GPIO on keyboard interrupt
