import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time

# MQTT Broker details (use your Mosquitto broker IP)
MQTT_BROKER = "192.168.4.143"  # Change if needed
MQTT_PORT = 1883
MQTT_TOPIC = "esp32/rts-buzzer"  # Topic for buzzer control

# GPIO setup
BUTTON_GPIO = 4  # Change to match your wiring
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Use pull-up resistor

# MQTT setup
client = mqtt.Client()
client.username_pw_set("homeassistant", "OolongTeasss0516!")  # If using authentication
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Function to send MQTT message to turn buzzer ON and OFF
def button_pressed(channel):
    print("Doorbell pressed! Sending MQTT ON message...")
    client.publish(MQTT_TOPIC, "ON", qos=1)  # Send 'ON' message
    time.sleep(2)  # Keep the buzzer on for 2 seconds
    print("Sending MQTT OFF message...")
    client.publish(MQTT_TOPIC, "OFF", qos=1)  # Send 'OFF' message

# Add event detection for button press
GPIO.add_event_detect(BUTTON_GPIO, GPIO.FALLING, callback=button_pressed, bouncetime=300)

# Keep script running
try:
    print("Waiting for button press...")
    client.loop_start()  # Start MQTT loop
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()
    client.loop_stop()
    client.disconnect()