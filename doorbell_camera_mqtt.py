import time
import paho.mqtt.client as mqtt
import os
from picamera2 import Picamera2  # picamera2 import for newer versions

# MQTT broker details
mqtt_broker = "192.168.4.143"  # Replace with your MQTT broker's IP address
mqtt_port = 1883  # Default MQTT port
mqtt_username = "homeassistant"  # Replace with your MQTT username
mqtt_password = "" # Replace with your MQTT password
mqtt_topic = "homeassistant/doorbell/video"  # MQTT topic for video

# Path where videos will be stored (NAS-mounted directory)
video_path = "/mnt/nas/doorbell_videos"  # Update with the correct NAS path

# Create the video directory if it doesn't exist
os.makedirs(video_path, exist_ok=True)

# Initialize MQTT client and set username/password for authentication
client = mqtt.Client()
client.username_pw_set(mqtt_username, mqtt_password)

# Connect to the MQTT broker
try:
    client.connect(mqtt_broker, mqtt_port, 60)
except Exception as e:
    print(f"Failed to connect to MQTT broker: {e}")
    exit(1)

# Function to publish a message to the MQTT topic
def publish_video_filename(filename):
    client.publish(mqtt_topic, filename)

# Function to record a video and publish filename
def record_video():
    # Set the filename for the video file
    filename = f"doorbell_{time.strftime('%Y%m%d_%H%M%S')}.h264"
    filepath = os.path.join(video_path, filename)

    # Start recording video using the Pi Camera
    with Picamera2() as camera:
        camera.start_recording(filepath)
        camera.wait_recording(10)  # Record for 10 seconds
        camera.stop_recording()

    # After recording, publish the filename to MQTT
    publish_video_filename(filename)
    print(f"Video saved as {filename}")

# Main loop to keep the script running and listening for events
while True:
    # For demonstration purposes, we'll trigger the video recording every 60 seconds
    record_video()
    time.sleep(60)
