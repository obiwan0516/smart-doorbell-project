import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import time
import subprocess
import os
import threading
import csv
import json
from datetime import datetime
from statistics import mean, median, stdev

# MQTT Broker details
MQTT_BROKER = "192.168.4.143"
MQTT_PORT = 1883
MQTT_TOPIC = "esp32/rts-buzzer"  # Topic for buzzer control
MQTT_RESPONSE_TOPIC = "esp32/buzzer-response"  # Topic to receive ESP32 response

# GPIO setup
BUTTON_GPIO = 4  # Change to match your wiring
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Use pull-up resistor

# Base path for data
BASE_PATH = "/var/www/html/videos/"
# Path for response time data
DATA_PATH = os.path.join(BASE_PATH, "response_data")
# Make sure the directories exist
os.makedirs(BASE_PATH, exist_ok=True)
os.makedirs(DATA_PATH, exist_ok=True)

# Timing variables
timing_data = {
    "button_press_times": [],
    "buzzer_response_times": [],
    "video_start_times": [],
    "button_to_buzzer_latencies": [],
    "button_to_video_latencies": []
}

# Current measurement variables
current_measurement = {
    "button_press_time": 0,
    "buzzer_response_received": False,
    "video_start_time": 0,
    "test_id": 0
}

# Mutex lock for thread-safe operations
data_lock = threading.Lock()

# Session ID (timestamp-based)
SESSION_ID = datetime.now().strftime("%Y%m%d_%H%M%S")
current_measurement["test_id"] = 0  # Start at 0, will increment before first test

# Debug flag
DEBUG = True

# Add flags to prevent issues
system_initialized = False
test_in_progress = False  # Flag to prevent multiple simultaneous tests

# Function to handle messages from ESP32
def on_message(client, userdata, message):
    global test_in_progress

    with data_lock:  # Thread-safe operations
        if DEBUG:
            print(f"MQTT Message received: {message.topic} = {message.payload.decode()}")

        if message.topic == MQTT_RESPONSE_TOPIC:
            payload = message.payload.decode()

            if payload == "BUZZER_ON" and test_in_progress:
                # Record buzzer response time
                buzzer_response_time = time.time()

                # Calculate latency
                if current_measurement["button_press_time"] > 0:
                    latency = buzzer_response_time - current_measurement["button_press_time"]

                    # Store data
                    timing_data["buzzer_response_times"].append(buzzer_response_time)
                    timing_data["button_to_buzzer_latencies"].append(latency)
                    current_measurement["buzzer_response_received"] = True

                    print(f"Buzzer response time: {latency:.4f} seconds")

                    # Log individual measurement
                    log_measurement("buzzer", current_measurement["test_id"],
                                    current_measurement["button_press_time"],
                                    buzzer_response_time, latency)

# Handle connection to MQTT broker
def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code {rc}")
    print(f"Subscribing to {MQTT_RESPONSE_TOPIC}")
    client.subscribe(MQTT_RESPONSE_TOPIC)

# Function to record video directly as MP4
def record_video():
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    video_file_mp4 = f"{BASE_PATH}video_{timestamp}.mp4"

    # Record video start time
    with data_lock:
        current_measurement["video_start_time"] = time.time()
        timing_data["video_start_times"].append(current_measurement["video_start_time"])

        # Calculate latency
        if current_measurement["button_press_time"] > 0:
            latency = current_measurement["video_start_time"] - current_measurement["button_press_time"]
            timing_data["button_to_video_latencies"].append(latency)

            print(f"Video start latency: {latency:.4f} seconds")

            # Log individual measurement
            log_measurement("video", current_measurement["test_id"],
                            current_measurement["button_press_time"],
                            current_measurement["video_start_time"], latency)

    print("Recording video for 10 seconds...")
    try:
        h264_file = f"{BASE_PATH}temp_{timestamp}.h264"

        # Record in H264 format first
        subprocess.run([
            "libcamera-vid",
            "-t", "10000",        # Duration in milliseconds
            "-o", h264_file,      # Output to temp file
            "--width", "1280",    # Video width
            "--height", "720",    # Video height
            "--nopreview"         # Don't show preview (headless mode)
        ], check=True)

        # Convert to MP4 using FFmpeg
        subprocess.run([
            "ffmpeg",
            "-i", h264_file,
            "-c:v", "copy",      # Copy video stream without re-encoding
            "-y",                # Overwrite output file if it exists
            video_file_mp4
        ], check=True)

        # Remove temporary file
        if os.path.exists(h264_file):
            os.remove(h264_file)

    except subprocess.CalledProcessError as e:
        print(f"Error in subprocess: {e}")
    except Exception as e:
        print(f"Error recording video: {e}")

    print(f"Video saved to {video_file_mp4}")
    return video_file_mp4

# Function to log individual timing measurement
def log_measurement(measurement_type, test_id, start_time, end_time, latency):
    filename = os.path.join(DATA_PATH, f"{SESSION_ID}_{measurement_type}_measurements.csv")

    # Check if file exists to determine if we need to write headers
    file_exists = os.path.isfile(filename)

    with open(filename, 'a', newline='') as file:
        writer = csv.writer(file)

        # Write headers if new file
        if not file_exists:
            writer.writerow(['test_id', 'timestamp', 'start_time', 'end_time', 'latency_seconds'])

        # Write measurement
        writer.writerow([
            test_id,
            datetime.fromtimestamp(start_time).strftime('%Y-%m-%d %H:%M:%S.%f'),
            start_time,
            end_time,
            latency
        ])

# Function to generate statistical report
def generate_statistics():
    with data_lock:
        # Calculate statistics
        stats = {}

        if timing_data["button_to_buzzer_latencies"]:
            buzzer_latencies = timing_data["button_to_buzzer_latencies"]
            stats["buzzer"] = {
                "count": len(buzzer_latencies),
                "min": min(buzzer_latencies),
                "max": max(buzzer_latencies),
                "mean": mean(buzzer_latencies),
                "median": median(buzzer_latencies),
                "stdev": stdev(buzzer_latencies) if len(buzzer_latencies) > 1 else 0
            }

        if timing_data["button_to_video_latencies"]:
            video_latencies = timing_data["button_to_video_latencies"]
            stats["video"] = {
                "count": len(video_latencies),
                "min": min(video_latencies),
                "max": max(video_latencies),
                "mean": mean(video_latencies),
                "median": median(video_latencies),
                "stdev": stdev(video_latencies) if len(video_latencies) > 1 else 0
            }

        # Write statistics to file
        stats_filename = os.path.join(DATA_PATH, f"{SESSION_ID}_statistics.txt")
        with open(stats_filename, "w") as stats_file:
            stats_file.write(f"Real-Time Systems Response Time Analysis\n")
            stats_file.write(f"Session ID: {SESSION_ID}\n")
            stats_file.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            stats_file.write(f"Total measurements: {current_measurement['test_id']}\n\n")

            if "buzzer" in stats:
                stats_file.write("BUTTON TO BUZZER RESPONSE TIMES:\n")
                stats_file.write(f"  Count: {stats['buzzer']['count']}\n")
                stats_file.write(f"  Minimum latency: {stats['buzzer']['min']:.6f} seconds\n")
                stats_file.write(f"  Maximum latency: {stats['buzzer']['max']:.6f} seconds\n")
                stats_file.write(f"  Mean latency: {stats['buzzer']['mean']:.6f} seconds\n")
                stats_file.write(f"  Median latency: {stats['buzzer']['median']:.6f} seconds\n")
                stats_file.write(f"  Standard deviation: {stats['buzzer']['stdev']:.6f} seconds\n\n")

            if "video" in stats:
                stats_file.write("BUTTON TO VIDEO RECORDING RESPONSE TIMES:\n")
                stats_file.write(f"  Count: {stats['video']['count']}\n")
                stats_file.write(f"  Minimum latency: {stats['video']['min']:.6f} seconds\n")
                stats_file.write(f"  Maximum latency: {stats['video']['max']:.6f} seconds\n")
                stats_file.write(f"  Mean latency: {stats['video']['mean']:.6f} seconds\n")
                stats_file.write(f"  Median latency: {stats['video']['median']:.6f} seconds\n")
                stats_file.write(f"  Standard deviation: {stats['video']['stdev']:.6f} seconds\n\n")

            stats_file.write("SYSTEM ANALYSIS NOTES:\n")
            stats_file.write("1. All measurements represent end-to-end system latency\n")
            stats_file.write("2. Buzzer response includes network traversal time through MQTT\n")
            stats_file.write("3. Video recording latency includes process startup overhead\n")
            stats_file.write("4. Both measurements include button debounce delay (1000ms)\n\n")

            stats_file.write("RAW DATA FILES:\n")
            stats_file.write(f"- {DATA_PATH}/{SESSION_ID}_buzzer_measurements.csv\n")
            stats_file.write(f"- {DATA_PATH}/{SESSION_ID}_video_measurements.csv\n")

        # Also save as JSON for potential further analysis
        json_filename = os.path.join(DATA_PATH, f"{SESSION_ID}_statistics.json")
        with open(json_filename, "w") as json_file:
            json.dump(stats, json_file, indent=2)

        print(f"Statistics saved to {stats_filename} and {json_filename}")
        return stats_filename

# Function to send MQTT message and measure response time
def button_pressed(channel):
    global system_initialized, test_in_progress

    # Ignore button events until the system is properly initialized
    if not system_initialized:
        print("Ignoring button event - system still initializing")
        return

    # Don't start a new test if one is already in progress
    if test_in_progress:
        print("Ignoring button press - test already in progress")
        return

    # Set flag to indicate test is in progress
    test_in_progress = True

    try:
        with data_lock:
            # Increment test ID
            current_measurement["test_id"] += 1
            test_id = current_measurement["test_id"]

            # Reset measurement flags
            current_measurement["buzzer_response_received"] = False

            # Record the press time for response measurement
            button_press_time = time.time()
            current_measurement["button_press_time"] = button_press_time
            timing_data["button_press_times"].append(button_press_time)

            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            print(f"[{timestamp}] Test #{test_id}: Button press detected!")

        # Send MQTT message to trigger buzzer
        print(f"Sending MQTT ON message...")
        client.publish(MQTT_TOPIC, "ON", qos=1)  # Send 'ON' message

        # Wait briefly for buzzer response (non-blocking)
        time.sleep(0.1)

        # Start recording video
        video_file = record_video()

        # Turn off buzzer after video recording
        print("Sending MQTT OFF message...")
        client.publish(MQTT_TOPIC, "OFF", qos=1)  # Send 'OFF' message

        # Generate updated statistics after each test
        generate_statistics()

        # Wait a short time for the system to stabilize before allowing another test
        time.sleep(1)

    finally:
        # Reset the test_in_progress flag when done
        test_in_progress = False

# Main function
def main():
    global client, system_initialized

    # Disable GPIO warnings (optional)
    GPIO.setwarnings(False)

    # MQTT setup
    client = mqtt.Client()
    client.username_pw_set("", "")  # Authentication
    client.on_connect = on_connect
    client.on_message = on_message

    # Connect to MQTT broker
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        print(f"Connecting to MQTT broker at {MQTT_BROKER}...")
        client.loop_start()  # Start MQTT loop
        time.sleep(1)  # Give connection time to establish
    except Exception as e:
        print(f"Failed to connect to MQTT broker: {e}")
        return

    print("Initializing GPIO and waiting for system to stabilize...")

    # Read the initial state of the button to avoid false triggers
    initial_button_state = GPIO.input(BUTTON_GPIO)
    print(f"Initial button state: {'HIGH (not pressed)' if initial_button_state else 'LOW (pressed)'}")

    # Wait for button to be released if it's currently pressed
    if initial_button_state == 0:  # Button is pressed (LOW due to pull-up)
        print("Button appears to be pressed at startup. Please release it...")
        while GPIO.input(BUTTON_GPIO) == 0:
            time.sleep(0.1)
        print("Button released, continuing initialization...")
        time.sleep(0.5)  # Allow time for button to stabilize

    # Add event detection for button press with increased bouncetime
    GPIO.remove_event_detect(BUTTON_GPIO)  # Remove any existing event detection
    time.sleep(0.5)  # Wait before adding new event detection
    GPIO.add_event_detect(BUTTON_GPIO, GPIO.FALLING, callback=button_pressed, bouncetime=2000)

    # Set initialization flag to allow button events
    time.sleep(1)  # Additional delay to ensure stability
    system_initialized = True
    print("System initialization complete, ready for button presses")

    # Initial statistics file creation
    generate_statistics()

    print(f"\n{'='*50}")
    print(f"REAL-TIME SYSTEMS RESPONSE TIME MEASUREMENT")
    print(f"Session ID: {SESSION_ID}")
    print(f"Data directory: {DATA_PATH}")
    print(f"Press Ctrl+C to end the session and generate final report")
    print(f"{'='*50}\n")

    # Keep script running
    try:
        print("Waiting for button press...")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Session ended by user.")
    finally:
        # Clean up resources
        GPIO.remove_event_detect(BUTTON_GPIO)
        GPIO.cleanup()
        client.loop_stop()
        client.disconnect()

        # Generate final statistics
        stats_file = generate_statistics()

        print(f"\n{'='*50}")
        print(f"SESSION COMPLETE")
        print(f"Final statistics saved to {stats_file}")
        print(f"{'='*50}\n")

if __name__ == "__main__":
    main()
