# modules/sensor_fusion/rpi_ultrasonic_sensor.py
import RPi.GPIO as GPIO
import time
import threading
import logging

logger = logging.getLogger(__name__)

class RPiUltrasonicSensor:
    def __init__(self, trigger_pin, echo_pin, name="sensor"):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.name = name
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(trigger_pin, GPIO.OUT)
        GPIO.setup(echo_pin, GPIO.IN)
        
        # Initialize pin states
        GPIO.output(trigger_pin, False)
        
        logger.info(f"Initialized ultrasonic sensor '{name}' on pins: trigger={trigger_pin}, echo={echo_pin}")
    
    def get_distance(self):
        """Measure distance in cm"""
        # Ensure trigger is LOW
        GPIO.output(self.trigger_pin, False)
        time.sleep(0.000002)  # 2 microseconds settle
        
        # Send 10µs pulse
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00001)  # 10 microseconds
        GPIO.output(self.trigger_pin, False)
        
        # Wait for echo to start
        pulse_start = time.time()
        timeout = pulse_start + 0.1  # 100ms timeout
        
        while GPIO.input(self.echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return -1  # Timeout error
        
        # Wait for echo to end
        pulse_end = time.time()
        timeout = pulse_end + 0.1  # 100ms timeout
        
        while GPIO.input(self.echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return -1  # Timeout error
        
        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150  # Speed of sound = 343m/s
        
        # Round to 1 decimal place
        return round(distance, 1)