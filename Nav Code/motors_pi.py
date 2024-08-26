### This is Chat GPT converting the code from Maix to Raspberry Pi 4b and adding comments for clarity ###
# Import necessary libraries
import RPi.GPIO as GPIO  # Library to control Raspberry Pi GPIO pins
import time  # Library for time-related functions

# ----------------------------
# GPIO Pin Definitions
# ----------------------------

# PWM (Enable) pins for motor speed control
ENA_PIN = 9   # Enable pin for Motor A
ENB_PIN = 10  # Enable pin for Motor B

# Control pins for Motor A
IN1_PIN = 17  # Input 1 for Motor A
IN2_PIN = 27  # Input 2 for Motor A

# Control pins for Motor B
IN3_PIN = 22  # Input 1 for Motor B
IN4_PIN = 23  # Input 2 for Motor B

# ----------------------------
# GPIO Setup
# ----------------------------

# Set the GPIO mode to BCM (Broadcom SOC channel) numbering
GPIO.setmode(GPIO.BCM)

# Set up PWM pins as outputs
GPIO.setup(ENA_PIN, GPIO.OUT)
GPIO.setup(ENB_PIN, GPIO.OUT)

# Set up control pins as outputs
GPIO.setup(IN1_PIN, GPIO.OUT)
GPIO.setup(IN2_PIN, GPIO.OUT)
GPIO.setup(IN3_PIN, GPIO.OUT)
GPIO.setup(IN4_PIN, GPIO.OUT)

# ----------------------------
# PWM Initialization
# ----------------------------

# Initialize PWM on the enable pins with a frequency of 50Hz
ENA = GPIO.PWM(ENA_PIN, 50)  # PWM object for Motor A
ENB = GPIO.PWM(ENB_PIN, 50)  # PWM object for Motor B

# Start PWM with 0% duty cycle (motors stopped)
ENA.start(0)
ENB.start(0)

# ----------------------------
# Motor Control Functions
# ----------------------------

def forward():
    """
    Move both motors forward at 50% speed.
    """
    ENA.ChangeDutyCycle(50)  # Set speed for Motor A
    ENB.ChangeDutyCycle(50)  # Set speed for Motor B
    GPIO.output(IN1_PIN, GPIO.HIGH)  # Set IN1 high
    GPIO.output(IN2_PIN, GPIO.LOW)   # Set IN2 low
    GPIO.output(IN3_PIN, GPIO.HIGH)  # Set IN3 high
    GPIO.output(IN4_PIN, GPIO.LOW)   # Set IN4 low

def back():
    """
    Move both motors backward at 50% speed.
    """
    ENA.ChangeDutyCycle(50)
    ENB.ChangeDutyCycle(50)
    GPIO.output(IN1_PIN, GPIO.LOW)
    GPIO.output(IN2_PIN, GPIO.HIGH)
    GPIO.output(IN3_PIN, GPIO.LOW)
    GPIO.output(IN4_PIN, GPIO.HIGH)

def right():
    """
    Turn right by running Motor A backward and Motor B forward.
    """
    ENA.ChangeDutyCycle(50)
    ENB.ChangeDutyCycle(50)
    GPIO.output(IN1_PIN, GPIO.LOW)
    GPIO.output(IN2_PIN, GPIO.HIGH)
    GPIO.output(IN3_PIN, GPIO.HIGH)
    GPIO.output(IN4_PIN, GPIO.LOW)

def left():
    """
    Turn left by running Motor A forward and Motor B backward.
    """
    ENA.ChangeDutyCycle(50)
    ENB.ChangeDutyCycle(50)
    GPIO.output(IN1_PIN, GPIO.HIGH)
    GPIO.output(IN2_PIN, GPIO.LOW)
    GPIO.output(IN3_PIN, GPIO.LOW)
    GPIO.output(IN4_PIN, GPIO.HIGH)

def stop():
    """
    Stop both motors.
    """
    ENA.ChangeDutyCycle(0)  # Set speed to 0%
    ENB.ChangeDutyCycle(0)
    GPIO.output(IN1_PIN, GPIO.LOW)
    GPIO.output(IN2_PIN, GPIO.LOW)
    GPIO.output(IN3_PIN, GPIO.LOW)
    GPIO.output(IN4_PIN, GPIO.LOW)

# ----------------------------
# Main Program Loop
# ----------------------------

try:
    while True:
        forward()             # Move forward
        time.sleep(5)         # Wait for 5 seconds

        back()                # Move backward
        time.sleep(5)         # Wait for 5 seconds

        left()                # Turn left
        time.sleep(5)         # Wait for 5 seconds

        right()               # Turn right
        time.sleep(5)         # Wait for 5 seconds

        stop()                # Stop all movement
        time.sleep(5)         # Wait for 5 seconds

except KeyboardInterrupt:
    # If a keyboard interrupt (Ctrl+C) is detected, exit the loop
    pass

finally:
    # Clean up all GPIO settings
    ENA.stop()                # Stop PWM for Motor A
    ENB.stop()                # Stop PWM for Motor B
    GPIO.cleanup()            # Reset all GPIO pins
