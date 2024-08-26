### This is Chat GPT converting the code from Maix to Raspberry Pi 4b and adding comments for clarity ###
# Import necessary libraries
import RPi.GPIO as GPIO  # Library to control Raspberry Pi GPIO pins
import time  # Library for time-related functions

# ----------------------------
# GPIO Pin Definitions
# ----------------------------

# Define GPIO pins for PWM control
PWM_PIN1 = 20  # PWM pin 1 (equivalent to JTAG_TCK on Maix)
PWM_PIN2 = 21  # PWM pin 2 (equivalent to JTAG_TDI on Maix)

# ----------------------------
# GPIO Setup
# ----------------------------

# Set the GPIO mode to BCM (Broadcom SOC channel) numbering
GPIO.setmode(GPIO.BCM)

# Set up PWM pins as outputs
GPIO.setup(PWM_PIN1, GPIO.OUT)
GPIO.setup(PWM_PIN2, GPIO.OUT)

# ----------------------------
# PWM Initialization
# ----------------------------

# Initialize PWM on the defined pins with a frequency of 50Hz
pwm1 = GPIO.PWM(PWM_PIN1, 50)  # PWM object for the first channel
pwm2 = GPIO.PWM(PWM_PIN2, 50)  # PWM object for the second channel

# Start PWM with an initial duty cycle of 5%
pwm1.start(5)
pwm2.start(5)

# ----------------------------
# Main Program Loop
# ----------------------------

try:
    while True:
        # Gradually increase the duty cycle from 5% to 10%
        for x in range(180):
            duty = 5 * (x / 180) + 5  # Calculate duty cycle
            pwm1.ChangeDutyCycle(duty)  # Update duty cycle for PWM1
            pwm2.ChangeDutyCycle(duty)  # Update duty cycle for PWM2
            time.sleep(0.02)  # Small delay to control the speed of change

        # Gradually decrease the duty cycle from 10% to 5%
        for x in range(180):
            duty = 5 * (1 - (x / 180)) + 5  # Calculate duty cycle
            pwm1.ChangeDutyCycle(duty)  # Update duty cycle for PWM1
            pwm2.ChangeDutyCycle(duty)  # Update duty cycle for PWM2
            time.sleep(0.02)  # Small delay to control the speed of change

except KeyboardInterrupt:
    # If a keyboard interrupt (Ctrl+C) is detected, exit the loop
    pass

finally:
    # Clean up all GPIO settings
    pwm1.stop()  # Stop PWM on the first channel
    pwm2.stop()  # Stop PWM on the second channel
    GPIO.cleanup()  # Reset all GPIO pins
