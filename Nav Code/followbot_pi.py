### This is Chat GPT converting the code from Maix to Raspberry Pi 4b ###
import cv2  # Import OpenCV for image processing
import RPi.GPIO as GPIO  # Import RPi.GPIO for controlling GPIO pins
import time  # Import time for delay and timing functions

# GPIO Pin Setup
PWM_PIN_L = 18  # GPIO pin for PWM signal to the left motor (BCM 18 = Physical Pin 12)
PWM_PIN_R = 19  # GPIO pin for PWM signal to the right motor (BCM 19 = Physical Pin 35)
IN1 = 23  # GPIO pin for IN1 (controls direction of left motor) (BCM 23 = Physical Pin 16)
IN2 = 24  # GPIO pin for IN2 (controls direction of left motor) (BCM 24 = Physical Pin 18)
IN3 = 27  # GPIO pin for IN3 (controls direction of right motor) (BCM 27 = Physical Pin 13)
IN4 = 22  # GPIO pin for IN4 (controls direction of right motor) (BCM 22 = Physical Pin 15)

# Set up GPIO mode and initialize the pins
GPIO.setmode(GPIO.BCM)  # Use BCM numbering scheme for GPIO pins
GPIO.setup(PWM_PIN_L, GPIO.OUT)  # Set up PWM_PIN_L as an output pin
GPIO.setup(PWM_PIN_R, GPIO.OUT)  # Set up PWM_PIN_R as an output pin
GPIO.setup(IN1, GPIO.OUT)  # Set up IN1 as an output pin
GPIO.setup(IN2, GPIO.OUT)  # Set up IN2 as an output pin
GPIO.setup(IN3, GPIO.OUT)  # Set up IN3 as an output pin
GPIO.setup(IN4, GPIO.OUT)  # Set up IN4 as an output pin

# PWM Setup
ENA = GPIO.PWM(PWM_PIN_L, 50)  # Initialize PWM on left motor pin with a 50Hz frequency
ENB = GPIO.PWM(PWM_PIN_R, 50)  # Initialize PWM on right motor pin with a 50Hz frequency
ENA.start(0)  # Start PWM with 0% duty cycle on left motor (stopped initially)
ENB.start(0)  # Start PWM with 0% duty cycle on right motor (stopped initially)

# Motor Control Functions

def forward(MOTOR_POWER):
    """Move the robot forward with a specified motor power."""
    ENA.ChangeDutyCycle(MOTOR_POWER)  # Set the duty cycle for left motor
    ENB.ChangeDutyCycle(MOTOR_POWER)  # Set the duty cycle for right motor
    GPIO.output(IN1, GPIO.HIGH)  # Set IN1 high (forward direction for left motor)
    GPIO.output(IN2, GPIO.LOW)  # Set IN2 low (disable reverse direction for left motor)
    GPIO.output(IN3, GPIO.HIGH)  # Set IN3 high (forward direction for right motor)
    GPIO.output(IN4, GPIO.LOW)  # Set IN4 low (disable reverse direction for right motor)

def back(MOTOR_POWER):
    """Move the robot backward with a specified motor power."""
    ENA.ChangeDutyCycle(MOTOR_POWER)  # Set the duty cycle for left motor
    ENB.ChangeDutyCycle(MOTOR_POWER)  # Set the duty cycle for right motor
    GPIO.output(IN1, GPIO.LOW)  # Set IN1 low (disable forward direction for left motor)
    GPIO.output(IN2, GPIO.HIGH)  # Set IN2 high (reverse direction for left motor)
    GPIO.output(IN3, GPIO.LOW)  # Set IN3 low (disable forward direction for right motor)
    GPIO.output(IN4, GPIO.HIGH)  # Set IN4 high (reverse direction for right motor)

def left(MOTOR_POWER):
    """Turn the robot left by rotating the motors in opposite directions."""
    ENA.ChangeDutyCycle(MOTOR_POWER)  # Set the duty cycle for left motor
    ENB.ChangeDutyCycle(MOTOR_POWER)  # Set the duty cycle for right motor
    GPIO.output(IN1, GPIO.LOW)  # Set IN1 low (disable forward direction for left motor)
    GPIO.output(IN2, GPIO.HIGH)  # Set IN2 high (reverse direction for left motor)
    GPIO.output(IN3, GPIO.HIGH)  # Set IN3 high (forward direction for right motor)
    GPIO.output(IN4, GPIO.LOW)  # Set IN4 low (disable reverse direction for right motor)

def right(MOTOR_POWER):
    """Turn the robot right by rotating the motors in opposite directions."""
    ENA.ChangeDutyCycle(MOTOR_POWER)  # Set the duty cycle for left motor
    ENB.ChangeDutyCycle(MOTOR_POWER)  # Set the duty cycle for right motor
    GPIO.output(IN1, GPIO.HIGH)  # Set IN1 high (forward direction for left motor)
    GPIO.output(IN2, GPIO.LOW)  # Set IN2 low (disable reverse direction for left motor)
    GPIO.output(IN3, GPIO.LOW)  # Set IN3 low (disable forward direction for right motor)
    GPIO.output(IN4, GPIO.HIGH)  # Set IN4 high (reverse direction for right motor)

def stop():
    """Stop the robot by setting motor power to zero."""
    ENA.ChangeDutyCycle(0)  # Stop the left motor
    ENB.ChangeDutyCycle(0)  # Stop the right motor
    GPIO.output(IN1, GPIO.LOW)  # Set IN1 low (disable forward direction for left motor)
    GPIO.output(IN2, GPIO.LOW)  # Set IN2 low (disable reverse direction for left motor)
    GPIO.output(IN3, GPIO.LOW)  # Set IN3 low (disable forward direction for right motor)
    GPIO.output(IN4, GPIO.LOW)  # Set IN4 low (disable reverse direction for right motor)

# Camera Initialization
cap = cv2.VideoCapture(0)  # Open the default camera (USB camera)
FRAME_C_W = 160  # Set the center width of the frame
FRAME_C_H = 120  # Set the center height of the frame
ACCEPTABLE_ERROR_IN_FRAMING = 30  # Define acceptable margin for error in framing
pan_servo_duty = 7.5  # Initial pan servo duty cycle (for tracking)
tilt_servo_duty = 8  # Initial tilt servo duty cycle (for tracking)

# Main loop for processing camera input and controlling motors
while True:
    ret, frame = cap.read()  # Capture a frame from the camera
    if not ret:
        break  # Exit loop if there's an error with the camera

    # Image processing
    frame = cv2.flip(frame, 1)  # Mirror the frame horizontally
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert the frame to HSV color space
    lower_bound = (33, 50, 50)  # Define lower bound for color detection (in HSV)
    upper_bound = (90, 255, 255)  # Define upper bound for color detection (in HSV)
    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)  # Create a mask for the detected color
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # Find contours

    if contours:  # If any contours were found
        biggest_blob = max(contours, key=cv2.contourArea)  # Find the largest contour (blob)
        x, y, w, h = cv2.boundingRect(biggest_blob)  # Get the bounding box for the largest blob
        cx, cy = x + w // 2, y + h // 2  # Calculate the center of the blob
        pixels_number = cv2.contourArea(biggest_blob)  # Calculate the area of the blob
        cv2.circle(frame, (cx, cy), 20, (0, 255, 0), 2)  # Draw a circle around the blob center
        cv2.putText(frame, f"X: {cx} Y: {cy} P: {int(pixels_number)}", (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)  # Display blob info

        # Adjust pan and tilt servo duty cycle based on blob position
        if cx > (FRAME_C_W + ACCEPTABLE_ERROR_IN_FRAMING):
            pan_servo_duty -= 0.3  # Decrease pan duty to move right
        elif cx < (FRAME_C_W - ACCEPTABLE_ERROR_IN_FRAMING):
            pan_servo_duty += 0.3  # Increase pan duty to move left

        if cy > (FRAME_C_H + ACCEPTABLE_ERROR_IN_FRAMING):
            tilt_servo_duty -= 0.2  # Decrease tilt duty to move up
        elif cy < (FRAME_C_H - ACCEPTABLE_ERROR_IN_FRAMING):
            tilt_servo_duty += 0.2  # Increase tilt duty to move down

        # Control motor movement based on blob size and position
        if pixels_number < 4000:  # Blob is small (far away or small object)
            if pan_servo_duty > 8.5:
                left(40)  # Turn left if the blob is to the left
            elif pan_servo_duty < 6.5:
                right(40)  # Turn right if the blob is to the right
