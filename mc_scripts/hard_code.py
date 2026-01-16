import RPi.GPIO as GPIO
import time

# ====================== SETTINGS ======================

# Speeds
SPEED_NORMAL = 70
SPEED_TURN = 55

# Durations (in seconds)
LONG_FORWARD_TIME = 8.0
SHORT_FORWARD_TIME = 1.5
TURN_TIME = 1.0  # same for left/right turns

# ======================================================

# Speed settings
SPEED_NORMAL = 70
SPEED_TURN = 55  # Reduced speed for turning to avoid slipping

# Pin setup
leftEn = 13
rightEn = 12
leftBackward = 5
leftForward = 6
rightForward = 16
rightBackward = 20

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

pins = [leftEn, rightEn, leftForward, leftBackward, rightForward, rightBackward]
for pin in pins:
    GPIO.setup(pin, GPIO.OUT)

pwmL = GPIO.PWM(leftEn, 100)
pwmR = GPIO.PWM(rightEn, 100)
pwmL.start(0)
pwmR.start(0)

# Movement control
def stop():
    pwmL.ChangeDutyCycle(0)
    pwmR.ChangeDutyCycle(0)
    GPIO.output(leftForward, GPIO.LOW)
    GPIO.output(leftBackward, GPIO.LOW)
    GPIO.output(rightForward, GPIO.LOW)
    GPIO.output(rightBackward, GPIO.LOW)

def forward(duration, speed=70):
    pwmL.ChangeDutyCycle(speed*1.3)
    pwmR.ChangeDutyCycle(speed)
    GPIO.output(leftForward, GPIO.HIGH)
    GPIO.output(leftBackward, GPIO.LOW)
    GPIO.output(rightForward, GPIO.HIGH)
    GPIO.output(rightBackward, GPIO.LOW)
    time.sleep(duration)
    stop()

def backward(duration, speed=70):
    pwmL.ChangeDutyCycle(speed)
    pwmR.ChangeDutyCycle(speed*0.9)
    GPIO.output(leftForward, GPIO.LOW)
    GPIO.output(leftBackward, GPIO.HIGH)
    GPIO.output(rightForward, GPIO.LOW)
    GPIO.output(rightBackward, GPIO.HIGH)
    time.sleep(duration)
    stop()

def right(duration, speed=70):
    pwmL.ChangeDutyCycle(speed)
    pwmR.ChangeDutyCycle(speed)
    GPIO.output(leftForward, GPIO.LOW)
    GPIO.output(leftBackward, GPIO.HIGH)
    GPIO.output(rightForward, GPIO.HIGH)
    GPIO.output(rightBackward, GPIO.LOW)
    time.sleep(duration)
    stop()

def left(duration, speed=70):
    pwmL.ChangeDutyCycle(speed)
    pwmR.ChangeDutyCycle(speed)
    GPIO.output(leftForward, GPIO.HIGH)
    GPIO.output(leftBackward, GPIO.LOW)
    GPIO.output(rightForward, GPIO.LOW)
    GPIO.output(rightBackward, GPIO.HIGH)
    time.sleep(duration)
    stop()

# Snake movement routine
def snake_movement():
    try:
        forward(LONG_FORWARD_TIME, SPEED_NORMAL)
        left(TURN_TIME, SPEED_TURN)
        forward(SHORT_FORWARD_TIME, SPEED_NORMAL)
        left(TURN_TIME, SPEED_TURN)
        forward(LONG_FORWARD_TIME, SPEED_NORMAL)
        right(TURN_TIME, SPEED_TURN)
        forward(SHORT_FORWARD_TIME, SPEED_NORMAL)
        right(TURN_TIME, SPEED_TURN)
        forward(LONG_FORWARD_TIME, SPEED_NORMAL)
        forward(LONG_FORWARD_TIME, SPEED_NORMAL)
        left(TURN_TIME, SPEED_TURN)
        forward(SHORT_FORWARD_TIME, SPEED_NORMAL)
        left(TURN_TIME, SPEED_TURN)
        forward(LONG_FORWARD_TIME, SPEED_NORMAL)
        right(TURN_TIME, SPEED_TURN)
        forward(SHORT_FORWARD_TIME, SPEED_NORMAL)
        right(TURN_TIME, SPEED_TURN)

        forward(LONG_FORWARD_TIME/2, SPEED_NORMAL)
        left(TURN_TIME, SPEED_TURN)
        forward(SHORT_FORWARD_TIME, SPEED_NORMAL)
        left(TURN_TIME, SPEED_TURN)
        forward(LONG_FORWARD_TIME/2, SPEED_NORMAL)

        right(TURN_TIME, SPEED_TURN)
        forward(SHORT_FORWARD_TIME, SPEED_NORMAL)
        right(TURN_TIME, SPEED_TURN)
        forward(LONG_FORWARD_TIME, SPEED_NORMAL)
        forward(LONG_FORWARD_TIME, SPEED_NORMAL)
        left(TURN_TIME, SPEED_TURN)
        forward(SHORT_FORWARD_TIME, SPEED_NORMAL)
        left(TURN_TIME, SPEED_TURN)
        forward(LONG_FORWARD_TIME, SPEED_NORMAL)
        right(TURN_TIME, SPEED_TURN)
        forward(SHORT_FORWARD_TIME, SPEED_NORMAL)
        right(TURN_TIME, SPEED_TURN)

    finally:
        stop()
        pwmL.stop()
        pwmR.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    print("Running snake pattern...")
    snake_movement()
