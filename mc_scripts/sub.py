#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

# Define motor control pins
MOTOR_PIN1 = 17  # Adjust according to your setup
MOTOR_PIN2 = 27

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_PIN1, GPIO.OUT)
GPIO.setup(MOTOR_PIN2, GPIO.OUT)

def motor_callback(msg):
    command = msg.data.lower()
    rospy.loginfo(f"Received command: {command}")

    if command == "forward":
        GPIO.output(MOTOR_PIN1, GPIO.HIGH)
        GPIO.output(MOTOR_PIN2, GPIO.LOW)
    elif command == "backward":
        GPIO.output(MOTOR_PIN1, GPIO.LOW)
        GPIO.output(MOTOR_PIN2, GPIO.HIGH)
    elif command == "stop":
        GPIO.output(MOTOR_PIN1, GPIO.LOW)
        GPIO.output(MOTOR_PIN2, GPIO.LOW)
    else:
        rospy.logwarn("Unknown command received!")

def motor_controller():
    rospy.init_node("motor_subscriber", anonymous=True)
    rospy.Subscriber("/motor_command", String, motor_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        motor_controller()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()  # Cleanup on exit
