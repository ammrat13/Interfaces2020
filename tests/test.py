import Jetson.GPIO as GPIO
from interfaces import wheel_encoders

if __name__ == "__main__":
    while True:
        print(wheel_encoders.pulsein(18, GPIO.HIGH))