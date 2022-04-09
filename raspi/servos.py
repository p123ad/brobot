from adafruit_servokit import ServoKit
import time
kit = ServoKit(channels=16)

print("Starting")


head = kit.servo[4]
#############
#

# Right Knee - servo[0]
# Right Hip - servo[1]
# Left Knee - servo[2]
# Left Hip - servo[3]

# Fahre alle servos so, dass sie direkt gerade lang sind

# time.sleep(1)
# kit.servo[0].angle = 90
# time.sleep(1)
# kit.servo[1].angle = 90
# time.sleep(1)
# kit.servo[2].angle = 90
# time.sleep(1)
# kit.servo[3].angle = 86

def shakeHead(headServo):
    print("Shake Head 1x")
    headServo.angle = 100    # Mittige Position
    time.sleep(0.3)
    headServo.angle = 140
    time.sleep(0.3)
    headServo.angle = 60
    time.sleep(0.4)
    headServo.angle = 100
    time.sleep(0.2)
    headServo.angle = None
    


if __name__ == "__main__":
    shakeHead(head)
    shakeHead(head)