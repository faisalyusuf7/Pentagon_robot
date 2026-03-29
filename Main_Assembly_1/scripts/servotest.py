# import time
# from adafruit_servokit import ServoKit

# # Set channels to the number of servo channels on your kit.
# # 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
# kit = ServoKit(channels=16)
# kit.servo[0].actuation_range = 180

# kit.servo[0].angle = 0



# time.sleep(1)

from adafruit_servokit import ServoKit

# Tune these values to match your exact 270-degree servo.
# Many 270 servos need a wider pulse range than ServoKit defaults.
SERVO_CHANNEL = 0
ACTUATION_RANGE = 270
MIN_PULSE_US = 500
MAX_PULSE_US = 2500

kit = ServoKit(channels=16)
servo = kit.servo[SERVO_CHANNEL]
servo.actuation_range = ACTUATION_RANGE
servo.set_pulse_width_range(MIN_PULSE_US, MAX_PULSE_US)

print("Press w + ENTER -> 0 deg")
print("Press s + ENTER -> 270 deg")
print("Press m + ENTER -> 135 deg (middle)")
print("Press q + ENTER -> quit")

while True:
    key = input(">> ").strip().lower()

    if key == "w":
        servo.angle = 5
        print("Servo -> 0 deg")

    elif key == "s":
        servo.angle = 270
        print("Servo -> 270 deg")

    elif key == "m":
        servo.angle = 135
        print("Servo -> 135 deg")

    elif key == "q":
        print("Exiting")
        break

# import time
# from adafruit_servokit import ServoKit

# kit = ServoKit(channels=16)
# kit.servo[0].actuation_range = 180

# print("Servo cycling: 0° ↔ 180° every 4 seconds")
# print("Press Ctrl+C to stop")

# try:
#     while True:
#         # Go UP
#         kit.servo[0].angle = 180
#         print("Servo → 180°")
#         time.sleep(2)

#         # Go DOWN
#         kit.servo[0].angle = 0
#         print("Servo → 0°")
#         time.sleep(3)

# except KeyboardInterrupt:
#     print("\nStopped by user")
