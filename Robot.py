#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor
from pybricks.parameters import Port, Color, Stop, Direction, Button
from pybricks.tools import wait
import time

ev3 = EV3Brick()
gripper_motor = Motor(Port.A)

# Configure the elbow motor. It has an 8-teeth and a 40-teeth gear
# connected to it. We would like positive speed values to make the
# arm go upward. This corresponds to counterclockwise rotation
# of the motor.
elbow_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE, [8, 40])

# Configure the motor that rotates the base. It has a 12-teeth and a
# 36-teeth gear connected to it. We would like positive speed values
# to make the arm go away from the Touch Sensor. This corresponds
# to counterclockwise rotation of the motor.
base_motor = Motor(Port.C, Direction.COUNTERCLOCKWISE, [12, 36])

# Limit the elbow and base accelerations. This results in
# very smooth motion. Like an industrial robot.
elbow_motor.control.limits(speed=60, acceleration=120)
base_motor.control.limits(speed=60, acceleration=120)

base_switch = TouchSensor(Port.S1)
elbow_sensor = ColorSensor(Port.S2)

# Initialize the elbow. First make it go down for one second.
# Then make it go upwards slowly (15 degrees per second) until
# the Color Sensor detects the white beam. Then reset the motor
# angle to make this the zero point. Finally, hold the motor
# in place so it does not move.


# def get_current_time(): 
#     return time.localtime().tm_hour, time.localtime().tm_min

# start_hour = 14
# start_minute = 24

# current_hour, current_minute = get_current_time()

# while not (current_hour == start_hour and current_minute == start_minute):
#     wait(1000)
#     print(get_current_time())
#     current_hour, current_minute = get_current_time()
elbow_motor.run_time(-30, 1000)

elbow_motor.run(15)
while elbow_sensor.reflection() > 0:
    wait(10)
    
elbow_motor.run_time(20,500)
elbow_motor.reset_angle(0)
elbow_motor.hold()

# Initialize the base. First rotate it until the Touch Sensor
# in the base is pressed. Reset the motor angle to make this
# the zero point. Then hold the motor in place so it does not move.
base_motor.run(-60)
while not base_switch.pressed():
    wait(10)
base_motor.reset_angle(0)
base_motor.hold()

# Initialize the gripper. First rotate the motor until it stalls.
# Stalling means that it cannot move any further. This position
# corresponds to the closed position. Then rotate the motor
# by 90 degrees such that the gripper is open.
gripper_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=40)
gripper_motor.reset_angle(0)
gripper_motor.run_target(200, -90)


def choose_parking():
    if Button.LEFT in ev3.buttons.pressed():
        return 205
    if Button.RIGHT in ev3.buttons.pressed():
        return 113
    if Button.CENTER in ev3.buttons.pressed():
        return 157

def robot_pick(position):
    # This function makes the robot base rotate to the indicated
    # position. There it lowers the elbow, closes the gripper, and
    # raises the elbow to pick up the object.

    # Rotate to the pick-up position.
    base_motor.run_target(60, position)
    # Lower the arm.
    elbow_motor.run_until_stalled(-40, then=Stop.HOLD, duty_limit=10) #-29
    # Close the gripper to grab the wheel stack.
    gripper_motor.run_until_stalled(200, then=Stop.HOLD, duty_limit=40)
    # Raise the arm to lift the wheel stack.
    elbow_motor.run_target(50, 0) #40


def robot_release(position):
    # This function makes the robot base rotate to the indicated
    # position. There it lowers the elbow, opens the gripper to
    # release the object. Then it raises its arm again.

    # Rotate to the drop-off position.
    base_motor.run_target(60, position)
    # Lower the arm to put the wheel stack on the ground.
    elbow_motor.run_until_stalled(-40, then=Stop.HOLD, duty_limit=10)
    # Open the gripper to release the wheel stack.
    gripper_motor.run_target(200, -90)
    # Raise the arm.
    elbow_motor.run_target(40, 0)



# Play three beeps to indicate that the initialization is complete.
for i in range(1):
    ev3.speaker.beep()
    wait(10)

# Define the three destinations for picking up and moving the wheel stacks.
LEFT = 205
MIDDLE = 157
RIGHT = 113
ELNOUR = 10

#Color calibration
robot_pick(RIGHT)
wait(100)
rightParking = elbow_sensor.color()
rightParkingReflectionSmall = elbow_sensor.reflection()
print(elbow_sensor.color())
print(elbow_sensor.reflection()) #två ord bara
robot_release(RIGHT)
ev3.speaker.beep()
wait(1000)
robot_pick(RIGHT)
wait(100)
rightParkingReflectionLarge = elbow_sensor.reflection()
robot_release(RIGHT)
rightParkingReflection = (rightParkingReflectionLarge + rightParkingReflectionSmall) / 2

robot_pick(MIDDLE)
wait(100)
middleParking = elbow_sensor.color()
middleParkingReflectionSmall = elbow_sensor.reflection()
print(elbow_sensor.color())
print(elbow_sensor.reflection())
robot_release(MIDDLE)
ev3.speaker.beep()
wait(1000)
robot_pick(MIDDLE)
wait(100)
middleParkingReflectionLarge = elbow_sensor.reflection()
robot_release(MIDDLE)
middleParkingReflection = (middleParkingReflectionLarge + middleParkingReflectionSmall) / 2

robot_pick(LEFT)
wait(100)
leftParking = elbow_sensor.color()
leftParkingReflectionSmall = elbow_sensor.reflection()
print(elbow_sensor.color())
robot_release(LEFT)
ev3.speaker.beep()
wait(1000)
robot_pick(LEFT)
wait(100)
leftParkingReflectionLarge = elbow_sensor.reflection()
robot_release(LEFT)
leftParkingReflection = (leftParkingReflectionLarge + leftParkingReflectionSmall) / 2

# This is the main part of the program. It is a loop that repeats endlessly.
#
# First, the robot moves the object on the left towards the middle.
# Second, the robot moves the object on the right towards the left.
# Finally, the robot moves the object that is now in the middle, to the right.
#
# Now we have a wheel stack on the left and on the right as before, but they
# have switched places. Then the loop repeats to do this over and over.
parkingSpots = {
    "middle" : None,
    "left" : None,
    "right" : None
}


while True:
    #elbow_sensor()

    # Move a wheel stack from the left to the middle.
    robot_pick(ELNOUR)
    wait(100)
    currentColor = elbow_sensor.color()
    currentReflection = elbow_sensor.reflection()
    ev3.screen.print(currentColor)

    if currentColor == middleParking:
        if currentReflection < middleParkingReflection:
            ev3.screen.print("SMALL")
        else:
            ev3.screen.print("LARGE")
        if parkingSpots["middle"] is None:
            while not ev3.buttons.pressed():
                wait(2000)
            parkingSpots["middle"] = choose_parking()
        robot_release(parkingSpots["middle"])
    elif currentColor == rightParking:
        if currentReflection < rightParkingReflection:
            ev3.screen.print("SMALL")
        else:
            ev3.screen.print("LARGE")
        if parkingSpots["right"] is None:
            while not ev3.buttons.pressed():
                wait(2000)
            parkingSpots["right"] = choose_parking()
        robot_release(parkingSpots["right"])
    elif currentColor == leftParking:
        if currentReflection < leftParkingReflection:
            ev3.screen.print("SMALL")
        else:
            ev3.screen.print("LARGE")
        if parkingSpots["left"] is None:
            while not ev3.buttons.pressed():
                wait(2000)
            parkingSpots["left"] = choose_parking()
        robot_release(parkingSpots["left"])
    elif currentColor is Color.BLACK:
        wait(10000)
    else:
        robot_release(ELNOUR)

    ev3.screen.clear()