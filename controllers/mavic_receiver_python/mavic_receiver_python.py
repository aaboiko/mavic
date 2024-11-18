import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
import struct
from time import time

from controller import Robot, Supervisor, Receiver
from controller import Camera
from controller import Compass
from controller import GPS
from controller import Gyro
from controller import InertialUnit
from controller import Keyboard
from controller import LED
from controller import Motor


SOUND_SPEED = 340

EMITTER_POSES = [
    np.array([9.89, -12.4, 7.0]),
    np.array([9.89, 3.5, 7.0]),
    np.array([-9.81, -12.4, 7.0]),
    np.array([-9.81, 3.5, 7.0])
]

def CLAMP(value, low, high):
    if value < low:
        return low
    else:
        if value > high:
            return high
        else:
            return value


def in_box(point, x_bounds, y_bounds, z_bounds):
    x, y, z = point
    x_min, x_max = x_bounds
    y_min, y_max = y_bounds
    z_min, z_max = z_bounds

    return x >= x_min and x <= x_max and y >= y_min and y <= y_max and z >= z_min and z <= z_max


def get_intersections(P1, P2, P3, r1, r2, r3):
    p1 = np.array([0, 0, 0])
    p2 = np.array([P2[0] - P1[0], P2[1] - P1[1], P2[2] - P1[2]])
    p3 = np.array([P3[0] - P1[0], P3[1] - P1[1], P3[2] - P1[2]])
    v1 = p2 - p1
    v2 = p3 - p1

    Xn = (v1) / np.linalg.norm(v1)

    tmp = np.cross(v1, v2)

    Zn = (tmp) / np.linalg.norm(tmp)

    Yn = np.cross(Xn, Zn)

    i = np.dot(Xn, v2)
    d = np.dot(Xn, v1)
    j = np.dot(Yn, v2)

    X = ((r1**2) - (r2**2) + (d**2)) / (2*d)
    Y = (((r1**2) - (r3**2) + (i**2) + (j**2)) / (2*j))-((i/j)*(X))
    Z1 = np.sqrt(max(0, r1**2 - X**2 - Y**2))
    Z2 = -Z1

    K1 = P1 + X * Xn + Y * Yn + Z1 * Zn
    K2 = P1 + X * Xn + Y * Yn + Z2 * Zn

    x_bounds = [EMITTER_POSES[2][0], EMITTER_POSES[0][0]]
    y_bounds = [EMITTER_POSES[0][1], EMITTER_POSES[1][1]]
    z_bounds = [0, EMITTER_POSES[0][2]]

    if in_box(K1, x_bounds, y_bounds, z_bounds):
        return K1

    return K2


def discretize(value):
    step = SOUND_SPEED / 1000000
    units = int(value / step)
    return units * step


def main():
    robot = Supervisor()
    print('robot initiated')
    timestep = int(robot.getBasicTimeStep())
    print('timestep: ' + str(timestep))

    emitter_1 = robot.getDevice("emitter_1")
    emitter_2 = robot.getDevice("emitter_2")
    emitter_3 = robot.getDevice("emitter_3")
    emitter_4 = robot.getDevice("emitter_4")
    emitters = [emitter_1, emitter_2, emitter_3, emitter_4]
    
    robot_node = robot.getSelf()
    sensor_slot = robot_node.getField("cameraSlot")

    emitter_nodes = [sensor_slot.getMFNode(i) for i in range(len(emitters))]
    emitter_poses = [emitter_node.getPosition() for emitter_node in emitter_nodes]

    camera = robot.getDevice('camera')
    camera.enable(timestep)
    front_left_led = robot.getDevice("front left led")
    front_right_led = robot.getDevice("front right led")
    imu = robot.getDevice("inertial unit")
    imu.enable(timestep)
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    compass = robot.getDevice("compass")
    compass.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)

    keyboard = Keyboard()
    keyboard.enable(timestep)

    camera_roll_motor = robot.getDevice("camera roll")
    camera_pitch_motor = robot.getDevice("camera pitch")

    front_left_motor = robot.getDevice("front left propeller")
    front_right_motor = robot.getDevice("front right propeller")
    rear_left_motor = robot.getDevice("rear left propeller")
    rear_right_motor = robot.getDevice("rear right propeller")
    motors = [front_left_motor, front_right_motor, rear_left_motor, rear_right_motor]

    for motor in motors:
        motor.setPosition(np.inf)
        motor.setVelocity(1.0)

    print("Start the drone...\n")

    while robot.step(timestep) != -1:
        if robot.getTime() > 1.0:
            break

    k_vertical_thrust = 68.5
    k_vertical_offset = 0.6
    k_vertical_p = 3.0
    k_roll_p = 50.0
    k_pitch_p = 30.0

    target_altitude = 1.0

    #Dealing with receiver
    receiver_1 = robot.getDevice("receiver_1")
    receiver_2 = robot.getDevice("receiver_2")
    receiver_3 = robot.getDevice("receiver_3")
    receiver_4 = robot.getDevice("receiver_4")
    receivers = [receiver_1, receiver_2, receiver_3, receiver_4]

    for receiver in receivers:
        receiver.enable(timestep)

    while robot.step(timestep) != -1:
        time = robot.getTime()

        roll, pitch, yaw = imu.getRollPitchYaw()
        altitude = gps.getValues()[2]
        roll_velocity, pitch_velocity, yaw_velocity = gyro.getValues()

        led_state = int(time) % 2
        front_left_led.set(led_state)
        front_right_led.set(1 - led_state)

        camera_roll_motor.setPosition(-0.115 * roll_velocity)
        camera_pitch_motor.setPosition(-0.1 * pitch_velocity)
        
        #Receiving data from emitters
        
        '''for receiver in receivers:
            if receiver.getQueueLength() > 0:
                message = receiver.nextPacket()
                channel = receiver.getChannel()
                print('channel: ' + str(channel) + ', message: ' + str(message))'''
        
        #Localization
        robot_position = np.array(robot_node.getPosition())
        distances = [discretize(np.linalg.norm(emitter_pose - robot_position)) for emitter_pose in EMITTER_POSES]
        
        intersection_points = []
        for i in range(len(EMITTER_POSES) - 2):
            for j in range(i + 1, len(EMITTER_POSES) - 1):
                for k in range(j + 1, len(EMITTER_POSES)):
                    intersection_point = get_intersections(EMITTER_POSES[i], EMITTER_POSES[j], EMITTER_POSES[k], distances[i], distances[j], distances[k])
                    intersection_points.append(intersection_point)

        errors = [np.linalg.norm(robot_position - point) for point in intersection_points]
        print('errors: ' + str(errors))

        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0

        key = keyboard.getKey()

        while key > 0:
            if key == Keyboard.UP:
                pitch_disturbance = -2.0
            elif key == Keyboard.DOWN:
                pitch_disturbance = 2.0
            elif key == Keyboard.RIGHT:
                yaw_disturbance = -1.3
            elif key == Keyboard.LEFT:
                yaw_disturbance = 1.3
            elif key == Keyboard.SHIFT + Keyboard.RIGHT:
                roll_disturbance = -1.0
            elif key == Keyboard.SHIFT + Keyboard.LEFT:
                roll_disturbance = 1.0
            elif key == Keyboard.SHIFT + Keyboard.UP:
                target_altitude += 0.05
                print("target altitude: " + str(target_altitude))
            elif key == Keyboard.SHIFT + Keyboard.DOWN:
                target_altitude -= 0.05
                print("target altitude: " + str(target_altitude))

            key = keyboard.getKey()

        roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance
        pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance
        yaw_input = yaw_disturbance
        clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)
        vertical_input = k_vertical_p * clamped_difference_altitude**3

        front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
        front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
        rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
        rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input

        front_left_motor.setVelocity(front_left_motor_input)
        front_right_motor.setVelocity(-front_right_motor_input)
        rear_left_motor.setVelocity(-rear_left_motor_input)
        rear_right_motor.setVelocity(rear_right_motor_input)

if __name__ == '__main__':
    main()