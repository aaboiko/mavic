import numpy as np
import cv2
import math
import matplotlib.pyplot as plt

from controller import Robot
from controller import Camera
from controller import Compass
from controller import GPS
from controller import Gyro
from controller import InertialUnit
from controller import Keyboard
from controller import LED
from controller import Motor


def CLAMP(value, low, high):
    if value < low:
        return low
    else:
        if value > high:
            return high
        else:
            return value
        

def imageToMat(image):
    res = np.zeros((400, 240, 3))
    it = iter(image)
    i = 0
    j = 0

    for item in zip(it, it, it, it):
        res[i,j] = item[0:3]
        i += 1

        if i == 400:
            j += 1
            i = 0
    
    return res
        

def main():
    robot = Robot()
    print('robot initiated')
    timestep = int(robot.getBasicTimeStep())
    print('timestep: ' + str(timestep))

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

    print('keyboard init...')
    keyboard = Keyboard()
    keyboard.enable(timestep)
    print('keyboard enabled')

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

    print("You can control the drone with your computer keyboard:\n")
    print("- 'up': move forward.\n")
    print("- 'down': move backward.\n")
    print("- 'right': turn right.\n")
    print("- 'left': turn left.\n")
    print("- 'shift + up': increase the target altitude.\n")
    print("- 'shift + down': decrease the target altitude.\n")
    print("- 'shift + right': strafe right.\n")
    print("- 'shift + left': strafe left.\n")

    k_vertical_thrust = 68.5
    k_vertical_offset = 0.6
    k_vertical_p = 3.0
    k_roll_p = 50.0
    k_pitch_p = 30.0

    target_altitude = 1.0

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

        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0

        #Dealing with aruco
        this_aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        this_aruco_parameters = cv2.aruco.DetectorParameters_create()

        image = camera.getImage()
        img = np.frombuffer(image, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))  # RGBA format
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, this_aruco_dictionary, parameters=this_aruco_parameters)
        #print('ids: ' + str(ids))

        marker_length = 0.5

        width = camera.getWidth()
        height = camera.getHeight()
        fov = camera.getFov()  

        fx = width / (2 * math.tan(fov / 2))
        fy = height / (2 * math.tan(fov / 2))
        cx = width / 2.0
        cy = height / 2.0

        camera_matrix = np.array([[fx, 0, cx],
                           [0, fy, cy],
                           [0, 0, 1]], dtype=float)
        
        distortion_coeffs = np.zeros((4, 1))

        if ids is not None:
            for i in range(len(ids)):
                rvec, tvec, marker_pts = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length, camera_matrix, distortion_coeffs)
                cv2.aruco.drawDetectedMarkers(img, corners, ids)
                cv2.drawFrameAxes(img, camera_matrix, distortion_coeffs, rvec[0], tvec[0], 0.1) 

                # Print the position and orientation
                #print(f'Marker ID: {ids[i][0]}')
                #print(f'Translation Vector (tvec): {tvec[0]}')
                #print(f'Rotation Vector (rvec): {rvec[0]}')
                #print('rvec: ' + str(rvec) + ', tvec: ' + str(tvec) + ', marker_pts: ' + str(marker_pts))
        
        ###################################

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