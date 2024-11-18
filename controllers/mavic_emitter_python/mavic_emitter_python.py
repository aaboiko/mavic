from controller import Robot, Supervisor, Emitter
import numpy as np
import struct
from time import time
    

def main():
    robot = Supervisor()
    time_step = int(robot.getBasicTimeStep())

    emitter_1 = robot.getDevice("emitter_1")
    emitter_2 = robot.getDevice("emitter_2")
    emitter_3 = robot.getDevice("emitter_3")
    emitter_4 = robot.getDevice("emitter_4")
    emitters = [emitter_1, emitter_2, emitter_3, emitter_4]

    robot_node = robot.getSelf()
    sensor_slot = robot_node.getField("sensorSlot")

    emitter_nodes = [sensor_slot.getMFNode(i) for i in range(len(emitters))]
    emitter_poses = [emitter_node.getPosition() for emitter_node in emitter_nodes]

    while robot.step(time_step) != -1:
        for i in range(len(emitters)):
            moment = int(time() * 1000)
            x, y, z = emitter_poses[i]
            message = struct.pack("fffQ", x, y, z, moment)

            emitters[i].send(message)

main()