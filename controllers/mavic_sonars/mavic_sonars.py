import numpy as np
from controller import Robot, Supervisor, DistanceSensor

x_bounds = [-9.81, 9.89]
y_bounds = [-3.5, 12.4]

xc_1 = np.array([9.89, 12.4])
xc_2 = np.array([9.89, -3.5])
xc_3 = np.array([-9.81, 12.4])
xc_4 = np.array([-9.81, -3.5])
xcs = [xc_1, xc_2, xc_3, xc_4]


def in_box(x, y):
    x_min, x_max = x_bounds
    y_min, y_max = y_bounds

    return x >= x_min and x <= x_max and y >= y_min and y <= y_max


def get_intersections(pose_0, pose_1, r0, r1):
    x0, y0, z0 = pose_0
    x1, y1, z1 = pose_1
    d = np.sqrt((x1 - x0)**2 + (y1 - y0)**2)
    
    # non intersecting
    if d > r0 + r1:
        print('circles do not intersect')
        return None
    # One circle within other
    if d < abs(r0 - r1):
        print('one circle is within another')
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        print('the circles are coincident')
        return None
    else:
        a = (r0**2 - r1**2 + d**2) / (2 * d)
        h = np.sqrt(r0**2 - a**2)
        x2 = x0 + a * (x1 - x0) / d   
        y2 = y0 + a * (y1 - y0) / d   
        x3 = x2 + h * (y1 - y0) / d     
        y3 = y2 - h * (x1 - x0) / d 

        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) /d
        
        if in_box(x3, y3):
            return x3, y3
        
        return x4, y4


print('sonar is working')

robot = Supervisor()
time_step = int(robot.getBasicTimeStep())

#Getting sonars
sonar_1 = robot.getDevice("sonar_1")
sonar_2 = robot.getDevice("sonar_2")
sonar_3 = robot.getDevice("sonar_3")
sonar_4 = robot.getDevice("sonar_4")
sonars = [sonar_1, sonar_2, sonar_3, sonar_4]

#Enabling sonars
for sonar in sonars:
    sonar.enable(time_step)


robot_node = robot.getSelf()
sensor_slot = robot_node.getField("sensorSlot")

sonar_nodes = [sensor_slot.getMFNode(i) for i in range(len(sonars))]
sonar_poses = [sonar_node.getPosition() for sonar_node in sonar_nodes]

while robot.step(time_step) != -1:
    to_print = "Sonar values: "
    rs = []

    for sonar, i in zip(sonars, range(len(sonars))):
        sampling_period = sonar.getSamplingPeriod()
        value = sonar.getValue()
        to_print += str(value) + " "
        rs.append(value)

    #print(to_print)

    
    