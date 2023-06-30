import pybullet as p
import time
import pybullet_data
import yaml
from cbs import cbs
import math

def createBoundaries(length, width):
    for i in range(length):
        p.loadURDF("cube.urdf", [i, -1, 0.5])
        p.loadURDF("cube.urdf", [i, width, 0.5])
    for i in range(width):
        p.loadURDF("cube.urdf", [-1, i, 0.5])
        p.loadURDF("cube.urdf", [length, i, 0.5])
    p.loadURDF("cube.urdf", [length, -1, 0.5])
    p.loadURDF("cube.urdf", [length, width, 0.5])
    p.loadURDF("cube.urdf", [-1, width, 0.5])
    p.loadURDF("cube.urdf", [-1, -1, 0.5])

def checkPosWithBias(Pos, goal, bias):
    if(Pos[0] < goal[0] + bias and Pos[0] > goal[0] - bias and Pos[1] < goal[1] + bias and Pos[1] > goal[1] - bias):
        return True
    else:
        return False

def check_goal_reached(agents,goals):
    for i in agents:
        pos = p.getBasePositionAndOrientation(i)[0]
        goal = goals[i]
        if(not checkPosWithBias(pos, goal, 0.2)):
            return False
    return True

def check_next_reached(agents, schedule, index):
    for i in agents:
        pos = p.getBasePositionAndOrientation(i)[0]
        if(index >= len(schedule[i])):
            continue
        if (not checkPosWithBias(pos, [schedule[i][index]["x"], schedule[i][index]["y"]], 0.2)):
            return False
    return True

def set_velocity(agent, schedule, index):
    speed = 20
    forward = 0
    basePos = p.getBasePositionAndOrientation(agent)
    x = basePos[0][0]
    y = basePos[0][1]
    if(index < len(schedule[agent])):
        next = schedule[agent][index]
    else:
        return 0, 0
    # print(basePos[0])
    print(agent, x, y, next)

    Orientation = list(p.getEulerFromQuaternion(basePos[1]))
    goal_direct = math.atan2((next["y"] - y), (next["x"] - x))
    if goal_direct < 0 and goal_direct < -math.pi / 3:
        goal_direct = goal_direct + math.pi * 2
    if Orientation[2] < 0 and Orientation[2] < -math.pi/3:
        Orientation[2] = Orientation[2] + math.pi*2

    print(Orientation[2], goal_direct)

    if (abs(Orientation[2] - goal_direct) <= 0.2):
        turn = 0
        direct = True
    else:
        forward = 0
        turn = goal_direct - Orientation[2]
        direct = False

    if (direct is True):
        forward = 1
        turn = 0

    if (checkPosWithBias(basePos[0], [next["x"], next["y"]], 0.2)):
        forward = 0
        turn = 0

    if(turn != 0):
        speed = 5
    rightWheelVelocity = (forward + turn) * speed
    leftWheelVelocity = (forward - turn) * speed
    return leftWheelVelocity, rightWheelVelocity


def read_input(yaml_file, env_loaded):
    agents = []
    goals = {}
    with open(yaml_file, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)
        if(env_loaded is True):
            for i in param["agents"]:
                goals[i["name"]] = i["goal"]
            return None, goals, True
        for i in param["agents"]:
            startPosition = (i["start"][0], i["start"][1], 0)
            boxId = p.loadURDF("data/turtlebot.urdf", startPosition, startOrientation, globalScaling=1)
            agents.append(boxId)
            goals[boxId] = i["goal"]
        dimensions = param["map"]["dimensions"]
        p.resetDebugVisualizerCamera(cameraDistance=dimensions[0] * 1, cameraYaw=0, cameraPitch=-89,
                                     cameraTargetPosition=[dimensions[0] / 2, dimensions[1] / 2, 0])

        createBoundaries(dimensions[0], dimensions[1])
        if env_loaded is False:
            for i in param["map"]["obstacles"]:
                p.loadURDF("cube.urdf", [i[0], i[1], 0.5])
    return agents, goals, True

def read_output(output_yaml_file):
    with open(output_yaml_file, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)
    return param["schedule"]

def navigation(agents, goals, schedule):
    index = 0
    while (not check_goal_reached(agents, goals)):
        time.sleep(1. / 240.)


        for i in agents:
            leftWheelVelocity, rightWheelVelocity = set_velocity(i, schedule, index)
            print(i, leftWheelVelocity, rightWheelVelocity)
            p.setJointMotorControl2(i, 0, p.VELOCITY_CONTROL, targetVelocity=leftWheelVelocity, force=1000)
            p.setJointMotorControl2(i, 1, p.VELOCITY_CONTROL, targetVelocity=rightWheelVelocity, force=1000)
        if(check_next_reached(agents, schedule, index)):
            index+=1

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setRealTimeSimulation(1)
p.setGravity(0, 0, -10)
startOrientation = p.getQuaternionFromEuler([0,0,0])
global env_loaded
env_loaded = False
agents, goals, env_loaded = read_input("scene/room_scene_4_bots.yaml", env_loaded)
cbs.main("scene/room_scene_4_bots.yaml", "output.yaml")
schedule = read_output("output.yaml")
navigation(agents, goals, schedule)

# _, goals, env_loaded = read_input("input3.yaml", env_loaded)
# cbs.main("input3.yaml", "output.yaml")
# schedule = read_output("output.yaml")
# navigation(agents, goals, schedule)






