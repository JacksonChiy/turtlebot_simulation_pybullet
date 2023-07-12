import pybullet as p
import time
import pybullet_data
import yaml
from cbs import cbs
import math
import threading

def createBoundaries(length, width):
    """
        create rectangular boundaries with length and width

        Args:

        length: integer

        width: integer
    """
    for i in range(length):
        p.loadURDF("data/rectangle_horizontal.urdf", [i, -1, 0.5])
        p.loadURDF("data/rectangle_horizontal.urdf", [i, width, 0.5])
    for i in range(width):
        p.loadURDF("data/rectangle_vertical.urdf", [-1, i, 0.5])
        p.loadURDF("data/rectangle_vertical.urdf", [length, i, 0.5])
    p.loadURDF("data/rectangle_corner.urdf", [length, -1, 0.5], p.getQuaternionFromEuler([0, 0, -math.pi/2]))
    p.loadURDF("data/rectangle_corner.urdf", [length, width, 0.5])
    p.loadURDF("data/rectangle_corner.urdf", [-1, width, 0.5], p.getQuaternionFromEuler([0, 0, math.pi/2]))
    p.loadURDF("data/rectangle_corner.urdf", [-1, -1, 0.5], p.getQuaternionFromEuler([0, 0, math.pi]))

def checkPosWithBias(Pos, goal, bias):
    """
        Check if pos is at goal with bias

        Args:

        Pos: Position to be checked, [x, y]

        goal: goal position, [x, y]

        bias: bias allowed

        Returns:

        True if pos is at goal, False otherwise
    """
    if(Pos[0] < goal[0] + bias and Pos[0] > goal[0] - bias and Pos[1] < goal[1] + bias and Pos[1] > goal[1] - bias):
        return True
    else:
        return False

def navigation(agent, goal, schedule):
    """
        Set velocity for robots to follow the path in the schedule.

        Args:

        agents: array containing the boxID for each agent

        schedule: dictionary with boxID as key and path to the goal as list for each robot.

        index: index of the current position in the path.

        Returns:

        Leftwheel and rightwheel velocity.
    """
    basePos = p.getBasePositionAndOrientation(agent)
    index = 0
    while(not checkPosWithBias(basePos[0], goal, 0.2)):
        basePos = p.getBasePositionAndOrientation(agent)
        next = [schedule[index]["x"], schedule[index]["y"]]
        if(checkPosWithBias(basePos[0], next, 0.3)):
            index = index + 1
        if(index == len(schedule)):
            p.setJointMotorControl2(agent, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=100)
            p.setJointMotorControl2(agent, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=100)
            break
        x = basePos[0][0]
        y = basePos[0][1]
        Orientation = list(p.getEulerFromQuaternion(basePos[1]))[2]
        goal_direction = math.atan2((schedule[index]["y"] - y), (schedule[index]["x"] - x))
        if(Orientation < 0):
            Orientation = Orientation + 2 * math.pi
        if(goal_direction < 0):
            goal_direction = goal_direction + 2 * math.pi

        theta = goal_direction - Orientation

        if theta < 0 and abs(theta) > abs(theta + 2 * math.pi):
            theta = theta + 2 * math.pi
        elif theta > 0 and abs(theta - 2 * math.pi) < theta:
            theta = theta - 2 * math.pi

        # print("theta:", theta)

        current = [x, y]

        distance = math.dist(current, next)

        k1 = 5
        k2 = 20

        # linear = k1 * (distance) * math.cos(theta) + 1.0
        A=20
        linear =min(A*math.exp(k1 * distance), 30.0)
        angular = k2 * theta

        rightWheelVelocity = linear + angular
        leftWheelVelocity = linear - angular

        p.setJointMotorControl2(agent, 0, p.VELOCITY_CONTROL, targetVelocity=leftWheelVelocity, force=100)
        p.setJointMotorControl2(agent, 1, p.VELOCITY_CONTROL, targetVelocity=rightWheelVelocity, force=100)
        # time.sleep(0.001)



def read_input(yaml_file, env_loaded):
    """
        Read input file, load boundaries, robot and obstacles, set up goals dictionary

        Args:

        yaml_file: input yaml file

        env_loaded: True or false, check if the boundaries, robots and obstacles have been loaded before

        Returns:

        agents: list of boxID
        goals: dictionary of goal position for each robot.
        env_loaded: True
    """
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
        p.resetDebugVisualizerCamera(cameraDistance=dimensions[0] * 0.7, cameraYaw=0, cameraPitch=-89,
                                     cameraTargetPosition=[dimensions[0] / 2, dimensions[1] / 2, 0])

        createBoundaries(dimensions[0], dimensions[1])
        if env_loaded is False:
            for i in param["map"]["obstacles"]["horizontal"]:
                p.loadURDF("data/rectangle_horizontal.urdf", [i[0], i[1], 0.5])
            for i in param["map"]["obstacles"]["vertical"]:
                p.loadURDF("data/rectangle_vertical.urdf", [i[0], i[1], 0.5])
            if(param["map"]["obstacles"]["top_left_corner"] is not None):
                for i in param["map"]["obstacles"]["top_left_corner"]:
                    p.loadURDF("data/rectangle_corner.urdf", [i[0], i[1], 0.5], p.getQuaternionFromEuler([0, 0, math.pi/2]))
            if (param["map"]["obstacles"]["top_right_corner"] is not None):
                for i in param["map"]["obstacles"]["top_right_corner"]:
                    p.loadURDF("data/rectangle_corner.urdf", [i[0], i[1], 0.5])
            if (param["map"]["obstacles"]["bottom_left_corner"] is not None):
                for i in param["map"]["obstacles"]["bottom_left_corner"]:
                    p.loadURDF("data/rectangle_corner.urdf", [i[0], i[1], 0.5], p.getQuaternionFromEuler([0, 0, math.pi]))
            if (param["map"]["obstacles"]["bottom_right_corner"] is not None):
                for i in param["map"]["obstacles"]["bottom_right_corner"]:
                    p.loadURDF("data/rectangle_corner.urdf", [i[0], i[1], 0.5], p.getQuaternionFromEuler([0, 0, -math.pi/2]))
            if (param["map"]["obstacles"]["horizontal_half"] is not None):
                for i in param["map"]["obstacles"]["horizontal_half"]:
                    p.loadURDF("data/rectangle_horizontal_half.urdf", [i[0], i[1], 0.5])
            if (param["map"]["obstacles"]["vertical_half"] is not None):
                for i in param["map"]["obstacles"]["vertical_half"]:
                    p.loadURDF("data/rectangle_vertical_half.urdf", [i[0], i[1], 0.5])
    return agents, goals, True

def read_output(output_yaml_file):
    """
        Read file from output.yaml, store path list.

        Args:

        output_yaml_file: output file from cbs.

        Returns:

        schedule: path to goal position for each robot.
    """
    with open(output_yaml_file, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)
    return param["schedule"]

def run(agents, goals, schedule):
    """
        Set up loop to publish leftwheel and rightwheel velocity for each robot to reach goal position.

        Args:

        agents: array containing the boxID for each agent

        schedule: dictionary with boxID as key and path to the goal as list for each robot.

        goals: dictionary with boxID as the key and the corresponding goal positions as values
    """
    threads = []
    for agent in agents:
        t = threading.Thread(target=navigation, args=(agent, goals[agent], schedule[agent]))
        threads.append(t)
        t.start()

    # for t in threads:
    #     t.join()


def drop_ball(agents):
    """
        Drop ball for each robot at their current positions.

        Args:

        agents: array containing the boxID for each agent
    """
    for i in agents:
        pos = p.getBasePositionAndOrientation(i)[0]
        ball_pos = [pos[0], pos[1], 2]
        p.loadURDF("data/sphere_small.urdf", ball_pos)

def drop_cube(agents):
    """
        Drop cubes for each robot at their current positions.

        Args:

        agents: array containing the boxID for each agent
    """
    for i in agents:
        pos = p.getBasePositionAndOrientation(i)[0]
        cube_pos = [pos[0], pos[1], 2]
        p.loadURDF("cube.urdf", cube_pos, globalScaling=0.1)

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
p.setRealTimeSimulation(1)
p.setGravity(0, 0, -10)
startOrientation = p.getQuaternionFromEuler([0,0,0])
global env_loaded
env_loaded = False

agents, goals, env_loaded = read_input("scene/room_scene_1_bot/room_scene_1_bot_env.yaml", env_loaded)
cbs.main("scene/room_scene_1_bot/room_scene_1_bot.yaml", "output.yaml")
schedule = read_output("output.yaml")
threads = []
run(agents, goals, schedule)
time.sleep(3)
# drop_cube(agents)
# _,goals,env_loaded = read_input("scene/room_scene_4_bots/room_scene_4_bots_stage_2.yaml", env_loaded)
# cbs.main("scene/room_scene_1_bot_stage_2.yaml", "output.yaml")
# schedule = read_output("output.yaml")
# navigation(agents, goals, schedule)