"""
Collision avoidance using Nonlinear Model-Predictive Control

author: Ashwin Bose (atb033@github.com)
"""

from utils.multi_robot_plot import plot_robot_and_obstacles
from utils.create_obstacles import create_obstacles
import numpy as np
from scipy.optimize import minimize, Bounds
import time
import copy

SIM_TIME = 18.
TIMESTEP = 0.1
NUMBER_OF_TIMESTEPS = int(SIM_TIME/TIMESTEP)
ROBOT_RADIUS = 0.5
VMAX = 2
VMIN = 0.2

# collision cost parameters
Qc = 5.
kappa = 4.

# nmpc parameters
HORIZON_LENGTH = int(4)
NMPC_TIMESTEP = 0.3
upper_bound = [(1/np.sqrt(2)) * VMAX] * HORIZON_LENGTH * 2
lower_bound = [-(1/np.sqrt(2)) * VMAX] * HORIZON_LENGTH * 2
no_agents = 3
# agent priority- 0,1,2 ....


def simulate(filename):
    # obstacles = create_obstacles(SIM_TIME, NUMBER_OF_TIMESTEPS)

    # automate task generation
    starts = np.array([[5, 5], [7,8],[5,8]])
    ps_desired = np.array([[7,8],[5,5], [7,2]])

    # starts of all agents
    robot_state = starts.astype(float)

    # (p,v) history for all agents
    static_obstacle = np.array([[6, 6]])
    no_obs = 1
    robot_state_history = np.empty((no_agents+no_obs, NUMBER_OF_TIMESTEPS+HORIZON_LENGTH, 4))
    for i in range(no_obs):
        robot_state_history[i, :, 0:2] = static_obstacle[i]

    for i in range(no_agents):
        for j in range(NUMBER_OF_TIMESTEPS):
        # predict the obstacles' position in future
        # obstacle_predictions = predict_obstacle_positions(obstacles[:, i, :])
            obstacle_predictions = robot_state_history[0:i+no_obs, j:j+HORIZON_LENGTH, 0:2] # check if the result is in desired format
            # find reference path (interpolation)
            xref = compute_xref(robot_state[i], ps_desired[i],
                            HORIZON_LENGTH, NMPC_TIMESTEP)
            # find path
            # compute velocity using nmpc
            vel, velocity_profile = compute_velocity(
                robot_state[i], obstacle_predictions, xref)
            interim = update_state(robot_state[i], vel, TIMESTEP)
            robot_state[i] = interim
            robot_state_history[i+no_obs, j, 0:2] = robot_state[i]

    plot_robot_and_obstacles(
        robot_state_history[0], robot_state_history[1:no_agents+no_obs], ROBOT_RADIUS, NUMBER_OF_TIMESTEPS, SIM_TIME, filename)


def compute_velocity(robot_state, obstacle_predictions, xref):
    """
    Computes control velocity of the copter
    """
    # u0 = np.array([0] * 2 * HORIZON_LENGTH)
    u0 = np.random.rand(2*HORIZON_LENGTH)
    def cost_fn(u): return total_cost(
        u, robot_state, obstacle_predictions, xref)

    bounds = Bounds(lower_bound, upper_bound)

    res = minimize(cost_fn, u0, method='SLSQP', bounds=bounds)
    velocity = res.x[:2]
    return velocity, res.x


def compute_xref(start, goal, number_of_steps, timestep):
    dir_vec = (goal - start)
    norm = np.linalg.norm(dir_vec)
    if norm < 0.1:
        new_goal = start
    else:
        dir_vec = dir_vec / norm
        new_goal = start + dir_vec * VMAX * timestep * number_of_steps
    return np.linspace(start, new_goal, number_of_steps).reshape((2*number_of_steps))


def total_cost(u, robot_state, obstacle_predictions, xref):
    x_robot = update_state(robot_state, u, NMPC_TIMESTEP)
    c1 = tracking_cost(x_robot, xref)
    c2 = total_collision_cost(x_robot, obstacle_predictions)
    total = c1 + c2
    return total


def tracking_cost(x, xref):
    return np.linalg.norm(x-xref)


def total_collision_cost(robot, obstacles):
    total_cost = 0
    for i in range(HORIZON_LENGTH):
        if (obstacles.size==0):
            continue
        # obstacles = obstacles[0]
        for j in range(len(obstacles)):
            obs = obstacles[j][i]
            rob = robot[2 * i: 2 * i + 2]
            # obs = obstacle[2 * i: 2 * i + 2]
            total_cost += collision_cost(rob, obs)
    return total_cost


def collision_cost(x0, x1):
    """
    Cost of collision between two robot_state
    """
    d = np.linalg.norm(x0 - x1)
    cost = Qc / (1 + np.exp(kappa * (d - 2*ROBOT_RADIUS)))
    return cost


def predict_obstacle_positions(obstacles):
    obstacle_predictions = []
    for i in range(np.shape(obstacles)[1]):
        obstacle = obstacles[:, i]
        obstacle_position = obstacle[:2]
        obstacle_vel = obstacle[2:]
        u = np.vstack([np.eye(2)] * HORIZON_LENGTH) @ obstacle_vel
        obstacle_prediction = update_state(obstacle_position, u, NMPC_TIMESTEP)
        obstacle_predictions.append(obstacle_prediction)
    return obstacle_predictions


def update_state(x0, u, timestep):
    """
    Computes the states of the system after applying a sequence of control signals u on
    initial state x0
    """
    N = int(len(u) / 2)
    lower_triangular_ones_matrix = np.tril(np.ones((N, N)))
    kron = np.kron(lower_triangular_ones_matrix, np.eye(2))

    new_state = np.vstack([np.eye(2)] * int(N)) @ x0 + kron @ u * timestep

    return new_state
