"""
Plotting tool for 2D multi-robot system

author: Ashwin Bose (@atb033)
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle
import numpy as np


def plot_robot_and_obstacles(robot, obstacles, robot_radius, num_steps, sim_time, filename):
    fig = plt.figure()
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(0, 10), ylim=(0, 10))
    ax.set_aspect('equal')
    ax.grid()
    lines = []

    plotcols = ["black", "blue", "green", "purple", "brown", "orange"]
    for index in range(obstacles.shape[0]):
        lobj = ax.plot([],[],lw=2,color=plotcols[index])[0]
        lines.append(lobj)

    static_patch = []
    for i in range(robot.shape[0]):
        static_patch.append(Rectangle((robot[i][0, 0] - robot_radius, robot[i][0, 1] - robot_radius),
                            2*robot_radius, 2*robot_radius, facecolor='red', edgecolor='black'))
    obstacle_list = []
    for obstacle in range(np.shape(obstacles)[0]):
        obstacle = Circle((0, 0), robot_radius,
                          facecolor='aqua', edgecolor='black')
        obstacle_list.append(obstacle)

    def init():
        for sp in static_patch:
            ax.add_patch(sp)
        for obstacle in obstacle_list:
            ax.add_patch(obstacle)
        for line in lines:
            line.set_data([],[])
        return [static_patch] + [lines] + obstacle_list

    def animate(i):
        line_x = []
        line_y = []
        for k in range(robot.shape[0]):
            static_patch[k].center = (robot[k][i,0], robot[k][i,1])
        for j in range(len(obstacle_list)):
            obstacle_list[j].center = (obstacles[j, i, 0], obstacles[j, i, 1])
        # for j in range(len(obstacle_list)):
            line_x.append(obstacles[j, :i, 0].tolist()) 
            line_y.append(obstacles[j, :i, 1].tolist())
        for lnum,line in enumerate(lines):
            line.set_data(line_x[lnum], line_y[lnum])
        
        return [static_patch] + [lines] + obstacle_list

    init()
    step = (sim_time / num_steps)
    for i in range(num_steps):
        animate(i)
        plt.pause(step)

    # Save animation
    if not filename:
        return

    ani = animation.FuncAnimation(
        fig, animate, np.arange(1, num_steps), interval=200,
        blit=True, init_func=init)

    ani.save(filename, "ffmpeg", fps=30)


def plot_robot(robot, timestep, radius=1, is_obstacle=False):
    if robot is None:
        return
    center = robot[:2, timestep]
    x = center[0]
    y = center[1]
    if is_obstacle:
        circle = plt.Circle((x, y), radius, color='aqua', ec='black')
        plt.plot(robot[0, :timestep], robot[1, :timestep], '--r',)
    else:
        circle = plt.Circle((x, y), radius, color='green', ec='black')
        plt.plot(robot[0, :timestep], robot[1, :timestep], 'blue')

    plt.gcf().gca().add_artist(circle)
