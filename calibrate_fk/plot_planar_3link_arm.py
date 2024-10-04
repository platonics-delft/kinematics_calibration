import numpy as np
import matplotlib.pyplot as plt


def plot_robot_arm(thetas, link_lengths, i):
    x, y = forward_kinematics_plot(thetas, link_lengths)


    plt.figure()
    plt.plot(x, y, '-o', markersize=10, lw=3, label="Robot Arm")
    
    plt.xlim([-sum(link_lengths), sum(link_lengths)])
    plt.ylim([-sum(link_lengths), sum(link_lengths)])
    
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title("3-Link Planar Robot Arm")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.legend()
    plt.savefig(f"robot_arm_{i}.png")

# Forward kinematics to compute the (x, y) coordinates of each link
def forward_kinematics_plot(theta, link_lengths):
    x = np.zeros(4)
    y = np.zeros(4)

    # Initial position at the base (0, 0)
    x[0], y[0] = 0, 0
    
    # Compute the (x, y) positions for each link
    for i in range(3):
        x[i+1] = x[i] + link_lengths[i] * np.cos(np.sum(theta[:i+1]))
        y[i+1] = y[i] + link_lengths[i] * np.sin(np.sum(theta[:i+1]))

    return x, y

