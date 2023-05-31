import random
import numpy as np
import matplotlib.pyplot as plt
from lma import *

class Node:
    def __init__(self, x, z, a1, a2):
        self.x = x
        self.z = z
        self.a1 = a1
        self.a2 = a2
        self.state = [x,z,a1,a2]
        self.parent = None

def get_random_node():
    x, z, a1, a2 = random.uniform(-2, 15), random.uniform(-2, 15), random.uniform(-np.pi, np.pi), random.uniform(-np.pi, np.pi)
    return Node(x,z,a1,a2)

def get_distance(node1, node2):
    weight_dz = 0.1
    weight_a1 = 0.1 
    weight_a2 = 0.1
    dz = weight_dz * np.sqrt((node1.x - node2.x)**2 + (node1.z - node2.z)**2)
    da1 = weight_a1 * abs(node1.a1 - node2.a1)
    da2 = weight_a2 * abs(node1.a2 - node2.a2)
    return dz + da1 + da2
    
def get_nearest_node(nodes, random_node):
    distances = [get_distance(node, random_node) for node in nodes]
    nearest_idx = np.argmin(distances)
    return nodes[nearest_idx]

def steer(nearest_node, random_node, extend_length=0.5):
    direction = np.array([random_node.x - nearest_node.x, random_node.z - nearest_node.z])
    distance = np.linalg.norm(direction)
    direction = direction / distance
    new_node = Node(nearest_node.x + extend_length * direction[0],
                    nearest_node.z + extend_length * direction[1],
                    random_node.a1,
                    random_node.a2)
    new_node.parent = nearest_node
    return new_node

def check(nodes):
    if (nodes.x == 1):
        cnt = 1

def main():#
    nodes = [Node(0, 0, 0, 0)]
    goal = Node(10, 0, np.pi/2, np.pi/2)

    fig, ax = plt.subplots(figsize=(10, 10))

    
    for _ in range(1000):
        random_node = get_random_node()
        nearest_node = get_nearest_node(nodes, random_node)
        new_node = steer(nearest_node, random_node)

        # Visualize
        if new_node.parent:
            x1, z1, x2, z2, xp, zp = plot_structure(new_node.state, ax)

        plt.grid(True)
        plt.axis("equal")
        plt.pause(0.01)

        nodes.append(new_node)
        if np.linalg.norm(np.array(new_node.x) - np.array(goal.x)) < 0.5:
            print("Reached the goal!")
            break

    plt.show()

if __name__ == "__main__":
    main()
