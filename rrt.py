import random
import math
import matplotlib.pyplot as plt
import numpy as np

class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRT():
    def __init__(self, start, goal, obstacle_list, width, height):
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.width = width
        self.height = height
        self.obstacle_list = obstacle_list
        self.node_list = []

    def planning(self, expand_distance=3.0, goal_sample_rate=5, max_iter=500):
        self.node_list = [self.start]
        for i in range(max_iter):
            if random.randint(0, 100) > goal_sample_rate:
                rnd_node = self.get_random_node()
            else:
                rnd_node = self.end
            nearest_node = self.get_nearest_node(self.node_list, rnd_node)
            new_node = self.steer(nearest_node, rnd_node, expand_distance)

            if self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= expand_distance:
                final_node = self.steer(self.node_list[-1], self.end, expand_distance)
                if self.check_collision(final_node, self.obstacle_list):
                    return self.generate_final_course(len(self.node_list) - 1)

            if i % 5 == 0:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float('inf')):
        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.x += extend_length * math.cos(theta)
        new_node.y += extend_length * math.sin(theta)

        new_node.parent = from_node
        return new_node #用于将生成的随机节点向最近节点方向移动一定距离，生成新的节点。

    def generate_final_course(self, goal_ind):
        path = [(self.end.x, self.end.y)]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.append((node.x, node.y))
        return path

    def calc_dist_to_goal(self, x, y):
        return np.hypot(self.end.x - x, self.end.y - y)

    def get_random_node(self):
        rnd = Node(random.uniform(0, self.width), random.uniform(0, self.height))
        return rnd

    @staticmethod
    def get_nearest_node(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2 for node in node_list]
        minind = dlist.index(min(dlist))
        return node_list[minind]

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    def check_collision(self, node, obstacle_list):
        for (ox, oy, size) in obstacle_list:
            dx_list = [ox - node.x, ox + size - node.x]
            dy_list = [oy - node.y, oy + size - node.y]
            d_list = [math.sqrt(dx ** 2 + dy ** 2) for dx in dx_list for dy in dy_list]

            if min(d_list) <= size:
                return False  # collision

        return True  # safe

    def draw_graph(self, rnd_node=None):
        plt.clf()
        if rnd_node is not None:
            plt.plot(rnd_node.x, rnd_node.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g")

        for (ox, oy, size) in self.obstacle_list:
            plt.plot(ox, oy, "sk", ms=size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([0, self.width, 0, self.height])
        plt.grid(True)
        plt.pause(0.01)

def main():
    # 参数设置
    obstacle_list = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # (x, y, radius)

    # 创建RRT对象
    rrt = RRT(start=[0, 0],
              goal=[10, 10],
              width=15,
              height=15,
              obstacle_list=obstacle_list)

    # 运行路径规划，获得路径
    path = rrt.planning()

    # 输出路径信息
    if path is None:
        print("Cannot find path")
    else:
        print("found path")

    # 展示路径规划结果
    rrt.draw_graph()
    if path is not None:
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
