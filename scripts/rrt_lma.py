import random
import numpy as np
import matplotlib.pyplot as plt
from lma import *
import math

class Node:
    def __init__(self, x, z, a1, a2):
        self.x = x
        self.z = z
        self.a1 = a1
        self.a2 = a2
        self.state = [x,z,a1,a2]
        self.parent = None

class Ground:
    def __init__(self, lenth):
        self.lenth = lenth
        self.data = [0]*lenth

class Circle:
    def __init__(self, x1, z1, x2, z2, x3, z3):
        self.before_x = x1
        self.before_z = z1        
        self.behind_x = x2
        self.behind_z = z2  
        self.change_x = x3
        self.change_z = z3  
def get_random_node(ground):
    x, z, a1, a2 = random.uniform(-2, 15), random.uniform(-2, 15), random.uniform(-np.pi, np.pi), random.uniform(-np.pi, np.pi)
    node = Node(x,z,a1,a2)
    temp_position = get_circle_position(node.state)
    return Node(x,z,a1,a2), temp_position

def pre_work(ground,new_node):
    x = new_node.state[0]
    z = new_node.state[1]
    a1 = new_node.state[2]
    a2 = new_node.state[3]
    
    temp_position = get_circle_position(new_node.state)
    # z = ground.data[int(x)] + 0.1
    z_values = [(temp_position.before_z, temp_position.before_x), 
                (temp_position.behind_z, temp_position.behind_x), 
                (temp_position.change_z, temp_position.change_x),
                (z, x)]
    min_z, corresponding_x = min(z_values)
    offset = ground.data[int(corresponding_x)] + 0.1 - min_z
    temp_position.before_z += offset
    temp_position.behind_z += offset
    temp_position.change_z += offset
    z += offset

    z_values = [(temp_position.before_z, temp_position.before_x), 
                (temp_position.behind_z, temp_position.behind_x), 
                (temp_position.change_z, temp_position.change_x), 
                (z, x)]
     # 对所有对应的x属性进行判断，并保证z值大于等于ground.data[x]
    min_x_offset = 0
    for z_value, x_value in z_values:
        flag = 0
        if x_value < len(ground.data):
            x_offset_up = 0
            x_offset_back = 0
            
            while (int(x_value) + x_offset_up < len(ground.data) and z_value - 0.1 < ground.data[int(x_value) + x_offset_up]):
                x_offset_up += 1
                flag = 1
            while (int(x_value) + x_offset_back >= 0 and z_value - 0.1 < ground.data[int(x_value) + x_offset_back]):
                x_offset_back -= 1
                flag = 1
            
            # print(x_value, abs(x_offset_back), x_offset_up)
            if (abs(x_offset_back) < x_offset_up):
                x_offset = x_offset_back
            else:
                x_offset = x_offset_up
            if (abs(x_offset) > abs(min_x_offset) and flag == 1):
                min_x_offset = x_offset


    # 对所有x属性进行偏移，确保不超过地面数据列表的长度
    if min_x_offset == 10:
        min_x_offset = 0
    # print("final_final", min_x_offset)
    temp_position.before_x = min(temp_position.before_x + min_x_offset, len(ground.data) - 1)
    temp_position.behind_x = min(temp_position.behind_x + min_x_offset, len(ground.data) - 1)
    temp_position.change_x = min(temp_position.change_x + min_x_offset, len(ground.data) - 1)
    x = min(x + min_x_offset, 10)

    return Node(x,z,a1,a2), temp_position
# def get_random_node(ground):
#     print()
#     x, z, a1, a2 = random.randint(0, 10), random.uniform(0, 9), random.uniform(-np.pi, np.pi), random.uniform(-np.pi, np.pi)    
#     # x, z, a1, a2 = 3, 0, 0, np.pi / 2
#     # Adjust z based on ground data
#     node = Node(x,z,a1,a2)
#     # print("this", x,z,a1,a2)
#     temp_position = get_circle_position(node.state)
#     node_print(temp_position)
#     # z = ground.data[x] + 0.5
#     # z_values = [(temp_position.before_z, temp_position.before_x), 
#     #             (temp_position.behind_z, temp_position.behind_x), 
#     #             (temp_position.change_z, temp_position.change_x),
#     #             (z, x)]
#     # min_z, corresponding_x = min(z_values)

#     # # 计算偏移量
#     # print("this", int(corresponding_x) ,min_z)
#     # offset = ground.data[int(corresponding_x)] + 0.1 - min_z
    
#     # # 对所有z属性进行偏移
#     # temp_position.before_z += offset
#     # temp_position.behind_z += offset
#     # temp_position.change_z += offset
#     # z += offset
#     # z_values = [(temp_position.before_z, temp_position.before_x), 
#     #             (temp_position.behind_z, temp_position.behind_x), 
#     #             (temp_position.change_z, temp_position.change_x), 
#     #             (z, x)]
#     #  # 对所有对应的x属性进行判断，并保证z值大于等于ground.data[x]
#     # min_x_offset = 0
#     # for z_value, x_value in z_values:
#     #     flag = 0
#     #     if x_value < len(ground.data):
#     #         x_offset_up = 0
#     #         x_offset_back = 0
            
#     #         while (int(x_value) + x_offset_up < len(ground.data) and z_value - 0.1 < ground.data[int(x_value) + x_offset_up]):
#     #             x_offset_up += 1
#     #             flag = 1
#     #         while (int(x_value) + x_offset_back >= 0 and z_value - 0.1 < ground.data[int(x_value) + x_offset_back]):
#     #             x_offset_back -= 1
#     #             flag = 1
            
#     #         # print(x_value, abs(x_offset_back), x_offset_up)
#     #         if (abs(x_offset_back) < x_offset_up):
#     #             x_offset = x_offset_back
#     #         else:
#     #             x_offset = x_offset_up
#     #         if (abs(x_offset) > abs(min_x_offset) and flag == 1):
#     #             min_x_offset = x_offset


#     # # 对所有x属性进行偏移，确保不超过地面数据列表的长度
#     # if min_x_offset == 10:
#     #     min_x_offset = 0
#     # # print("final_final", min_x_offset)
#     # temp_position.before_x = min(temp_position.before_x + min_x_offset, len(ground.data) - 1)
#     # temp_position.behind_x = min(temp_position.behind_x + min_x_offset, len(ground.data) - 1)
#     # temp_position.change_x = min(temp_position.change_x + min_x_offset, len(ground.data) - 1)
#     # x = min(x + min_x_offset, 10)
#     return Node(x,z,a1,a2), temp_position

def calculate_line_equation(x1, y1, x2, y2):
    # 计算斜率
    slope = (y2 - y1) / (x2 - x1)

    # 计算截距
    intercept = y1 - slope * x1
    print(f"连线方程为 y = {slope}x + {intercept}")

    return slope, intercept

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

def get_circle_position(node):
    """
    L12 = 1 #底盘轮间距
    Lr = 3.1 #三轮中心三角形周长
    r = 0.1 #轮子半径
    """
    
    temp_x1, temp_y1, temp_x2, temp_y2, temp_xp, temp_yp = get_structure(node)
    position = Circle(temp_x1, temp_y1, temp_x2, temp_y2, temp_xp, temp_yp)
    return position 


def find_tangent_lines(x1, y1, x2, y2, r):
    dx, dy = x2 - x1, y2 - y1
    k = dy / dx
    si = math.atan(k)
    xx1 = x1 + r * math.sin(si)
    xx2 = x1 - r * math.sin(si)
    yy1 = y1 + r * math.cos(si)
    yy2 = y1 - r * math.cos(si)
    b1 = yy1 - k * xx1
    b2 = yy1 - k * xx2
    return k, b1, b2



def is_intersect(k, b, x1, x2, ground, r, flag):
    arctan = math.atan(k)
    if (flag == 2):
        s = int(round(x1 - r * math.sin(arctan)))
        t = int(round(x2 - r * math.sin(arctan)))
        print("s_to_end2", s, t, r * math.sin(arctan), k ,b)
        for i in range(s, t + 1):
            temp_y = k * i + b
            # print("s_to_end2",i ,temp_y, ground.data[i])
            if (temp_y < ground.data[i]):
                print("test", temp_y, ground.data[i], i, k, b)
                return True
    else:
        s = int(x1 + r * math.sin(arctan))
        t = int(x2 + r * math.sin(arctan))
        print("s_to_end2", s, t, r * math.sin(arctan), k ,b)
        for i in range(s, t + 1):
            temp_y = k * i + b
            print("s_to_end2", temp_y, ground.data[i], i, k, b)
            if (temp_y < ground.data[i]):
                print("test2", temp_y, ground.data[i])
                return True
    return False



def check_line(x1, y1, x2, y2, r, flag, ground):
    k1, b1, b2 = find_tangent_lines(x1, y1, x2, y2, r)
    print("5345454", k1, b1, b2)
    if is_intersect(k1, b1, x1, x2, ground, r, 1) == True or is_intersect(k1, b2, x1, x2, ground, r, 2) == True: 
        return 1
    return 0

def check(temp_position, ground):
    epsilon = 0.001
    cnt = 0
    node = []
    coordinates = [
        (temp_position.before_x, temp_position.before_z),
        (temp_position.behind_x, temp_position.behind_z),
        (temp_position.change_x, temp_position.change_z),
    ]
    print(coordinates)
    for x, z in coordinates:
        if z < 0: continue
        print(x, z, ground.data[int(x)])
        if 0 <=z - (ground.data[int(x)] + 0.1) <= epsilon:
            cnt += 1
    if (cnt >=2):
        # print("come ")
        if (check_line(temp_position.before_x, temp_position.before_z, temp_position.behind_x, temp_position.behind_z, 0.1, 1, ground)) == True:
            print("before, behind")
            return 0
        if (check_line(temp_position.before_x, temp_position.before_z, temp_position.change_x, temp_position.change_z, 0.1, 2, ground) == True):
            print("before, change")
            return 0
        if (check_line(temp_position.change_x, temp_position.change_z, temp_position.behind_x, temp_position.behind_z, 0.1, 3, ground) == True):
            print("change, behind")
            return 0
    return cnt

def node_print(temp_position):
    print(temp_position.before_x, temp_position.before_z)
    print(temp_position.behind_x, temp_position.behind_z)
    print(temp_position.change_x, temp_position.change_z)

def build_map(m, n, s):
    ground = Ground(200)
    for i in range(len(ground.data)):
        if i < s:
            ground.data[i] = 0
        elif s <= i < (s + m):
            ground.data[i] = n
        else:
            ground.data[i] = 0
    return ground


def main():#
    nodes = [Node(0, 0, 0, 0)]
    goal = Node(10, 0, np.pi/2, np.pi/2)
    ground = build_map(5, 0.5, 3)
    fig, ax = plt.subplots(figsize=(10, 10))
    # 添加长方形到图形中，位置在x轴上3处，长度为5，高度为0.5
    rectangle = patches.Rectangle((3, 0), 5, 0.50, edgecolor='r', facecolor='none')
    ax.add_patch(rectangle)    

    for _ in range(1000000):
        random_node, temp_position = get_random_node(ground)
        print(_)
        # random_node = Node(1, 0.1, 0, 90)
        # temp_position = get_circle_position(node.state)
        node_print(temp_position)
        nearest_node = get_nearest_node(nodes, random_node)
        new_node = steer(nearest_node, random_node)
        # temp_position = get_circle_position(new_node.state)
        # Visualize
        if (new_node.parent):
            new_node, temp_position = pre_work(ground, new_node)
            if (check(temp_position, ground) >= 2):
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
