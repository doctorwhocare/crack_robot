import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def lines_inter(x1, y1, x2, y2, x3, y3, x4, y4):
    if max(x3, x4) < min(x1, x2) or max(y3, y4) < min(y1, y2) or max(x1, x2) < min(x3, x4) or max(y1, y2) < min(y3, y4):
        inter = False
    elif (((x1 - x3) * (y4 - y3) - (y1 - y3) * (x4 - x3)) * ((x2 - x3) * (y4 - y3) - (y2 - y3) * (x4 - x3)) > 0 or
            ((x3 - x1) * (y2 - y1) - (y3 - y1) * (x2 - x1)) * ((x4 - x1) * (y2 - y1) - (y4 - y1) * (x2 - x1)) > 0):
        inter = False
    else:
        inter = True

    return inter

def plot_structure(state, ax):
    L12 = 1
    Lr = 3.1
    r = 0.1
    dr = 0.001

    x1 = state[0]
    y1 = state[1]
    ang21 = state[2]
    angp = state[3]

    x2 = x1 + L12 * np.cos(ang21)
    y2 = y1 + L12 * np.sin(ang21)

    Lp0 = (Lr - 2 * np.pi * r - L12) / 2
    Lp90 = np.sqrt((Lr - 2 * np.pi * r - 2 * L12) * (Lr - 2 * np.pi * r)) / 2
    Lp = Lp90 * Lp0 / np.sqrt((Lp90 * np.cos(angp))**2 + (Lp0 * np.sin(angp))**2)
    xp = x1 + L12 * np.cos(ang21) / 2 + Lp * np.cos(angp)
    yp = y1 + L12 * np.sin(ang21) / 2 + Lp * np.sin(angp)

    # fig, ax = plt.subplots()

    circle1 = patches.Circle((x1, y1), r, linewidth=1, edgecolor='r', facecolor='none')
    ax.add_patch(circle1)
    circle2 = patches.Circle((x2, y2), r, linewidth=1, edgecolor='b', facecolor='none')
    ax.add_patch(circle2)
    circlep = patches.Circle((xp, yp), r, linewidth=1, edgecolor='g', facecolor='none')
    ax.add_patch(circlep)

    plt.plot([x1, x2], [y1, y2], 'k-')
    plt.plot([(x1+x2)/2, xp], [(y1+y2)/2, yp], 'k-')

    p1 = np.arctan2((yp - y1), (xp - x1))
    p2 = np.arctan2((yp - y2), (xp - x2))
    if not lines_inter(x2, y2, (x1+xp)/2, (y1+yp)/2, x1 + dr * np.cos(p1 + np.pi / 2), y1 + dr * np.sin(p1 + np.pi / 2), xp + dr * np.cos(p1 + np.pi / 2), yp + dr * np.sin(p1 + np.pi / 2)):
        plt.plot([x1, xp] + r * np.cos(p1 + np.pi / 2), [y1, yp] + r * np.sin(p1 + np.pi / 2), 'k-')
    else:
        plt.plot([x1, xp] + r * np.cos(p1 - np.pi / 2), [y1, yp] + r * np.sin(p1 - np.pi / 2), 'k-')

    if not lines_inter(xp, yp, (x1+x2)/2, (y1+y2)/2, x1 + dr * np.cos(ang21 + np.pi / 2), y1 + dr * np.sin(ang21 + np.pi / 2), x2 + dr * np.cos(ang21 + np.pi / 2), y2 + dr * np.sin(ang21 + np.pi / 2)):
        plt.plot([x1, x2] + r * np.cos(ang21 + np.pi / 2), [y1, y2] + r * np.sin(ang21 + np.pi / 2), 'k-')
    else:
        plt.plot([x1, x2] + r * np.cos(ang21 - np.pi / 2), [y1, y2] + r * np.sin(ang21 - np.pi / 2), 'k-')

    if not lines_inter(x1, y1, (xp+x2)/2, (yp+y2)/2, xp + dr * np.cos(p2 + np.pi / 2), yp + dr * np.sin(p2 + np.pi / 2), x2 + dr * np.cos(p2 + np.pi / 2), y2 + dr * np.sin(p2 + np.pi / 2)):
        plt.plot([xp, x2] + r * np.cos(p2 + np.pi / 2), [yp, y2] + r * np.sin(p2 + np.pi / 2), 'k-')
    else:
        plt.plot([xp, x2] + r * np.cos(p2 - np.pi / 2), [yp, y2] + r * np.sin(p2 - np.pi / 2), 'k-')

    # plt.axis('equal')
    # plt.show()

    return x1, y1, x2, y2, xp, yp

def get_structure(state):
    L12 = 1
    Lr = 3.1
    r = 0.1

    x1 = state[0]
    y1 = state[1]
    ang21 = state[2]
    angp = state[3]

    x2 = x1 + L12 * np.cos(ang21)
    y2 = y1 + L12 * np.sin(ang21)

    Lp0 = (Lr - 2 * np.pi * r - L12) / 2
    Lp90 = np.sqrt((Lr - 2 * np.pi * r - 2 * L12) * (Lr - 2 * np.pi * r)) / 2
    Lp = Lp90 * Lp0 / np.sqrt((Lp90 * np.cos(angp))**2 + (Lp0 * np.sin(angp))**2)
    xp = x1 + L12 * np.cos(ang21) / 2 + Lp * np.cos(angp)
    yp = y1 + L12 * np.sin(ang21) / 2 + Lp * np.sin(angp)

    return x1, y1, x2, y2, xp, yp

if __name__ == "__main__":
    state = [0, 0, np.pi/4, np.pi/6]
    fig, ax = plt.subplots()
    plot_structure(state, ax)