import math

def find_external_tangent_lines(x1, y1, x2, y2, r):
    dx, dy = x2 - x1, y2 - y1
    d = math.sqrt(dx*dx + dy*dy)
    if d <= 2*r:
        return None
    l = math.sqrt(d*d - 4*r*r)
    mx, my = (x1 + x2) / 2, (y1 + y2) / 2  # midpoint
    dx /= d
    dy /= d
    # tangent points are derived by rotating midpoint around original points by +- 90 degrees and moving along the direction
    tangent_lines = []
    for sign in [-1, 1]:
        x3 = mx - sign*r*dy
        y3 = my + sign*r*dx
        # line's slope is -dx/dy because it's perpendicular to the line between centers
        k = -dx/dy if dy != 0 else float(0)
        b = y3 - k*x3
        tangent_lines.append((k, b))

    return tangent_lines

# testing with your input
lines = find_external_tangent_lines(8.5, 0.1, 9.5, 0.1, 0.1)
for line in lines:
    print('y = {:.2f}x + {:.2f}'.format(*line))
