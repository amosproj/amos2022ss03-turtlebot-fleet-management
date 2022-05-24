import math
from typing import List

import matplotlib.pyplot as plt


class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.name = str(len(points))

    def get_coords(self):
        return self.x, self.y

    def is_equal(self, p) -> bool:
        return self.x == p.x and self.y == p.y

    @staticmethod
    def get_point(x: float, y: float):
        for p in points:
            if p.x == x and p.y == y:
                return p
        new_point = Point(x, y)
        points.append(new_point)
        return new_point


class Line:
    def __init__(self, start: Point, end: Point):
        self.start = start
        self.end = end

    def point_on_line(self, p: Point) -> bool:
        if self.start.is_equal(p) or self.end.is_equal(p):
            return False
        line_length = math.dist((self.start.x, self.start.y), (self.end.x, self.end.y))
        start_to_p = math.dist((self.start.x, self.start.y), (p.x, p.y))
        end_to_p = math.dist((self.end.x, self.end.y), (p.x, p.y))
        dist_diff = abs(line_length - start_to_p - end_to_p)
        return dist_diff < 0.01

    def split_line(self, p: Point):
        assert self.point_on_line(p)
        line1 = Line(self.start, p)
        line2 = Line(p, self.end)
        lines.append(line1)
        lines.append(line2)
        lines.remove(self)
        return line1, line2


points: List[Point] = list()
lines: List[Line] = list()


def import_vmap(filename: str):
    f = open(filename, "r")
    vmap_lines = f.readlines()

    for vmap_line in vmap_lines:
        if vmap_line.startswith("#"):
            continue
        if vmap_line.startswith("POSE"):
            break
        if vmap_line.startswith("LANE"):
            coords_old = vmap_line.split()
            coords = [0, round(float(coords_old[1]), 3), round(float(coords_old[2]), 3),
                      round(float(coords_old[3]), 3), round(float(coords_old[4]), 3)]
            p1 = Point.get_point(float(coords[1]), float(coords[2]))
            p2 = Point.get_point(float(coords[3]), float(coords[4]))
            line = Line(p1, p2)
            lines.append(line)

    merge_lines()
    remove_duplicate_lines()
    return points, lines


def merge_lines():
    queue = lines.copy()
    while len(queue) > 0:
        line = queue.pop(0)
        for point in points:
            if line.point_on_line(point):
                print("Line " + line.start.name + " " + line.end.name + " will be split")
                line1, line2 = line.split_line(point)
                queue.append(line1)
                queue.append(line2)
                break


def remove_duplicate_lines():
    changes = True
    while changes:
        changes = False
        for line1 in lines:
            for line2 in lines:
                if line1 is line2:
                    continue
                if (line1.start is line2.start and line1.end is line2.end) or \
                        (line1.end is line2.start and line1.start is line2.end) or \
                        (line1.start is line2.end and line1.end is line2.start):
                    print("Found a duplicate line " + line1.start.name + " " + line1.end.name + ", removing...")
                    lines.remove(line2)
                    changes = True
                    break
            if changes:
                break


def create_plot():
    for line in lines:
        plt.plot([line.start.x, line.end.x], [line.start.y, line.end.y], linestyle="dashed", marker="s")
    for point in points:
        plt.annotate(point.name, point.get_coords())
    # plt.axis([4, 5, 2.4, 3])
    # plt.axis([4, 4.1, -5, -4.75])
    # plt.axis([-0.2, 0.2, -5.5, -4])
    # plt.show(dpi=3000, bbox_inches="tight")
    plt.savefig("imported_vmap.png", dpi=3000, bbox_inches="tight")

# import_vmap("demo.vmap")
# create_plot()
