from graphic import Circle, Rectangle
from node import Quadrant, Quadtree
import concurrent.futures
from threading import Thread
import math
import numpy as np
from random import random
from random import seed
import tkinter as tk
import formula
import time


class CircleZone(Circle):
    def __init__(self, x, y, radius):
        super(CircleZone, self).__init__(x, y, radius)
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, width=1000, height=1000)
        self.canvas.pack()
        self.animation = None
        self._circles = []
        self._grid = Quadtree(Quadrant(x, y, 2 * radius, 2 * radius))
        self.count = 0
        self.draw(self.canvas)
        self.add_random_circles(100)
        self._grid.draw(self.canvas)
        self._grid.quadtree_search_result()
        self.move_circles_randomly(100)
        self.root.mainloop()

    def grid(self):
        return self._grid

    def randomize_circle_coord(self, radius):
        # Generate x randomly inside the area.
        x_range = 2 * (self._radius - radius)
        min_x = self._center[0] - x_range / 2
        x = random() * x_range + min_x

        # Generate y randomly inside the area.
        y_range = 2 * math.sqrt(math.pow(self._radius - radius, 2) - math.pow(x - self._center[0], 2))
        min_y = self._center[1] - y_range / 2
        y = random() * y_range + min_y
        return np.array([x, y])

    def add_random_circles(self, n=1, random_radius=False, min_radius=1, max_radius=10, radius=10, overlap=False,
                           iterations=100):
        for i in range(n):
            if random_radius:
                radius = random() * (max_radius - min_radius) + min_radius
            coord = self.randomize_circle_coord(radius)
            x, y = coord[0], coord[1]
            circle = Circle(x, y, radius)
            if overlap:
                quadrants = self._grid.quadtree_search(circle, overlap)
                for quadrant in quadrants:
                    quadrant.contents().append(circle)
                self._grid.contents().append(circle)
            else:
                # Iterate until the new circle doesn't overlap the existing circles.
                j = 0
                while j < iterations:
                    quadrants = self._grid.quadtree_search(circle, overlap)
                    if len(quadrants) > 0:
                        for quadrant in quadrants:
                            quadrant.contents().append(circle)
                        self._grid.contents().append(circle)
                        self._circles.append(circle)
                        break
                    else:
                        coord = self.randomize_circle_coord(radius)
                        x, y = coord
                        circle.set_center(x, y)
                        j += 1

    def draw(self, canvas, fill="", outline="black"):
        super(CircleZone, self).draw(canvas, fill=fill, outline=outline)
        self._grid.draw(canvas, fill=fill, outline=outline)

    def move_circles_randomly(self, magnitude):
        print("Count: " + str(self.count))
        for i in range(len(self._circles)):  # n = 1 for testing purpose.
            # Randomize the direction of the movement.
            angle = math.radians(random() * 360)
            x = magnitude * math.cos(angle)
            y = magnitude * math.sin(angle)
            max_displacement = np.array([x, y])
            max_distance = np.dot(max_displacement, max_displacement)

            # Get the circle's information.
            target = self._circles[i]
            target_radius = target.get_radius()
            current_position = target.get_center()

            # Calculate the circle's new position.
            new_position = current_position + max_displacement
            target.set_center(new_position[0], new_position[1])

            # Check if the circle is still inside the zone.
            if not self.confines_circle(target):
                new_position = formula.point_on_circumference(self._radius - target_radius, self._center,
                                                              current_position, new_position)
                target.set_center(new_position[0], new_position[1])
                max_displacement = new_position - current_position
                max_distance = np.dot(max_displacement, max_displacement)

            # Check if the circle collides with other circles along its path.
            if not math.isclose(max_distance, 0, rel_tol=1e-09):
                target.set_center(current_position[0], current_position[1])
                quadrants = self._grid.rectangle_overlap(current_position, new_position, target_radius, self.canvas)
                trajectory = formula.Segment(current_position, new_position)
                circles = set()
                obstacles = set()
                for quadrant in quadrants:
                    contents = quadrant.contents()
                    for content in contents:
                        if content != target:
                            circles.add(content)
                            point = content.get_center()
                            vector = point - current_position
                            theta = formula.angle_between(max_displacement, vector)
                            if abs(theta) < 90 and not math.isclose(abs(theta), 90):
                                distance_from_trajectory = trajectory.squared_distance_from_point(point)
                                distance_from_obstacle = target.squared_distance_from_point(point)
                                width_threshold = target.get_radius() + content.get_radius()
                                if distance_from_trajectory < math.pow(width_threshold, 2):
                                    distance_along_trajectory = distance_from_obstacle - distance_from_trajectory \
                                        if not math.isclose(distance_from_trajectory, 0, rel_tol=1e-09)\
                                        else distance_from_obstacle
                                    length_threshold = math.sqrt(np.dot(max_displacement, max_displacement)) + target_radius \
                                                       + content.get_radius()
                                    if distance_along_trajectory < math.pow(length_threshold, 2):
                                        obstacles.add(content)
                obstacles = list(obstacles)
                obstacles.sort(key=lambda circle: target.distance_from_circle(circle))
                non_obstacles = list(circles.difference(obstacles))
                non_obstacles.sort(key=lambda circle: target.distance_from_circle(circle))

                # Movement stops at nearest obstacle.
                j = 0
                while j < len(obstacles):
                    point = obstacles[j].get_center()
                    distance_from_trajectory = trajectory.squared_distance_from_point(point)
                    distance_from_obstacle = math.pow(obstacles[j].get_radius() + target_radius, 2)
                    projection = formula.project_vector(point - current_position, max_displacement)
                    distance_from_position = math.sqrt(distance_from_obstacle - distance_from_trajectory)
                    delta = formula.resize_vector(max_displacement, distance_from_position)
                    new_position = current_position + projection - delta
                    target.set_center(new_position[0], new_position[1])
                    displacement = new_position - current_position
                    distance = np.dot(displacement, displacement)
                    if distance > max_distance and not math.isclose(distance, max_distance):
                        max_displacement = formula.resize_vector(displacement, math.sqrt(max_distance))
                        new_position = current_position + max_displacement
                        target.set_center(new_position[0], new_position[1])
                        max_distance = np.dot(max_displacement, max_displacement)
                    j += 1
                    while j < len(obstacles):
                        if target.overlaps_circle(obstacles[j]):
                            break
                        j += 1
                for j in range(len(non_obstacles)):
                    if target.overlaps_circle(non_obstacles[j]):
                        new_position = current_position
                        target.set_center(new_position[0], new_position[1])
                        break
                max_displacement = new_position - current_position
                max_distance = np.dot(max_displacement, max_displacement)
                target.set_center(new_position[0], new_position[1])
                if not math.isclose(max_distance, 0, rel_tol=1e-09):
                    for quadrant in quadrants:
                        if target in quadrant.contents():
                            quadrant.contents().remove(target)
                    quadrants = self._grid.circle_quadrants(target)
                    for quadrant in quadrants:
                        if target not in quadrant.contents() and len(quadrant.leaves()) == 0:
                            quadrant.contents().append(target)
                    target.redraw(self.canvas)
        self.count += 1
        self.animation = self.root.after(int(1000/60), self.move_circles_randomly, magnitude)


class RectangleZone(Rectangle):
    def __init__(self, x, y, width, height):
        super(RectangleZone, self).__init__(x, y, width, height)
        self._circles = []
        self._grid = Quadtree(Quadrant(x, y, width, height))

    def grid(self):
        return self._grid

    def random_circle_coord(self, radius):
        x_range = self._width - 2 * radius
        min_x = self._center[0] - x_range / 2
        x = random() * x_range + min_x
        y_range = self._height - 2 * radius
        min_y = self._center[1] - y_range / 2
        y = random() * y_range + min_y
        return np.array([x, y])

    def add_random_circles(self, n=1, random_radius=False, min_radius=1, max_radius=10, radius=10, overlap=False,
                           iterations=100):
        for i in range(n):
            if random_radius:
                radius = random() * (max_radius - min_radius) + min_radius
            coord = self.random_circle_coord(radius)
            x, y = coord[0], coord[1]
            circle = Circle(x, y, radius)
            if overlap:
                boolean = self._grid.linear_search(circle, overlap)
                quadrants = self._grid.quadtree_search(circle, overlap)
                for quadrant in quadrants:
                    quadrant.contents().append(circle)
                self._grid.contents().append(circle)
            else:
                j = 0
                while j < iterations:
                    boolean = self._grid.linear_search(circle, overlap)
                    quadrants = self._grid.quadtree_search(circle, overlap)
                    if len(quadrants) > 0 and boolean:
                        for quadrant in quadrants:
                            quadrant.contents().append(circle)
                        self._grid.contents().append(circle)
                        break
                    else:
                        coord = self.random_circle_coord(radius)
                        x, y = coord
                        circle.set_center(x, y)
                        j += 1

    def draw(self, canvas, fill="", outline="black"):
        super(RectangleZone, self).draw(canvas)
        self._grid.draw(canvas)


if __name__ == "__main__":
    seed()
    circle_zone = CircleZone(500, 500, 100)
