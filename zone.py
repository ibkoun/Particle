from graphic import Circle, Rectangle
from node import Quadrant, Quadtree
import math
import numpy as np
from random import random
from random import seed
import tkinter as tk
import formula


class CircleZone(Circle):
    def __init__(self, x, y, radius):
        super(CircleZone, self).__init__(x, y, radius)
        self._circles = []
        self._grid = Quadtree(Quadrant(x, y, 2 * radius, 2 * radius))

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

    def move_circles_randomly(self, magnitude, canvas):
        for i in range(1):  # n = 1 for testing purpose.
            # Randomize the direction of the movement.
            angle = math.radians(random() * 360)
            x = magnitude * math.cos(angle)
            y = magnitude * math.sin(angle)
            displacement = np.array([x, y])
            distance = np.dot(displacement, displacement)

            # Get the circle's information.
            target = self._circles[i]
            target_radius = target.get_radius()
            current_position = target.get_center()

            # Calculate the circle's new position.
            new_position = current_position + displacement
            target_quadrants = self._grid.circle_quadrants(target)
            target.set_center(new_position[0], new_position[1])

            # Check if the circle is still inside the zone.
            if not self.confines_circle(target):
                new_position = formula.point_on_circumference(self._radius - target_radius, self._center,
                                                              current_position, new_position)
                target.set_center(new_position[0], new_position[1])
                displacement = new_position - current_position
                distance = np.dot(displacement, displacement)

            # Check if the circle collides with other circles along its path.
            if distance > 0:
                quadrants = self._grid.rectangle_overlap(current_position, new_position, target_radius, canvas)
                trajectory = formula.Segment(current_position, new_position)
                obstacles = []
                for quadrant in quadrants:
                    contents = quadrant.contents()
                    for content in contents:
                        if content != target:
                            point = content.get_center()
                            vector = point - current_position
                            distance_from_trajectory = trajectory.squared_distance_from_point(point)
                            distance_from_obstacle = target.squared_distance_from_point(point)
                            if distance_from_trajectory < math.pow(target.get_radius() + content.get_radius(), 2):
                                distance_along_trajectory = distance_from_obstacle - distance_from_trajectory \
                                    if distance_from_trajectory > 0 else distance_from_obstacle
                                threshold = math.sqrt(np.dot(displacement, displacement)) + target_radius \
                                            + content.get_radius()
                                if distance_along_trajectory < math.pow(threshold, 2):
                                    theta = formula.angle_between(displacement, vector)
                                    if abs(theta) < 90:
                                        obstacles.append(content)
                    obstacles.sort(key=lambda circle: target.distance_from_point(circle.get_center()))
                    j = 0
                    while j < len(obstacles):
                        point = obstacles[j].get_center()
                        distance_from_trajectory = trajectory.squared_distance_from_point(point)
                        if math.isclose(distance_from_trajectory, 0):
                            displacement = formula.resize_vector(displacement,
                                                                 target_radius + obstacles[j].get_radius())
                            new_position = point - displacement
                        else:
                            distance_from_obstacle = math.pow(obstacles[j].get_radius() + target_radius, 2)
                            projection = formula.project_vector(point - current_position, displacement)
                            distance_from_position = math.sqrt(distance_from_obstacle - distance_from_trajectory)
                            displacement = formula.resize_vector(displacement, distance_from_position)
                            position = current_position + projection - displacement
                            vector = position - current_position
                            distance = np.dot(vector, vector)
                            squared_magnitude = math.pow(magnitude, 2)
                            if distance < squared_magnitude or math.isclose(distance, squared_magnitude):
                                new_position = position
                        target.set_center(new_position[0], new_position[1])
                        j += 1
                        while j < len(obstacles):
                            if target.collides_circle(obstacles[j]):
                                break
                            j += 1
                if not self.confines_circle(target):
                    new_position = formula.point_on_circumference(self._radius - target_radius, self._center,
                                                                  current_position, new_position)
                    target.set_center(new_position[0], new_position[1])
                if not np.all(np.isclose(current_position, new_position)):
                    for quadrant in target_quadrants:
                        if target in quadrant.contents():
                            quadrant.contents().remove(target)
                    target_quadrants = self._grid.circle_quadrants(target, True)
                    for quadrant in target_quadrants:
                        if target not in quadrant.contents() and len(quadrant.leaves()) == 0:
                            quadrant.contents().append(target)
                target.redraw(canvas)


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

    class App:
        def __init__(self):
            self.root = tk.Tk()
            self.canvas = tk.Canvas(self.root, width=1000, height=1000)
            self.canvas.pack()
            self.circle_zone = CircleZone(500, 500, 250)
            self.m = 0
            self.n = 100
            self.circle_zone.add_random_circles(self.n)
            # self.circleZone.grid().linear_search_result()
            # print()
            self.circle_zone.grid().quadtree_search_result()
            print()
            self.circle_zone.grid().result(self.n)
            self.circle_zone.draw(self.canvas)
            self.animate()
            self.root.mainloop()

        def animate(self):
            self.circle_zone.move_circles_randomly(10, self.canvas)
            # self.circle_zone.grid().redraw(self.canvas)
            self.root.after(int(1000/60), self.animate)

    app = App()
