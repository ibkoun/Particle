import numpy as np
import formula
import math
import tkinter as tk
from node import Quadtree, Quadrant
from abc import ABCMeta
from graphic import Circle, Rectangle
from random import random, seed


# Particle behavior interfaces.
class MotionBehavior(metaclass=ABCMeta):
    @classmethod
    def __subclasscheck__(cls, subclass):
        return (hasattr(subclass, 'move') and callable(subclass.move) and
                hasattr(subclass, 'rotate') and callable(subclass.rotate) or
                NotImplemented)

    def move(self, magnitude, direction=None, min_angle=0, max_angle=360, phasing=False):
        raise NotImplementedError

    def rotate(self, angle):
        raise NotImplementedError


class TrackingBehavior(metaclass=ABCMeta):
    @classmethod
    def __subclasscheck__(cls, subclass):
        return (hasattr(subclass, 'search') and callable(subclass.search) and
                hasattr(subclass, 'destroy') and callable(subclass.destroy) or
                NotImplemented)

    def search(self):
        raise NotImplementedError

    def destroy(self, target):
        raise NotImplementedError


class Particle(Circle, MotionBehavior, TrackingBehavior):
    def __init__(self, x=0, y=0, radius=1, field_of_view=None, tag=None, world=None):
        super(Particle, self).__init__(x, y, radius)
        self.field_of_view = field_of_view
        self.tag = tag
        self.world = world

    def move(self, magnitude, direction=None, min_angle=0, max_angle=360, phasing=False):
        assert min_angle <= max_angle, "The minimum angle must be smaller than maximum angle."
        angle = math.degrees(math.acos(direction[0] / 0)) if direction \
            else math.radians(random() * (max_angle - min_angle) + min_angle)
        x = magnitude * math.cos(angle)
        y = magnitude * math.sin(angle)
        displacement = np.array([x, y])
        squared_distance = np.dot(displacement, displacement)
        departure = self.get_center()
        destination = departure + displacement
        self.set_center(destination[0], destination[1])

        # Check if the particle is still inside the zone.
        if not self.world.shape.confines_circle(self):
            destination = self.world.shape.confine_circle_coord(self, departure, destination)
            displacement = destination - departure
            squared_distance = np.dot(displacement, displacement)
            self.set_center(destination[0], destination[1])
            if not self.world.shape.confines_circle(self):
                canvas.create_oval(destination[0] - self.get_radius(), destination[1] - self.get_radius(),
                                   destination[0] + self.get_radius(), destination[1] + self.get_radius(), fill="red")

        # Check if the particle collides with other particles along its path.
        if not math.isclose(squared_distance, 0, rel_tol=1e-09):
            self.set_center(departure[0], departure[1])
            quadrants = self.world.grid.rectangle_overlap(departure, destination, self.get_radius(), canvas)
            trajectory = formula.Segment(departure, destination)
            particles = set()
            obstacles = set()
            for quadrant in quadrants:
                contents = quadrant.contents()
                for content in contents:
                    if content != self:
                        particles.add(content)
                        point = content.get_center()
                        vector = point - departure
                        theta = formula.angle_between(displacement, vector)
                        if theta < 90 and not math.isclose(theta, 90):
                            distance_from_trajectory = trajectory.squared_distance_from_point(point)
                            distance_from_obstacle = self.squared_distance_from_point(point)
                            rectangle_width = self.get_radius() + content.get_radius()
                            if distance_from_trajectory < math.pow(rectangle_width, 2):
                                distance_along_trajectory = distance_from_obstacle - distance_from_trajectory \
                                    if not math.isclose(distance_from_trajectory, 0, rel_tol=1e-09) \
                                    else distance_from_obstacle
                                rectangle_length = math.sqrt(
                                    squared_distance) + self.get_radius() + content.get_radius()
                                if distance_along_trajectory < math.pow(rectangle_length, 2):
                                    obstacles.add(content)
                                    # content.redraw(canvas, fill="red")
            obstacles = list(obstacles)
            obstacles.sort(key=lambda particle: self.distance_from_circle(particle))
            non_obstacles = list(particles.difference(obstacles))
            non_obstacles.sort(key=lambda particle: self.distance_from_circle(particle))

            # Movement stops at the nearest obstacle.
            j = 0
            while j < len(obstacles):
                point = obstacles[j].get_center()
                distance_from_trajectory = trajectory.squared_distance_from_point(point)
                distance_from_obstacle = math.pow(obstacles[j].get_radius() + self.get_radius(), 2)
                projection = formula.project_vector(point - departure, displacement)
                distance_from_position = math.sqrt(distance_from_obstacle - distance_from_trajectory)
                delta = formula.resize_vector(displacement, distance_from_position)
                destination = departure + projection - delta
                vector = destination - departure
                squared_magnitude = np.dot(vector, vector)

                # Check if the magnitude of the corrected displacement is less than or equal to the original.
                if squared_magnitude > squared_distance and not math.isclose(squared_magnitude, squared_distance):
                    displacement = formula.resize_vector(vector, math.sqrt(squared_distance))
                    destination = departure + displacement
                    squared_distance = np.dot(displacement, displacement)

                # Check if the particle collides with other obstacles.
                self.set_center(destination[0], destination[1])
                j += 1
                while j < len(obstacles):
                    if self.overlaps_circle(obstacles[j]):
                        break
                    j += 1

            # Check if the particle overlaps other particles that were outside the path.
            for j in range(len(non_obstacles)):
                if self.overlaps_circle(non_obstacles[j]):
                    destination = departure
                    break

            # Update the coordinate of the particle.
            displacement = destination - departure
            squared_distance = np.dot(displacement, displacement)
            self.set_center(destination[0], destination[1])

            # Update the quadtree.
            if not math.isclose(squared_distance, 0, rel_tol=1e-09):
                for quadrant in quadrants:
                    if self in quadrant.contents():
                        quadrant.contents().remove(self)
                quadrants = self.world.grid.overlapped_by_circle(self)
                for quadrant in quadrants:
                    if self not in quadrant.contents() and len(quadrant.leaves()) == 0:
                        quadrant.contents().append(self)
                self.redraw(canvas)

    def rotate(self, angle):
        self._rotation += angle

    def direction(self):
        x = self.field_of_view[0] * math.cos(math.radians(self._rotation))
        y = self.field_of_view[0] * math.sin(math.radians(self._rotation))
        return np.array([x, y])

    def search(self):
        if self.field_of_view and len(self.field_of_view) == 2:
            # Get the central vision.
            facing_direction_vector = self.direction()
            # central_vision_extent = self._center + facing_direction_vector
            # self._center = self._center + formula.resize_vector(facing_direction_vector, self._radius)

            # Get the left outer boundary of the peripheral vision.
            left_outer_boundary_vector = formula.rotate_vector(facing_direction_vector, self.field_of_view[1] / 2)
            left_outer_boundary_extent = self._center + left_outer_boundary_vector
            left_outer_boundary = formula.Segment(self._center, left_outer_boundary_extent)

            # Get the right outer boundary of the peripheral vision.
            right_outer_boundary_vector = formula.rotate_vector(facing_direction_vector, -self.field_of_view[1] / 2)
            right_outer_boundary_extent = self._center + right_outer_boundary_vector
            right_outer_boundary = formula.Segment(self._center, right_outer_boundary_extent)

            # Draw the field of view.
            canvas.create_line(self._center[0], self._center[1], left_outer_boundary_extent[0],
                               left_outer_boundary_extent[1], fill="blue", width=2)
            canvas.create_line(self._center[0], self._center[1], right_outer_boundary_extent[0],
                               right_outer_boundary_extent[1], fill="blue", width=2)
            # circle_zone.canvas.create_line(self._center[0], self._center[1], central_vision_extent[0],
            # central_vision_extent[1], fill="blue", width=2)
            canvas.create_arc(self._center[0] - self.field_of_view[0],
                              self._center[1] - self.field_of_view[0],
                              self._center[0] + self.field_of_view[0],
                              self._center[1] + self.field_of_view[0],
                              outline="blue", width=2, style=tk.ARC,
                              start=360 - (self._rotation + self.field_of_view[1] / 2),
                              extent=self.field_of_view[1])

            # Find the quadrants of the world that are inside the field of view.
            quadrants = []
            queue = [self.world.grid.get_root()]
            particles_searched = 0
            particles_selected = 0
            quadrants_searched = 0
            quadrants_selected = 0
            while len(queue) > 0:
                quadrants_searched += 1
                quadrant = queue.pop(0)
                center = quadrant.get_center()
                width = quadrant.get_width()
                height = quadrant.get_height()

                # The boundaries of the current quadrant.
                x1 = center[0] - width / 2
                x2 = center[0] + width / 2
                y1 = center[1] - height / 2
                y2 = center[1] + height / 2

                # The four corners of the current quadrant.
                north_west = np.array([x1, y1])
                north_east = np.array([x2, y1])
                south_west = np.array([x1, y2])
                south_east = np.array([x2, y2])

                # Vectors obtained by joining the center of the particle to each corner of the quadrant.
                cnw = north_west - self._center
                cne = north_east - self._center
                csw = south_west - self._center
                cse = south_east - self._center

                # Angles between the central vision vector and the previously calculated vectors.
                angle_cnw = formula.angle_between(facing_direction_vector, cnw)
                angle_cne = formula.angle_between(facing_direction_vector, cne)
                angle_csw = formula.angle_between(facing_direction_vector, csw)
                angle_cse = formula.angle_between(facing_direction_vector, cse)
                angle_threshold = self.field_of_view[1] / 2

                # Squared distances obtained from previously calculated vectors.
                sqrd_cnw = np.dot(cnw, cnw)
                sqrd_cne = np.dot(cne, cne)
                sqrd_csw = np.dot(csw, csw)
                sqrd_cse = np.dot(cse, cse)
                squared_distance_threshold = math.pow(self.field_of_view[0], 2)

                # The borders of the current quadrant.
                north_border = formula.Segment(north_west, north_east)
                south_border = formula.Segment(south_west, south_east)
                west_border = formula.Segment(north_west, south_west)
                east_border = formula.Segment(north_east, south_east)

                # Check if the quadrant contains the particle's coordinate or the furthest points of the outer boundaries.
                if quadrant.contains_point(self._center) \
                        or quadrant.contains_point(left_outer_boundary_extent) \
                        or quadrant.contains_point(right_outer_boundary_extent):
                    quadrants.append(quadrant)
                    queue += quadrant.leaves()
                    quadrants_selected += 1

                # Check if the left outer boundary intersects with the quadrant.
                elif north_border.intersects_segment(left_outer_boundary) \
                        or south_border.intersects_segment(left_outer_boundary) \
                        or west_border.intersects_segment(left_outer_boundary) \
                        or east_border.intersects_segment(left_outer_boundary):
                    quadrants.append(quadrant)
                    queue += quadrant.leaves()
                    quadrants_selected += 1

                # Check if the right outer boundary intersects with the quadrant.
                elif north_border.intersects_segment(right_outer_boundary) \
                        or south_border.intersects_segment(right_outer_boundary) \
                        or west_border.intersects_segment(right_outer_boundary) \
                        or east_border.intersects_segment(right_outer_boundary):
                    quadrants.append(quadrant)
                    queue += quadrant.leaves()
                    quadrants_selected += 1

                # Check if quadrant is inside the field of view.
                elif (angle_cnw < angle_threshold and sqrd_cnw < squared_distance_threshold) \
                        or (angle_cne < angle_threshold and sqrd_cne < squared_distance_threshold) \
                        or (angle_csw < angle_threshold and sqrd_csw < squared_distance_threshold) \
                        or (angle_cse < angle_threshold and sqrd_cse < squared_distance_threshold):
                    quadrants.append(quadrant)
                    queue += quadrant.leaves()
                    quadrants_selected += 1

            # Get the particles that are inside the field of view.
            particles = set()
            targets = []
            for quadrant in quadrants:
                quadrant.redraw(canvas, outline="red")
                particles.update(quadrant.contents())
            for particle in particles:
                particles_searched += 1
                if particle != self:
                    vector = particle.get_center() - self._center
                    squared_distance = math.sqrt(np.dot(vector, vector))
                    angle = formula.angle_between(facing_direction_vector, vector)
                    epsilon = (2 * np.dot(vector, vector) - math.pow(particle.get_radius(), 2)) / (
                            2 * np.dot(vector, vector))
                    epsilon = math.degrees(math.acos(epsilon))
                    if (angle - epsilon < self.field_of_view[1] / 2
                        or math.isclose(angle - epsilon, self.field_of_view[1] / 2)) \
                            and (squared_distance < self.field_of_view[0] + particle.get_radius()
                                 or math.isclose(squared_distance, self.field_of_view[0] + particle.get_radius())):
                        # print(str(min_angle) + " <= " + str(a) + " <= " + str(max_angle))
                        targets.append(particle)
                        particle.redraw(canvas, fill="red")
                        particles_selected += 1
            self.redraw(canvas, fill="blue")
            targets.sort(key=lambda target: self.distance_from_circle(target), reverse=True)
            print()
            print("SEARCH")
            print("Quadrants (selected/searched): {}/{}".format(quadrants_selected, quadrants_searched))
            print("Particles (selected/searched): {}/{}".format(particles_selected, particles_searched))
            return targets

    def destroy(self, target):
        pass


class ParticleSystem:
    def __init__(self):
        self.shape = None
        self.grid = None
        self.particles = []

    def make_circle(self, x, y, radius):
        self.shape = Circle(x, y, radius)
        self.grid = Quadtree(Quadrant(x, y, 2 * radius, 2 * radius))
        # TODO: Update the grid if it was already created.

    def make_rectangle(self, x, y, width, height):
        self.shape = Rectangle(x, y, width, height)
        self.grid = Quadtree(Quadrant(x, y, width, height))
        # TODO: Update the grid if it was already created.

    def add_particles(self, n=1, random_radius=False, min_radius=1, max_radius=10, radius=10, overlap=False,
                      iterations=100):
        for i in range(n):
            particle = Particle()
            if random_radius:
                particle.randomize_radius(min_radius, max_radius)
            else:
                particle.set_radius(radius)
            self.shape.randomize_circle_coord(particle)
            # particle.rotation = random() * 360
            # particle.field_of_view = np.array([200, 45])
            particle.world = self
            if overlap:
                quadrants = self.grid.quadtree_search(particle, overlap)
                for quadrant in quadrants:
                    quadrant.contents().append(particle)
                self.grid.contents().append(particle)
            else:
                j = 0
                while j < iterations:
                    quadrants = self.grid.quadtree_search(particle, overlap)
                    if len(quadrants) > 0:
                        for quadrant in quadrants:
                            quadrant.contents().append(particle)
                        self.grid.contents().append(particle)
                        self.particles.append(particle)
                        break
                    else:
                        self.shape.randomize_circle_coord(particle)
                        j += 1

    def draw(self, canvas, fill="", outline="black"):
        self.shape.draw(canvas, fill=fill, outline=outline)
        self.grid.draw(canvas, fill=fill, outline=outline)

    def redraw(self, canvas, fill="", outline="black"):
        self.shape.draw(canvas, fill=fill, outline=outline)
        self.grid.redraw(canvas, fill=fill, outline=outline)

    def draw_particle(self, canvas, index, fill="", outline="black"):
        self.particles[index].draw(canvas, fill=fill, outline=outline)

    def redraw_particle(self, canvas, index, fill="", outline="black"):
        self.particles[index].draw(canvas, fill=fill, outline=outline)

    def draw_particles(self, canvas, fill="", outline="black"):
        for particle in self.particles:
            particle.draw(canvas, fill=fill, outline=outline)

    def redraw_particles(self, canvas, fill="", outline="black"):
        for particle in self.particles:
            particle.redraw(canvas, fill=fill, outline=outline)


def animate():
    for particle in particle_system.particles:
        particle.move(50)
    master.after(int(1000 / 60), animate)


if __name__ == "__main__":
    seed()
    master = tk.Tk()
    canvas = tk.Canvas(master, width=1000, height=1000)
    canvas.pack()
    particle_system = ParticleSystem()
    particle_system.make_circle(500, 500, 200)
    particle_system.add_particles(50)
    particle_system.draw(canvas)
    particle_system.draw_particles(canvas)
    animate()
    master.mainloop()
