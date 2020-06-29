import numpy as np
import math


class Graphic2D:
    def draw(self, canvas, fill="", outline="black"):
        pass

    def redraw(self, canvas, fill="", outline="black"):
        pass


class Collider2D:
    def collides_circle(self, circle):
        pass


class Shape2D(Graphic2D):
    def __init__(self, x, y):
        self._center = np.array([x, y])
        self._item = None

    def get_center(self):
        return np.copy(self._center)

    def set_center(self, x, y):
        self._center[0] = x
        self._center[1] = y

    def get_item(self):
        return self._item

    def set_item(self, item):
        self._item = item

    def squared_distance_from_point(self, point):
        vector = point.get_center() - self._center
        return np.dot(vector, vector)

    def distance_from_point(self, point):
        vector = point.get_center() - self._center
        return math.sqrt(np.dot(vector, vector))


class Circle(Shape2D, Collider2D):
    def __init__(self, x, y, radius):
        super(Circle, self).__init__(x, y)
        self._radius = radius
    
    def get_radius(self):
        return self._radius
    
    def set_radius(self, radius):
        self._radius = radius
         
    def draw(self, canvas, fill="", outline="black"):
        if not self._item:
            x, y = self._center
            x1 = x - self._radius
            x2 = x + self._radius
            y1 = y - self._radius
            y2 = y + self._radius
            self._item = canvas.create_oval(x1, y1, x2, y2, fill=fill, outline=outline)

    def redraw(self, canvas, fill="", outline="black"):
        if self._item:
            x, y = self._center
            x1 = x - self._radius
            x2 = x + self._radius
            y1 = y - self._radius
            y2 = y + self._radius
            canvas.coords(self._item, x1, y1, x2, y2)
            canvas.itemconfig(self._item, fill=fill, outline=outline)

    def collides_circle(self, circle):
        vector = self._center - circle.get_center()
        distance = np.dot(vector, vector)
        threshold = math.pow(self._radius + circle.get_radius(), 2)
        return distance < threshold or math.isclose(distance, threshold)

    def detects_circle(self, circle):
        vector = self._center - circle.get_center()
        distance = np.dot(vector, vector)
        threshold = math.pow(self._radius, 2)
        return distance < threshold or math.isclose(distance, threshold)

    def confines_circle(self, circle):
        vector = circle.get_center() - self._center
        distance = np.dot(vector, vector)
        threshold = math.pow(self._radius - circle.get_radius(), 2)
        return distance < threshold or math.isclose(distance, threshold)

    def overlaps_circle(self, circle, epsilon):
        vector = circle.get_center() - self._center
        distance = np.dot(vector, vector)
        threshold = math.pow(self._radius + circle.get_radius() - epsilon, 2)
        return distance < threshold or math.isclose(distance, threshold)


class Rectangle(Shape2D):
    def __init__(self, x, y, width, height):
        super(Rectangle, self).__init__(x, y)
        self._width = width
        self._height = height

    def get_center(self):
        return self._center

    def get_width(self):
        return self._width

    def set_width(self, width):
        self._width = width

    def get_height(self):
        return self._height

    def set_height(self, height):
        self._height = height

    def draw(self, canvas, fill="", outline="black"):
        if not self._item:
            x = self._center[0]
            y = self._center[1]
            width = self._width / 2
            height = self._height / 2
            x1 = x - width
            x2 = x + width
            y1 = y - height
            y2 = y + height
            self._item = canvas.create_rectangle(x1, y1, x2, y2, fill=fill, outline=outline)

    def redraw(self, canvas, fill="", outline="black"):
        if self._item:
            canvas.itemconfig(self._item, fill=fill, outline=outline)

    def contains_point(self, point):
        x, y = point
        cx, cy = self._center
        x1 = cx - self._width / 2
        x2 = cx + self._width / 2
        y1 = cy - self._height / 2
        y2 = cy + self._height / 2
        return x1 <= x <= x2 and y1 <= y <= y2

    def detects_circle(self, circle):
        cx = self._center[0]
        cy = self._center[1]
        x, y = circle.get_center()
        left = x + circle.get_radius() > cx - self._width / 2
        right = x - circle.get_radius() < cx + self._width / 2
        top = y + circle.get_radius() > cy - self._height / 2
        bottom = y - circle.get_radius() < cy + self._height / 2
        return left and right and top and bottom

    def confines_circle(self, circle):
        cx = self._center[0]
        cy = self._center[1]
        x, y = circle.get_center()
        left = x - circle.get_radius() > cx - self._width / 2\
            or math.isclose(x - circle.get_radius(), cx - self._width / 2)
        right = x + circle.get_radius() < cx + self._width / 2\
            or math.isclose(x + circle.get_radius(), cx + self._width / 2)
        top = y - circle.get_radius() > cy - self._height / 2\
            or math.isclose(y - circle.get_radius(), cy - self._height / 2)
        bottom = y + circle.get_radius() < cy + self._height / 2\
            or math.isclose(y + circle.get_radius(), cy + self._height / 2)
        return left and right and top and bottom
