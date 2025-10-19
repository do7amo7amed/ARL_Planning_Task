from __future__ import annotations
import math
from typing import List,Tuple
from src.models import CarPose, Cone, Path2D


class PathPlanning:
    """Student-implemented path planner.

    You are given the car pose and an array of detected cones, each cone with (x, y, color)
    where color is 0 for yellow (right side) and 1 for blue (left side). The goal is to
    generate a sequence of path points that the car should follow.

    Implement ONLY the generatePath function.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        """Return a list of path points (x, y) in world frame.

        Requirements and notes:
        - Cones: color==0 (yellow) are on the RIGHT of the track; color==1 (blue) are on the LEFT.
        - You may be given 2, 1, or 0 cones on each side.
        - Use the car pose (x, y, yaw) to seed your path direction if needed.
        - Return a drivable path that stays between left (blue) and right (yellow) cones.
        - The returned path will be visualized by PathTester.

        The path can contain as many points as you like, but it should be between 5-10 meters,
        with a step size <= 0.5. Units are meters.

        Replace the placeholder implementation below with your algorithm.
        """
        #cones: yellow (right),blue (left)
        yellow_cones = [c for c in self.cones if c.color == 0]
        blue_cones = [c for c in self.cones if c.color == 1]
    
        yellow_cones.sort(key=lambda c: self._dist(c))
        blue_cones.sort(key=lambda c: self._dist(c))

        # Midpoint 
        midpoints: List[Tuple[float, float]] = []
        path: Path2D = []

        # 1: both sides visible 
        if blue_cones and yellow_cones:
            num_pairs = min(len(blue_cones), len(yellow_cones))
            for i in range(num_pairs):
                mx = (blue_cones[i].x + yellow_cones[i].x) / 2
                my = (blue_cones[i].y + yellow_cones[i].y) / 2
                midpoints.append((mx, my))

        #  2: one side visible 
        elif blue_cones or yellow_cones:
            track_width = 3.0  
            cones = blue_cones if blue_cones else yellow_cones
            for c in cones:
                if blue_cones:
                    dx = math.cos(self.car_pose.yaw - math.pi / 2)
                    dy = math.sin(self.car_pose.yaw - math.pi / 2)
                    yellow_guess = (c.x + track_width * dx, c.y + track_width * dy)
                    mx = (c.x + yellow_guess[0]) / 2
                    my = (c.y + yellow_guess[1]) / 2
                else:
                # Guess missing blue cone
                    dx = math.cos(self.car_pose.yaw + math.pi / 2)
                    dy = math.sin(self.car_pose.yaw + math.pi / 2)
                    blue_guess = (c.x + track_width * dx, c.y + track_width * dy)
                    mx = (c.x + blue_guess[0]) / 2
                    my = (c.y + blue_guess[1]) / 2

                midpoints.append((mx, my))  

        #  3: no cones visible 
        else:
            mx = self.car_pose.x + 3.0 * math.cos(self.car_pose.yaw)
            my = self.car_pose.y + 3.0 * math.sin(self.car_pose.yaw)
            midpoints.append((mx, my))

        # Start path from car position
        start = (self.car_pose.x, self.car_pose.y)

        if midpoints:
            path.append(start)

            # Connect all midpoints smoothly
            for i in range(len(midpoints) - 1):
                x0, y0 = midpoints[i]
                x1, y1 = midpoints[i + 1]
                steps = 10
                for j in range(steps):
                    t = j / (steps - 1)
                    x = x0 + t * (x1 - x0)
                    y = y0 + t * (y1 - y0)
                    path.append((x, y))

            # Extend forward 
            if len(midpoints) >= 2:
                x_last, y_last = midpoints[-1]
                x_prev, y_prev = midpoints[-2]
                heading = math.atan2(y_last - y_prev, x_last - x_prev)
            else:
                heading = self.car_pose.yaw

            step = 0.5
            for k in range(10):  
                x_ext = midpoints[-1][0] + step * k * math.cos(heading)
                y_ext = midpoints[-1][1] + step * k * math.sin(heading)
                path.append((x_ext, y_ext))

        return path

    def _dist(self, cone: Cone) -> float:
        """Distance from car to cone."""
        return math.hypot(cone.x - self.car_pose.x, cone.y - self.car_pose.y)
    