import pygame
import math
import sys
import numpy as np

SIZE = WIDTH, HEIGHT = 900, 700
WHITE = 255, 255, 255
screen = pygame.display.set_mode(SIZE)
dt = 0.001


class Vehicle:

    def __init__(self, pos, speed, yaw):
        self.p = pos
        self.spd = speed
        self.max_angle = math.radians(45)
        self.yaw = math.radians(yaw % 360)
        self.wheel_angle = 0
        self.steering_angle = 0
        self.v = np.array([self.spd * math.cos(self.yaw), self.spd * math.sin(self.yaw)])
        self.width = 15
        self.length = 30
        self.lateral_control_gain = 10
        self.closed_loop_gain = 1000
        self.custom_path = np.array([])

    def error_calc(self, track):
        shortest_dist = np.cross(track[1] - track[0], track[0] - self.p) / np.linalg.norm(track[1] - track[0])
        return shortest_dist

    def update_steering(self, track):
        # circle_scenario(self)
        # line_scenario(self, track)
        custom_scenario(self)
        if self.steering_angle > math.pi:
            self.steering_angle -= 2 * math.pi
        elif self.steering_angle < -math.pi:
            self.steering_angle += 2 * math.pi

        if self.steering_angle > self.max_angle:
            self.steering_angle = self.max_angle
        elif self.steering_angle < -self.max_angle:
            self.steering_angle = -self.max_angle

    def update_yaw(self):
        self.yaw += self.spd * math.sin(self.wheel_angle) / self.length * dt
        self.yaw %= math.pi * 2

    def update_pos(self):
        self.p += np.array([self.v[0] * dt, self.v[1] * dt])

    def update_vel(self):
        self.v = np.array(
            [self.spd * math.cos(-self.yaw - self.wheel_angle), self.spd * math.sin(self.yaw + self.wheel_angle)])

    def update_wheel_angle(self):
        error = self.steering_angle - self.wheel_angle
        self.wheel_angle += error * self.closed_loop_gain * dt

    def draw_car(self):
        pygame.draw.circle(screen, (0, 0, 0), (self.p[0], self.p[1]), 3)
        pygame.draw.circle(screen, (0, 0, 0), (
            self.p[0] - 0.5 * self.width * math.sin(-self.yaw), self.p[1] - 0.5 * self.width * math.cos(-self.yaw)), 1)
        pygame.draw.circle(screen, (0, 0, 0), (
            self.p[0] + 0.5 * self.width * math.sin(-self.yaw), self.p[1] + 0.5 * self.width * math.cos(-self.yaw)), 1)
        pygame.draw.circle(screen, (0, 0, 0), (
            self.p[0] - 0.5 * self.width * math.sin(-self.yaw) - self.length * math.cos(-self.yaw),
            self.p[1] - 0.5 * self.width * math.cos(-self.yaw) + self.length * math.sin(-self.yaw)), 1)
        pygame.draw.circle(screen, (0, 0, 0), (
            self.p[0] + 0.5 * self.width * math.sin(-self.yaw) - self.length * math.cos(-self.yaw),
            self.p[1] + 0.5 * self.width * math.cos(-self.yaw) + self.length * math.sin(-self.yaw)), 1)


def line_scenario(vehicle, track):
    phi = np.arctan2((track[1][1] - track[0][1]), (track[1][0] - track[0][0])) - vehicle.yaw
    e = vehicle.error_calc(track)
    vehicle.steering_angle = phi + np.arctan2(vehicle.lateral_control_gain * e, np.linalg.norm(vehicle.v))

    pygame.draw.line(screen, (0, 0, 0), track[0], track[1])


def circle_scenario(vehicle):
    circle_pos = np.array([400, 400])
    circle_rad = 100

    phi = np.arctan2(-(vehicle.p[0] - circle_pos[0]), (vehicle.p[1] - circle_pos[1])) - vehicle.yaw
    e = -(np.linalg.norm(circle_pos - vehicle.p) - circle_rad)
    vehicle.steering_angle = phi + np.arctan2(vehicle.lateral_control_gain * e, np.linalg.norm(vehicle.v))

    pygame.draw.circle(screen, (0, 0, 0), circle_pos, circle_rad, 1)

def custom_scenario(vehicle):
    if len(vehicle.custom_path) == 0:
        vehicle.custom_path = np.random.rand(5,2) * np.array([WIDTH, HEIGHT])
        vehicle.custom_path[0] = [300, 500]
    for idx, coord in enumerate(vehicle.custom_path):
        if idx != len(vehicle.custom_path) - 1:
            pygame.draw.line(screen, (0, 0, 0), vehicle.custom_path[idx], vehicle.custom_path[idx+1])
    if np.linalg.norm(vehicle.custom_path[1] - vehicle.p) < 20:
        vehicle.custom_path = vehicle.custom_path[1:]
        vehicle.custom_path = np.concatenate((vehicle.custom_path, np.random.rand(1,2) * np.array([WIDTH, HEIGHT])), axis=0)
        print(vehicle.custom_path)
    phi = np.arctan2((vehicle.custom_path[1][1] - vehicle.custom_path[0][1]), (vehicle.custom_path[1][0] - vehicle.custom_path[0][0])) - vehicle.yaw
    e = vehicle.error_calc(vehicle.custom_path)
    vehicle.steering_angle = phi + np.arctan2(vehicle.lateral_control_gain * e, np.linalg.norm(vehicle.v))



if __name__ == "__main__":
    pygame.init()
    car = Vehicle(np.array([300.0, 500.0]), 1000, 180)
    font = pygame.font.SysFont("calibri", 16)
    time_elapsed = 0
    while True:
        screen.fill(WHITE)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()

        car.update_steering([np.array([0, 300]), np.array([1000, 300])])
        car.update_wheel_angle()
        car.update_yaw()
        car.update_vel()
        car.update_pos()
        car.draw_car()

        text_time = font.render(f'Elapsed Time: {time_elapsed}', False, (0, 0, 0))
        text_yaw = font.render(f'Yaw Angle: {car.yaw * 180 / math.pi}', False, (0, 0, 0))
        text_steering = font.render(f'Steering Angle: {car.steering_angle * 180 / math.pi}', False, (0, 0, 0))
        text_wheel = font.render(f'Wheel Angle: {car.wheel_angle * 180 / math.pi}', False, (0, 0, 0))

        time_elapsed += dt
        screen.blit(text_time, (0, 0))
        screen.blit(text_yaw, (0, 16))
        screen.blit(text_steering, (0, 32))
        screen.blit(text_wheel, (0, 48))

        pygame.display.flip()
