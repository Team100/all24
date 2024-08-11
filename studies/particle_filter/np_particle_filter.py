# pylint: disable=missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods

# the original particle filter thing i borrowed from https://github.com/mjl/particle_filter_demo
# does not use numpy at all.
# this does, as a step towards using cupy.

import math
from turtle import Turtle
import random
import bisect
import time

PARTICLE_COUNT = 1000
ROBOT_HAS_COMPASS = False
ROBOT_SPEED = 0.1
WIDTH = 5
HEIGHT = 5

BEACONS = [(0.5, 0.5), (0.5, 4.5)]


def weight_to_color(weight):
    red = weight
    blue = 1 - weight
    return (red, 0, blue)


def distance(x1, y1, x2, y2):
    return math.hypot((x1 - x2), (y1 - y2))


def all_beacon_distance(x, y):
    d = []
    for c_x, c_y in BEACONS:
        d.append(distance(c_x, c_y, x, y))
    return d

def init(turtle):
    turtle.screen.mode("standard")
    turtle.screen.tracer(50000, delay=0)
    turtle.screen.register_shape("dot", ((-3, -3), (-3, 3), (3, 3), (3, -3)))
    turtle.screen.register_shape("tri", ((-3, -2), (0, 3), (3, -2), (0, 0)))
    turtle.speed(0)
    turtle.screen.title("particle filter demo")
    turtle.screen.setworldcoordinates(0, 0, WIDTH, HEIGHT)


def draw(turtle):
    turtle.up()
    turtle.color("#00ffff")
    for x, y in BEACONS:
        turtle.setposition(x, y)
        turtle.dot(10)
    turtle.screen.update()


def show_mean(turtle, x, y):
    turtle.color("#000000")
    turtle.setposition(x, y)
    turtle.shape("circle")
    turtle.stamp()



def show_particles(turtle, particles):
    turtle.clearstamps()
    turtle.shape("tri")
    for p in particles:
        turtle.setposition(p.x, p.y)
        turtle.setheading(p.h)
        turtle.color(weight_to_color(p.w))
        turtle.stamp()

def show_robot(turtle, robot):
    turtle.color("green")
    turtle.shape("classic")
    turtle.setposition(robot.x, robot.y)
    turtle.setheading(robot.h)
    turtle.stamp()
    turtle.screen.update()


def w_gauss(a, b):
    sqsum = 0
    for aa, bb in zip(a, b):
        sqsum += (aa - bb) * (aa - bb)
    g = math.exp(-1.0 * (sqsum / (0.1)))
    return g


def compute_mean_point(particles):
    m_x, m_y, m_count = 0, 0, 0
    for p in particles:
        m_count += p.w
        m_x += p.x * p.w
        m_y += p.y * p.w
    m_x /= m_count
    m_y /= m_count
    return m_x, m_y


class WeightedDistribution:
    def __init__(self, state):
        accum = 0.0
        self.state = [p for p in state if p.w > 0]
        self.distribution = []
        for x in self.state:
            accum += x.w
            self.distribution.append(accum)

    def pick(self):
        try:
            return self.state[
                bisect.bisect_left(self.distribution, random.uniform(0, 1))
            ]
        except IndexError:
            # Happens when all particles are improbable w=0
            return None


class Particle:
    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.h = heading
        self.w = 1

    def move(self, speed):
        r = math.radians(self.h)
        self.x += math.cos(r) * speed
        self.y += math.sin(r) * speed


class Robot:
    def __init__(self):
        self.x = WIDTH / 4
        self.y = HEIGHT / 2
        self.h = 270

    def move(self):
        self.h += 5
        r = math.radians(self.h)
        self.x += math.cos(r) * ROBOT_SPEED
        self.y += math.sin(r) * ROBOT_SPEED


def create_random(count):
    return [
        Particle(
            random.uniform(0, WIDTH), random.uniform(0, HEIGHT), random.uniform(0, 360)
        )
        for _ in range(0, count)
    ]


def duplicate(p):
    x = p.x + random.uniform(-0.1, 0.1)
    y = p.y + random.uniform(-0.1, 0.1)
    h = p.h + random.uniform(-0.1, 0.1)
    return Particle(x, y, h)


def resample(particles):
    new_particles = []

    nu = sum(p.w for p in particles)
    if nu:
        for p in particles:
            p.w = p.w / nu

    dist = WeightedDistribution(particles)

    for _ in particles:
        p = dist.pick()
        if p is None:
            new_particle = create_random(1)[0]
        else:
            new_particle = duplicate(p)
        new_particles.append(new_particle)

    particles = new_particles
    return particles


def reweight(particles, r_d):
    for p in particles:
        if p.x < 0 or p.y < 0 or p.x > WIDTH or p.y > HEIGHT:
            p.w = 0
        else:
            p_d = all_beacon_distance(p.x, p.y)
            p.w = w_gauss(r_d, p_d)


def main():

    turtle = Turtle()
    init(turtle)
    draw(turtle)

    particles = create_random(PARTICLE_COUNT)

    robbie = Robot()

    loop_counter = 0

    while True:
        loop_counter += 1
        # time per iteration
        t0 = time.time_ns()

        distances = all_beacon_distance(robbie.x, robbie.y)

        reweight(particles, distances)

        m_x, m_y = compute_mean_point(particles)

        show_particles(turtle, particles)
        show_mean(turtle, m_x, m_y)
        show_robot(turtle, robbie)

        particles = resample(particles)

        old_heading = robbie.h
        robbie.move()
        d_h = robbie.h - old_heading

        # Move particles according to my belief of movement (this may
        # be different than the real movement, but it's all I got)
        for p in particles:
            p.h += d_h  # in case robot changed heading, swirl particle heading too
            p.move(ROBOT_SPEED)

        t1 = time.time_ns()
        duration = t1 - t0
        if loop_counter > 100:
            loop_counter = 0
            print(f"duration (us): {duration//1000}")
            print(f"duration per particle (us): {duration//(1000*PARTICLE_COUNT)}")


if __name__ == "__main__":
    main()
