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


def all_beacon_distance(x: float, y: float) -> list[float]:
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
    turtle.up()
    turtle.color("#00ffff")
    for x, y in BEACONS:
        turtle.setposition(x, y)
        turtle.dot(10)
    turtle.screen.update()


def show_mean(turtle, particles):
    x, y = compute_mean_point(particles)
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


def show_robot(turtle, x, y, h):
    turtle.color("green")
    turtle.shape("classic")
    turtle.setposition(x, y)
    turtle.setheading(h)
    turtle.stamp()
    turtle.screen.update()


def w_gauss(a: list[float], b: list[float]) -> float:
    sqsum: float = 0.0
    for aa, bb in zip(a, b):
        sqsum += (aa - bb) * (aa - bb)
    g: float = math.exp(-1.0 * (sqsum / (0.1)))
    return g


def compute_mean_point(particles) -> tuple[float, float]:
    m_x = 0.0
    m_y = 0.0
    m_count = 0.0
    for p in particles:
        m_count += p.w
        m_x += p.x * p.w
        m_y += p.y * p.w
    m_x /= m_count
    m_y /= m_count
    return m_x, m_y


class Particle:
    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.h = heading
        self.w = 1


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

    accum = 0.0
    state = [p for p in particles if p.w > 0]
    distribution = []
    for x in state:
        accum += x.w
        distribution.append(accum)

    for _ in particles:
        try:
            p = state[bisect.bisect_left(distribution, random.uniform(0, 1))]
            new_particle = duplicate(p)
        except IndexError:
            # Happens when all particles are improbable w=0
            new_particle = create_random(1)[0]

        new_particles.append(new_particle)

    particles = new_particles
    return particles


def reweight(particles, robot_x: float, robot_y: float) -> None:
    distances: list[float] = all_beacon_distance(robot_x, robot_y)
    for p in particles:
        if p.x < 0 or p.y < 0 or p.x > WIDTH or p.y > HEIGHT:
            p.w = 0
        else:
            p_d = all_beacon_distance(p.x, p.y)
            p.w = w_gauss(distances, p_d)


def main():
    turtle = Turtle()
    init(turtle)

    particles = create_random(PARTICLE_COUNT)

    robot_x = WIDTH / 4
    robot_y = HEIGHT / 2
    robot_h = 270

    loop_counter = 0

    while True:
        loop_counter += 1
        t0 = time.time_ns()

        reweight(particles, robot_x, robot_y)

        if loop_counter % 5 == 0:
            show_particles(turtle, particles)
            show_mean(turtle, particles)
            show_robot(turtle, robot_x, robot_y, robot_h)

        particles = resample(particles)

        old_heading = robot_h

        robot_h += 5
        r = math.radians(robot_h)
        robot_x += math.cos(r) * ROBOT_SPEED
        robot_y += math.sin(r) * ROBOT_SPEED

        d_h = robot_h - old_heading

        # Move particles according to my belief of movement (this may
        # be different than the real movement, but it's all I got)
        for p in particles:
            p.h += d_h
            r = math.radians(p.h)
            p.x += math.cos(r) * ROBOT_SPEED
            p.y += math.sin(r) * ROBOT_SPEED

        t1 = time.time_ns()
        duration = t1 - t0
        if loop_counter % 2 == 0:
            print(f"duration (us): {duration//1000}")
            print(f"duration per particle (us): {duration//(1000*PARTICLE_COUNT)}")


if __name__ == "__main__":
    main()
