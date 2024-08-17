# pylint: disable=missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods

# the original particle filter thing i borrowed from https://github.com/mjl/particle_filter_demo
# does not use numpy at all.
# this does, as a step towards using cupy.

import math
import random
import bisect
import time
import numpy as np
from matplotlib import pyplot as plt  # type:ignore
from matplotlib.animation import FuncAnimation  # type:ignore

PARTICLE_COUNT = 1000
ROBOT_HAS_COMPASS = False
ROBOT_SPEED = 0.1
WIDTH = 5
HEIGHT = 5
BEACONS = np.array([[0.5, 0.5], [0.5, 4.5]])


def distance(x1, y1, x2, y2):
    return math.hypot((x1 - x2), (y1 - y2))


def all_beacon_distance(x: float, y: float) -> list[float]:
    d = []
    for c_x, c_y in BEACONS:
        d.append(distance(c_x, c_y, x, y))
    return d


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

    fig = plt.figure()
    axis = plt.axes(xlim=(-1, 6), ylim=(-1, 6))
    particle_points = axis.scatter([], [], marker=".", s=1)
    robot_points = axis.scatter([], [], marker="o")
    mean_points = axis.scatter([], [], s=100, facecolors="none", edgecolors="black")

    axis.scatter(BEACONS[:, 0], BEACONS[:, 1])

    particles = create_random(PARTICLE_COUNT)

    robot_x = WIDTH / 4
    robot_y = HEIGHT / 2
    robot_h = 270

    def init():
        particle_points.set_offsets(([], []))
        robot_points.set_offsets(([], []))
        mean_points.set_offsets(([], []))
        return (
            particle_points,
            robot_points,
            mean_points,
        )

    def animate(i):

        nonlocal particles
        nonlocal robot_x
        nonlocal robot_y
        nonlocal robot_h

        t0 = time.time_ns()

        reweight(particles, robot_x, robot_y)

        if i % 5 == 0:
            x, y = compute_mean_point(particles)
            mean_points.set_offsets(np.column_stack([[x], [y]]))
            p_x = [p.x for p in particles]
            p_y = [p.y for p in particles]
            particle_points.set_offsets(np.column_stack([p_x, p_y]))
            robot_points.set_offsets(np.column_stack([[robot_x], [robot_y]]))

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
        if i % 2 == 0:
            print(f"duration (us): {duration//1000}")
            print(f"duration per particle (us): {duration//(1000*PARTICLE_COUNT)}")

        return (
            particle_points,
            robot_points,
            mean_points,
        )

    anim = FuncAnimation(
        fig, animate, init_func=init, frames=200, interval=0, blit=True
    )

    plt.show()


if __name__ == "__main__":
    main()
