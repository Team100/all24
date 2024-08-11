# pylint: disable=missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods

# the original particle filter thing i borrowed from https://github.com/mjl/particle_filter_demo
# does not use numpy at all.
# this does, as a step towards using cupy.

import math
from turtle import Turtle
import random
import bisect
import time

UPDATE_EVERY = 0
DRAW_EVERY = 2
PARTICLE_COUNT = 5000
ROBOT_HAS_COMPASS = False
ROBOT_SPEED = 0.1


maze_data = (
    (2, 0, 0, 0, 0),
    (0, 0, 0, 0, 0),
    (0, 0, 0, 0, 0),
    (0, 0, 0, 0, 0),
    (2, 0, 0, 0, 0),
)


class World(object):
    def __init__(self):
        self.maze = maze_data
        self.turtle = Turtle()
        self.turtle.screen.tracer(50000, delay=0)
        self.turtle.screen.register_shape("dot", ((-3, -3), (-3, 3), (3, 3), (3, -3)))
        self.turtle.screen.register_shape("tri", ((-3, -2), (0, 3), (3, -2), (0, 0)))
        self.turtle.speed(0)
        self.turtle.screen.title("particle filter demo")

        self.width = len(self.maze[0])
        self.height = len(self.maze)
        self.turtle.screen.setworldcoordinates(0, 0, self.width, self.height)
        self.update_cnt = 0
        self.one_px = float(self.turtle.screen.window_width()) / float(self.width) / 2

        self.beacons = []
        for y, line in enumerate(self.maze):
            for x, block in enumerate(line):
                if block:
                    nb_y = self.height - y - 1
                    if block == 2:
                        self.beacons.extend(((x + 0.5, nb_y + 0.5),))

    def draw(self):
        self.turtle.up()
        self.turtle.color("#00ffff")
        for x, y in self.beacons:
            self.turtle.setposition(x, y)
            self.turtle.dot(10)
        self.turtle.screen.update()

    def weight_to_color(self, weight):
        red = weight
        blue = 1 - weight
        return (red, 0, blue)

    def in_bounds(self, x, y):
        if x < 0 or y < 0 or x > self.width or y > self.height:
            return False
        return True

    def show_mean(self, x, y, confident=False):
        if confident:
            self.turtle.color("#00AA00")
        else:
            self.turtle.color("#cccccc")
        self.turtle.setposition(x, y)
        self.turtle.shape("circle")
        self.turtle.stamp()

    def show_particles(self, particles):
        self.update_cnt += 1
        if UPDATE_EVERY > 0 and self.update_cnt % UPDATE_EVERY != 1:
            return

        self.turtle.clearstamps()
        self.turtle.shape("tri")

        draw_cnt = 0
        px = {}
        for p in particles:
            draw_cnt += 1
            if DRAW_EVERY == 0 or draw_cnt % DRAW_EVERY == 1:
                # Keep track of which positions already have something
                # drawn to speed up display rendering
                scaled_x = int(p.x * self.one_px)
                scaled_y = int(p.y * self.one_px)
                scaled_xy = scaled_x * 10000 + scaled_y
                if scaled_xy not in px:
                    px[scaled_xy] = 1
                    self.turtle.setposition(p.x, p.y)
                    self.turtle.setheading(p.h)
                    self.turtle.color(self.weight_to_color(p.w))
                    self.turtle.stamp()

    def show_robot(self, robot):
        self.turtle.color("green")
        self.turtle.shape("turtle")
        self.turtle.setposition(robot.x, robot.y)
        self.turtle.setheading(robot.h)
        self.turtle.stamp()
        self.turtle.screen.update()

    def random_place(self):
        x = random.uniform(0, self.width)
        y = random.uniform(0, self.height)
        return x, y

    def random_free_place(self):
        while True:
            x, y = self.random_place()
            if self.in_bounds(x, y):
                return x, y

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def all_beacon_distance(self, x, y):
        d = []
        for c_x, c_y in self.beacons:
            d.append(self.distance(c_x, c_y, x, y))
        return d


def w_gauss(a, b):
    sqsum = 0
    for aa, bb in zip(a, b):
        sqsum += (aa - bb) * (aa - bb)
    g = math.exp(-1.0 * (sqsum / (0.1)))
    return g


def compute_mean_point(world, particles):

    m_x, m_y, m_count = 0, 0, 0
    for p in particles:
        m_count += p.w
        m_x += p.x * p.w
        m_y += p.y * p.w

    if m_count == 0:
        return -1, -1, False

    m_x /= m_count
    m_y /= m_count

    # Now compute how good that mean is -- check how many particles
    # actually are in the immediate vicinity
    m_count = 0
    for p in particles:
        if world.distance(p.x, p.y, m_x, m_y) < 1:
            m_count += 1

    return m_x, m_y, m_count > PARTICLE_COUNT * 0.95


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
        dx = math.sin(r) * speed
        dy = math.cos(r) * speed
        self.x += dx
        self.y += dy
        return True


class Robot():
    def __init__(self):
        self.x = 2
        self.y = 2
        self.h = random.uniform(0,360)
 
    def move(self, maze):
        while True:
            r = math.radians(self.h)
            dx = math.sin(r) * ROBOT_SPEED
            dy = math.cos(r) * ROBOT_SPEED
            if maze.in_bounds(self.x + dx, self.y+ dy):
                self.x += dx
                self.y += dy
                break
            self.h = random.uniform(0, 360)


def create_random(count, maze):
    return [
        Particle(*maze.random_free_place(), random.uniform(0, 360))
        for _ in range(0, count)
    ]


def duplicate(p):
    x = p.x + random.uniform(-0.1, 0.1)
    y = p.y + random.uniform(-0.1, 0.1)
    h = p.h + random.uniform(-0.1, 0.1)
    return Particle(x, y, h)


def main():

    world = World()
    world.draw()

    particles = create_random(PARTICLE_COUNT, world)
    robbie = Robot()

    loop_counter = 0

    while True:
        loop_counter += 1
        # time per iteration
        t0 = time.time_ns()

        distances = world.all_beacon_distance(robbie.x, robbie.y)

        reweight(world, particles, distances)

        # ---------- Try to find current best estimate for display ----------
        m_x, m_y, m_confident = compute_mean_point(world, particles)

        world.show_particles(particles)
        world.show_mean(m_x, m_y, m_confident)
        world.show_robot(robbie)

        particles = resample(world, particles)

        # ---------- Move things ----------
        old_heading = robbie.h
        robbie.move(world)
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


def reweight(world, particles, r_d):
    for p in particles:
        if world.in_bounds(p.x, p.y):
            p_d = world.all_beacon_distance(p.x, p.y)
            p.w = w_gauss(r_d, p_d)
        else:
            p.w = 0


def resample(world, particles):
    new_particles = []

    # Normalise weights
    nu = sum(p.w for p in particles)
    if nu:
        for p in particles:
            p.w = p.w / nu

        # create a weighted distribution, for fast picking
    dist = WeightedDistribution(particles)

    for _ in particles:
        p = dist.pick()
        if p is None:
            # No pick b/c all totally improbable
            new_particle = create_random(1, world)[0]
        else:
            # duplicate the chosen particle, with noise
            new_particle = duplicate(p)
        new_particles.append(new_particle)

    particles = new_particles
    return particles


if __name__ == "__main__":
    main()
