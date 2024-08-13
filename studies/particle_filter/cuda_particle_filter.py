# pylint: disable=missing-class-docstring,missing-function-docstring,missing-module-docstring,too-few-public-methods,global-statement


import math
from turtle import Turtle

# import random
# import bisect
import time
import numpy as np
import cupy as cp  # type:ignore
from cupyx import jit  # type:ignore

print(f"CuPy version {cp.__version__}")
mempool = cp.get_default_memory_pool()
mempool.free_all_blocks()
print(f"mempool.used_bytes {mempool.used_bytes()}")


PARTICLE_COUNT = 20000
PARTICLES_TO_PLOT = 100
ROBOT_HAS_COMPASS = False
ROBOT_SPEED = 0.05
HEADING_RATE = 2.5
WIDTH = 5
HEIGHT = 5
PLOT_EVERY_N = 3


RESAMPLE = True


# particle (x,y,h,w), Nx4
particles_xyhw = cp.random.uniform(
    low=(0, 0, 0, 0), high=(WIDTH, HEIGHT, 0, 0), size=(PARTICLE_COUNT, 4)
)

# particles_h = cp.zeros(PARTICLE_COUNT)

# used temporarily by the resampler
new_particles_xyhw = cp.zeros_like(particles_xyhw)
indices = cp.zeros(PARTICLE_COUNT)

# particle weights (w), Nx1
# particles_w = cp.zeros(PARTICLE_COUNT)

# beacon (x,y), Mx2
beacon_xy = cp.array([[0.5, 0.5], [0.5, 4.5]])

# particle-to-beacon distances (d), NxM
distances = cp.zeros((particles_xyhw.shape[0], beacon_xy.shape[0]))

# TODO: use this
# this is 2d so i can use it as a list-of-one beacon
robot_xy = cp.array([[WIDTH / 4, HEIGHT / 2]])
# robot-to-beacon distances (d), 1xM
robot_distances = cp.zeros((1, beacon_xy.shape[0]))


def init(turtle):
    turtle.screen.mode("standard")
    turtle.resizemode("user")
    turtle.screen.tracer(50000, delay=0)
    turtle.speed(0)
    turtle.screen.title("particle filter demo")
    turtle.screen.setworldcoordinates(0, 0, WIDTH, HEIGHT)
    turtle.up()
    turtle.color("gray")
    for x, y in beacon_xy:
        turtle.setposition(x, y)
        turtle.dot(40)
    turtle.screen.update()


def weight_to_color(weight):
    if np.isnan(weight):
        weight = 0
    red = weight
    blue = 1 - weight
    return (red, 0, blue)


def show_robot(turtle, x, y, h):
    turtle.shape("classic")
    turtle.pen(pencolor="green", fillcolor="white", outline=5)
    turtle.shapesize(2)
    turtle.setposition(x, y)
    turtle.setheading(h)
    turtle.stamp()


def show_particles(turtle):
    turtle.shape("square")
    turtle.pen(outline=1)
    turtle.shapesize(1)
    for p in particles_xyhw[:: PARTICLE_COUNT / PARTICLES_TO_PLOT]:
        turtle.setposition(p[0], p[1])
        turtle.setheading(p[2])
        turtle.color(weight_to_color(p[3].item()), (1, 1, 1))
        turtle.stamp()


def show_mean(turtle, x, y) -> None:
    turtle.pen(pencolor="black", fillcolor="white", outline=5)
    turtle.shapesize(2)
    turtle.setposition(x, y)
    turtle.shape("triangle")
    turtle.stamp()


@jit.rawkernel()
def distance_kernel(
    particles: cp.ndarray,  # Nx4 (x, y, h, w)
    beacons: cp.ndarray,  # Mx2
    dist: cp.ndarray,  # NxM
    size: np.int32,  # this is N
) -> None:
    # pylint:disable=no-member
    tid = jit.blockIdx.x * jit.blockDim.x + jit.threadIdx.x
    ntid = jit.gridDim.x * jit.blockDim.x
    for i in range(tid, size, ntid):
        # i is the row in N
        for j in range(beacons.shape[0]):
            # j is the row in M
            p_x = particles[i, 0]
            p_y = particles[i, 1]
            b_x = beacons[j, 0]
            b_y = beacons[j, 1]
            dx = p_x - b_x
            dy = p_y - b_y
            d = cp.hypot(dx, dy)
            dist[i, j] = d


@jit.rawkernel()
def zero_out_of_bounds(particles: cp.ndarray, size: np.int32) -> None:
    # pylint:disable=no-member
    tid = jit.blockIdx.x * jit.blockDim.x + jit.threadIdx.x
    ntid = jit.gridDim.x * jit.blockDim.x
    for i in range(tid, size, ntid):
        p_x = particles[i, 0]
        p_y = particles[i, 1]
        if p_x < 0 or p_y < 0 or p_x > WIDTH or p_y > HEIGHT:
            particles[i, 3] = 0


@jit.rawkernel()
def weigh_similar(
    particle_distances: cp.ndarray,
    robot_dist: cp.ndarray,
    particles: cp.ndarray,
    size: np.int32,
) -> None:
    # pylint:disable=no-member
    tid = jit.blockIdx.x * jit.blockDim.x + jit.threadIdx.x
    ntid = jit.gridDim.x * jit.blockDim.x
    # loop over particles
    for i in range(tid, size, ntid):
        sqsum = 0.0
        # loop over beacons
        for j in range(particle_distances.shape[1]):
            p_d = particle_distances[i, j]
            r_d = robot_dist[0, j]
            diff_d = p_d - r_d
            sqsum += diff_d * diff_d
        # weights[i] = cp.exp(-1.0 * sqsum / 0.1)
        particles[i, 3] = cp.exp(-1.0 * sqsum / 0.1)


# populate the particle-to-beacon distance array.
def all_beacon_distance() -> None:
    # pylint:disable=no-value-for-parameter
    distance_kernel(
        (128,), (1024,), (particles_xyhw, beacon_xy, distances, PARTICLE_COUNT)
    )


# populate the robot-to-beacon distance array
def robot_beacon_distance() -> None:
    # pylint:disable=no-value-for-parameter
    distance_kernel(
        (128,), (1024,), (robot_xy, beacon_xy, robot_distances, PARTICLE_COUNT)
    )


def normalize(particles) -> None:
    wsum = cp.sum(particles[:, 3])
    particles[:, 3] /= wsum


def reweight() -> None:
    # pylint:disable=no-value-for-parameter,too-many-function-args
    # populate distances
    all_beacon_distance()
    # populate robot_distances
    robot_beacon_distance()

    weigh_similar(
        (128,), (1024,), (distances, robot_distances, particles_xyhw, PARTICLE_COUNT)
    )
    zero_out_of_bounds((128,), (1024,), (particles_xyhw, PARTICLE_COUNT))
    normalize(particles_xyhw)


def compute_mean() -> tuple[float, float]:
    xy = cp.average(particles_xyhw, axis=0, weights=particles_xyhw[:, 3])
    return xy[0], xy[1]


def resample() -> None:
    global particles_xyhw

    selector = cp.random.uniform(size=PARTICLE_COUNT)
    cumweights = cp.cumsum(particles_xyhw[:, 3])
    keys = cp.searchsorted(cumweights, selector)
    particles_xyhw = particles_xyhw[keys]
    particles_xyhw[:, 0] += cp.random.uniform(-0.1, 0.1, size=PARTICLE_COUNT)
    particles_xyhw[:, 1] += cp.random.uniform(-0.1, 0.1, size=PARTICLE_COUNT)


def main():
    turtle = Turtle()
    init(turtle)
    print(f"mempool.used_bytes {mempool.used_bytes()}")

    robot_xy[0, 0] = WIDTH / 4
    robot_xy[0, 1] = HEIGHT / 2
    robot_h = 270

    loop_counter = 0

    while True:
        loop_counter += 1
        t0 = time.time_ns()

        reweight()
        # time.sleep(0.1)
        x, y = compute_mean()

        if loop_counter % PLOT_EVERY_N == 0:
            turtle.clearstamps()
            show_particles(turtle)
            show_mean(turtle, x, y)
            show_robot(turtle, robot_xy[0, 0], robot_xy[0, 1], robot_h)
            turtle.screen.update()

        if RESAMPLE:
            resample()

        old_heading = robot_h

        robot_h += HEADING_RATE
        r = math.radians(robot_h)
        robot_xy[0, 0] += math.cos(r) * ROBOT_SPEED
        robot_xy[0, 1] += math.sin(r) * ROBOT_SPEED

        d_h = robot_h - old_heading

        if RESAMPLE:

            # just mirror the robot heading for now

            particles_xyhw[:, 2] = cp.ones(PARTICLE_COUNT) * robot_h
            r = cp.deg2rad(particles_xyhw[:, 2])
            dx = cp.cos(r) * ROBOT_SPEED
            dy = cp.sin(r) * ROBOT_SPEED
            d = cp.hstack((dx, dy))
            particles_xyhw[:, 0] += dx
            particles_xyhw[:, 1] += dy

        t1 = time.time_ns()
        duration = t1 - t0
        if loop_counter % 2 == 0:
            print(f"duration (us): {duration//1000}")
            print(f"duration per particle (us): {duration//(1000*PARTICLE_COUNT)}")


if __name__ == "__main__":
    main()
