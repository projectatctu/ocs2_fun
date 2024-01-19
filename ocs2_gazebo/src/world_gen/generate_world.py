#!/usr/bin/env python3

import os
import rospkg

import argparse

from map_generator import MapGenerator
from ply_generator import PlyGenerator
from shapes import Box

def parse_args():
    parser = argparse.ArgumentParser(description="Generate a world file")
    parser.add_argument(
        "--world_file", type=str, default="custom", help="World file name"
    )
    parser.add_argument(
        "--type", type=str, default="stairs", choices=['box', 'stairs', 'random_blocks'], help="Type of world to generate"
    )

    return parser.parse_args()

def generate_stairs():
    STEPS = 10
    STEP_HEIGHT = 0.13
    STEP_WIDTH = 5.7
    STEP_DEPTH = 0.30

    # Generate stairs
    m = MapGenerator()
    floor = Box("floor", *[0, 0, -0.1], *[10, 10, 0.2], visualize=False)
    m.add_shape(floor)
    for i in range(STEPS):
        add = 0.0
        if i == (STEPS - 1):
            add = 1.0
        p = [i * STEP_DEPTH + add / 2, 0, i * STEP_HEIGHT + STEP_HEIGHT / 2]
        s = [(STEP_DEPTH + add), STEP_WIDTH, STEP_HEIGHT]
        b = Box("s{}".format(i), *p, *s)
        m.add_shape(b)
    return m


def generate_box():
    m = MapGenerator()
    floor = Box("floor", *[0, 0, -0.1], *[5, 5, 0.2], visualize=False)
    box = Box("box", *[1, 0, 0.20/2], *[3.0, 3.0, 0.20])
    m.add_shape(floor)
    m.add_shape(box)
    return m


def generate_random_blocks():
    BLOCK_HEIGHT = 0.4
    BLOCK_DEPTH = 0.5
    BLOCK_WIDTH = 0.5
    BLOCKS = 60

    import random

    m = MapGenerator()

    floor = Box("floor", *[0, 0, -0.1], *[7, 7, 0.2], visualize=False)
    
    for i in range(BLOCKS):
        x = random.uniform(0, 3)
        y = random.uniform(-3, 3)
        z = random.uniform(-BLOCK_HEIGHT, 0.0)
        p = [x, y, z]
        s = [BLOCK_DEPTH, BLOCK_WIDTH, BLOCK_HEIGHT]
        b = Box("b{}".format(i), *p, *s)
        m.add_shape(b)

    m.add_shape(floor)
    return m


def main():

    args = parse_args()

    if args.type == 'stairs':
        m = generate_stairs()
    elif args.type == 'box':
        m = generate_box()
    elif args.type == 'random_blocks':
        m = generate_random_blocks()
    else:
        raise ValueError("Unknown type {}".format(args.type))

    # Store world
    path = rospkg.RosPack().get_path("ocs2_gazebo")
    world_folder = os.path.join(path, "launch", "worlds", args.world_file)
    
    if not os.path.exists(world_folder):
        os.makedirs(world_folder)
    
    world_file = os.path.join(world_folder, args.world_file + ".world")

    m.generate_map(world_file)

    ply_generator = PlyGenerator()
    ply_file = os.path.join(world_folder, args.world_file + ".ply")
    ply_generator.generate_ply(ply_file, m.shapes)

    print("World generated!")


if __name__ == "__main__":
    main()
