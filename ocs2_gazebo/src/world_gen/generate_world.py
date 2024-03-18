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
    
    # World specific arguments
    
    # Stairs
    parser.add_argument(
        "--steps", type=int, default=10, help="Number of steps"
    )
    parser.add_argument(
        "--step_height", type=float, default=0.13, help="Height of each step"
    )
    parser.add_argument(
        "--step_width", type=float, default=5.7, help="Width of each step"
    )
    parser.add_argument(
        "--step_depth", type=float, default=0.30, help="Depth of each step"
    )
    
    # Box
    parser.add_argument(
        "--box_size", type=float, default=0.35, help="Width of box"
    )
    
    # Random blocks
    parser.add_argument(
        "--blocks", type=int, default=60, help="Number of blocks"
    )
    parser.add_argument(
        "--block_height", type=float, default=0.4, help="Height of each block"
    )
    parser.add_argument(
        "--block_width", type=float, default=0.5, help="Width of each block"
    )
    parser.add_argument(
        "--block_depth", type=float, default=0.5, help="Depth of each block"
    )

    return parser.parse_args()

def generate_stairs(params):
    STEPS = params.steps
    STEP_HEIGHT = params.step_height
    STEP_WIDTH = params.step_width
    STEP_DEPTH = params.step_depth

    # Generate stairs
    m = MapGenerator()
    floor = Box("floor", *[0, 0, -0.1], *[25, 25, 0.2], visualize=False)
    m.add_shape(floor)
    for i in range(STEPS):
        add = 0.0
        if i == (STEPS - 1):
            add = 10.0
        p = [i * STEP_DEPTH + add / 2, 0, i * STEP_HEIGHT + STEP_HEIGHT / 2]
        s = [(STEP_DEPTH + add), STEP_WIDTH, STEP_HEIGHT]
        b = Box("s{}".format(i), *p, *s)
        m.add_shape(b)
    return m


def generate_box(params):
    BOX_SIZE = params.box_size
    m = MapGenerator()
    floor = Box("floor", *[0.6, 0, -0.1], *[25, 25, 0.2], visualize=False)
    box = Box("box", *[0, 0, BOX_SIZE/2], *[1.2, 3, BOX_SIZE])
    m.add_shape(floor)
    m.add_shape(box)
    return m


def generate_random_blocks(params):
    BLOCK_HEIGHT = params.block_height
    BLOCK_DEPTH = params.block_depth
    BLOCK_WIDTH = params.block_width
    BLOCKS = params.blocks

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
        m = generate_stairs(args)
    elif args.type == 'box':
        m = generate_box(args)
    elif args.type == 'random_blocks':
        m = generate_random_blocks(args)
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
