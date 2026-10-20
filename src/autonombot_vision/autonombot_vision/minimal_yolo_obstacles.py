#!/usr/bin/env python3
"""Compatibility console entry for `minimal_yolo_obstacles`.

Delegates to `vision_obstacle_detector` (YOLO + /vision_obstacles). Prefer
`ros2 run autonombot_vision vision_obstacle_detector` for new setups.
"""


def main(args=None):
    from autonombot_vision.vision_obstacle_detector import main as vision_main

    return vision_main(args)


if __name__ == '__main__':
    main()
