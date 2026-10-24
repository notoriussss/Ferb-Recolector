"""
rclpy helper shared by the vision nodes: the spin/shutdown boilerplate that
every node's `main()` used to repeat.
"""

from collections.abc import Callable

import rclpy
from rclpy.node import Node


def spin_node(node_factory: Callable[[], Node], args=None) -> None:
    """
    Init rclpy, spin the node until Ctrl+C and shut everything down.
    """
    rclpy.init(args=args)
    node = node_factory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
