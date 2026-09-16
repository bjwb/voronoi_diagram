#!/usr/bin/env python3
"""Publish pgm_voronoi.py graph JSON as an RViz LINE_LIST Marker.

source /opt/ros/humble/setup.bash
python3 voronoi_rviz.py output/map_voronoi_graph.json \
    --resolution 0.05 --origin -12.9 -11.2 0

Set RViz Fixed Frame to map, add Marker, and select /voronoi.
Keep this process running. It publishes once per second and uses transient-local
QoS so an RViz instance opened later can receive the existing marker.
"""

import argparse
import json
import math
import sys
from pathlib import Path


def finite_number(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("Numbers must be finite.")
    return number


def graph_line_points(graph, resolution=None, origin=None):
    """Convert edge endpoints to meters, including image Y flip and origin yaw.

    Always start with the stored pixel coordinates to avoid scaling or rotating
    already converted positions twice. Overrides affect display only.
    """
    metadata = graph["map"]
    resolution = resolution if resolution is not None else metadata.get("resolution")
    if resolution is None:
        raise ValueError("This JSON uses pixels. Supply --resolution in meters/pixel "
                         "and --origin X Y YAW to match the displayed map.")
    resolution = finite_number(resolution)
    if resolution <= 0:
        raise ValueError("Resolution must be positive.")
    origin = origin if origin is not None else metadata.get("origin", [0, 0, 0])
    if len(origin) != 3:
        raise ValueError("Origin must contain X, Y, YAW.")
    ox, oy, yaw = (finite_number(value) for value in origin)
    height = finite_number(metadata["height"])
    if height <= 0:
        raise ValueError("Image height must be positive.")
    c, s = math.cos(yaw), math.sin(yaw)
    positions = {}
    for node in graph["nodes"]:
        u, v = (finite_number(value) for value in node["pixel"])
        x, y = (u + 0.5) * resolution, (height - v - 0.5) * resolution
        if node["id"] in positions:
            raise ValueError("Duplicate node ID in graph.")
        positions[node["id"]] = (ox + c * x - s * y, oy + s * x + c * y)
    return [positions[edge[key]] for edge in graph["edges"] for key in ("source", "target")]


def make_marker(points, frame_id="map", line_width=0.03, z=0.03):
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker

    marker = Marker()
    marker.header.frame_id = frame_id
    # Stamp zero requests the latest TF and also works with simulated time.
    marker.ns = "voronoi"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD if points else Marker.DELETE
    marker.pose.orientation.w = 1.0
    marker.scale.x = float(line_width)
    marker.color.r = 1.0
    marker.color.g = 0.1
    marker.color.b = 0.1
    marker.color.a = 1.0
    marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y in points]
    # Zero lifetime keeps this marker until replaced/deleted or RViz is reset.
    return marker


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("graph", type=Path, help="JSON exported by pgm_voronoi.py")
    parser.add_argument("--resolution", type=finite_number, help="meters/pixel; override JSON")
    parser.add_argument("--origin", type=finite_number, nargs=3, metavar=("X", "Y", "YAW"),
                        help="map origin in meters/radians; override JSON")
    parser.add_argument("--frame-id", default="map", help="coordinate frame of the map")
    parser.add_argument("--topic", default="/voronoi")
    parser.add_argument("--line-width", type=finite_number, default=0.03, help="meters")
    parser.add_argument("--z", type=finite_number, default=0.03,
                        help="height above the map in meters, avoids overlapping surfaces")
    arguments = sys.argv[1:]
    ros_index = arguments.index("--ros-args") if "--ros-args" in arguments else len(arguments)
    args = parser.parse_args(arguments[:ros_index])
    if args.line_width <= 0:
        parser.error("--line-width must be positive.")
    if not args.frame_id.strip():
        parser.error("--frame-id must not be empty.")
    try:
        graph = json.loads(args.graph.read_text(encoding="utf-8"))
        points = graph_line_points(graph, args.resolution, args.origin)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(1, f"Invalid graph/settings: {error}\n")
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from visualization_msgs.msg import Marker
    except ImportError as error:
        parser.exit(1, f"ROS 2 Python packages are unavailable: {error}\n"
                    "Source your ROS 2 setup.bash and use its system Python.\n")

    rclpy.init(args=arguments[ros_index:])
    node = None
    try:
        node = rclpy.create_node("voronoi_rviz")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(Marker, args.topic, qos)
        marker = make_marker(points, args.frame_id, args.line_width, args.z)
        publisher.publish(marker)
        node.create_timer(100.0, lambda: publisher.publish(marker))
        node.get_logger().info(
            f"Publishing {len(points) // 2} Voronoi edges to {publisher.topic_name}; "
            f"frame={args.frame_id}, width={args.line_width} m. Ctrl+C to stop.")
        if not points:
            node.get_logger().warning("The graph is empty; no lines will be visible.")
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
