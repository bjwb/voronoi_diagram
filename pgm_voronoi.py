#!/usr/bin/env python3
"""Build a free-space Voronoi roadmap from a ROS 2 PGM map (no ROS required).

Examples:
    python3 pgm_voronoi.py map/map2.pgm
    python3 pgm_voronoi.py map/map2.yaml --robot-radius 0.22 --safety-margin 0.05

Image coordinates (u, v) have integer pixel centers and v pointing down.
Exported (x, y) coordinates point right/up and include the YAML origin/yaw.
Distances are meters when resolution is available, otherwise pixels.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy.ndimage import binary_dilation
from scipy.spatial import QhullError, Voronoi


def finite_number(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("Numbers must be finite.")
    return number


def load_map(path, resolution=None, origin=None, free_thresh=None, negate=None):
    """Read PGM or map YAML; a matching sidecar YAML is used automatically."""
    path = Path(path).expanduser().resolve()
    yaml_path = path if path.suffix.lower() in (".yaml", ".yml") else next(
        (p for p in (path.with_suffix(".yaml"), path.with_suffix(".yml"))
         if p.is_file()), None)
    config = {}
    if yaml_path is not None:
        with yaml_path.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        if not isinstance(config, dict):
            raise ValueError("Map YAML must contain a mapping.")
        for key in ("image", "resolution", "origin"):
            if key not in config:
                raise ValueError(f"Map YAML is missing '{key}'.")
        if config.get("mode", "trinary") not in ("trinary", "scale"):
            raise ValueError("Only trinary/scale map YAML is supported; raw mode is not.")
        yaml_image = (yaml_path.parent / str(config["image"])).resolve()
        if path == yaml_path:
            path = yaml_image
        elif path != yaml_image:
            raise ValueError(f"Sidecar YAML points to a different image: {yaml_image}")

    resolution = resolution if resolution is not None else config.get("resolution")
    if resolution is not None:
        resolution = finite_number(resolution)
        if resolution <= 0:
            raise ValueError("Resolution must be positive (meters/pixel).")
    origin = origin if origin is not None else config.get("origin", [0, 0, 0])
    if not isinstance(origin, (list, tuple)) or len(origin) != 3:
        raise ValueError("Origin must be [x, y, yaw].")
    origin = [finite_number(x) for x in origin]
    if resolution is None and any(origin):
        raise ValueError("A nonzero origin requires --resolution or a map YAML.")
    threshold = finite_number(free_thresh if free_thresh is not None
                              else config.get("free_thresh", 0.196))
    occupied_thresh = finite_number(config.get("occupied_thresh", 0.65))
    if not 0 <= threshold < occupied_thresh <= 1:
        raise ValueError("Require 0 <= free_thresh < occupied_thresh <= 1.")
    negate = negate if negate is not None else config.get("negate", 0)
    if negate not in (0, 1, False, True):
        raise ValueError("Negate must be 0 or 1.")

    with Image.open(path) as image:
        if image.mode != "L":
            raise ValueError("Use an 8-bit grayscale PGM/image (pixel values 0..255).")
        gray = np.asarray(image).copy()
    probability = gray.astype(np.float64) / 255.0
    if not negate:
        probability = 1.0 - probability
    # Strict comparison follows Humble map_server; ambiguous gray stays blocked.
    free = probability < threshold
    metadata = {
        "image": str(path), "yaml": str(yaml_path) if yaml_path else None,
        "width": gray.shape[1], "height": gray.shape[0],
        "resolution": resolution, "origin": origin,
        "units": "m" if resolution is not None else "px",
        "free_thresh": threshold, "occupied_thresh": occupied_thresh,
        "negate": int(negate), "unknown_is_blocked": True,
        "pixel_coordinates": "u right, v down; integer coordinates are pixel centers",
        "xy_coordinates": "origin + yaw rotation of ((u+0.5), (height-v-0.5)) * scale",
    }
    return gray, free, metadata


def voronoi_segments(free, clearance_px=0.0, min_site_angle=45.0):
    """Return vertices and (source, target, length_px, clearance_lower_bound_px).

    Sites are centers of blocked cells bordering free space, including the map
    exterior. Neighboring site pairs (<2 pixels apart) are suppressed to remove
    the comb-like bisectors between adjacent samples of the same wall. A minimum
    angle between generators at a leaf branch's attachment also suppresses
    branches caused by the staircase sampling of a slanted wall. Entire leaf
    chains are removed so this filter cannot fragment a connected roadmap.

    Along a Voronoi ridge its generators are the nearest sites. The minimum
    generator-to-segment distance minus sqrt(0.5) bounds the distance to the
    entire blocked cell squares, not just their centers. With free endpoints
    and a positive bound, the whole segment stays inside free space. This is
    conservative: narrow passages can be removed by up to ~0.71 px of padding.
    """
    free = np.asarray(free, dtype=bool)
    if free.ndim != 2 or not free.size:
        raise ValueError("Free mask must be a nonempty 2-D array.")
    if not math.isfinite(clearance_px) or clearance_px < 0:
        raise ValueError("Clearance must be finite and nonnegative.")
    if not math.isfinite(min_site_angle) or not 0 <= min_site_angle < 180:
        raise ValueError("Minimum site angle must be in [0, 180) degrees.")
    if not free.any():
        return np.empty((0, 2)), []
    padded = np.pad(free, 1, constant_values=False)
    boundary = ~padded & binary_dilation(padded, structure=np.ones((3, 3), bool))
    sites = (np.argwhere(boundary)[:, ::-1] - 1).astype(np.float64)
    diagram = Voronoi(sites)

    finite = [(i, ends) for i, ends in enumerate(diagram.ridge_vertices)
              if len(ends) == 2 and -1 not in ends]
    if not finite:
        return diagram.vertices, []
    ridge_ids = np.array([i for i, _ in finite])
    ends = np.array([pair for _, pair in finite])
    generators = sites[diagram.ridge_points[ridge_ids]]
    start, finish = diagram.vertices[ends[:, 0]], diagram.vertices[ends[:, 1]]
    delta = finish - start
    length_sq = np.sum(delta * delta, axis=1)
    separation_sq = np.sum((generators[:, 0] - generators[:, 1]) ** 2, axis=1)
    keep = (length_sq > 1e-16) & (separation_sq >= 4.0 - 1e-10)
    midpoint_radius_sq = np.sum(((start + finish) / 2 - generators[:, 0]) ** 2, axis=1)
    # For an isosceles triangle: sin(angle/2)^2 = separation^2 / (4*radius^2).
    weak_angle = separation_sq < (4 * midpoint_radius_sq
                                  * math.sin(math.radians(min_site_angle) / 2) ** 2)
    for points in (start, finish):
        # Test bounds before integer conversion, including numerical outliers.
        in_bounds = (np.isfinite(points).all(axis=1)
                     & (points[:, 0] >= -0.5) & (points[:, 0] < free.shape[1] - 0.5)
                     & (points[:, 1] >= -0.5) & (points[:, 1] < free.shape[0] - 0.5))
        idx = np.flatnonzero(in_bounds)
        pixels = np.floor(points[idx] + 0.5).astype(int)
        valid = np.zeros(len(points), dtype=bool)
        valid[idx] = free[pixels[:, 1], pixels[:, 0]]
        keep &= valid
    projection = np.sum((generators[:, 0] - start) * delta, axis=1)
    fraction = np.clip(projection / np.maximum(length_sq, 1e-16), 0, 1)
    closest = start + fraction[:, None] * delta
    clearance = np.linalg.norm(closest - generators[:, 0], axis=1) - math.sqrt(0.5)
    keep &= clearance > clearance_px + 1e-9
    segments = [(int(a), int(b), float(math.sqrt(length_sq[i])), float(clearance[i]))
                for i, (a, b) in enumerate(ends) if keep[i]]
    weak_pairs = {tuple(pair) for pair in ends[keep & weak_angle]}
    return diagram.vertices, prune_short_branches(segments, 0, weak_pairs)


def prune_short_branches(segments, min_length_px, weak_pairs=None):
    """Remove short leaf chains or wall-sampling branches; retain closed loops.

    weak_pairs optionally identifies ridges with too small a generator angle.
    Only leaf chains whose junction attachment is weak are removed, rather than
    deleting individual ridges and leaving disconnected pieces near the walls.
    """
    if min_length_px <= 0 and not weak_pairs:
        return segments
    adjacency = {}
    for edge_id, (a, b, _, _) in enumerate(segments):
        adjacency.setdefault(a, set()).add(edge_id)
        adjacency.setdefault(b, set()).add(edge_id)
    active = set(range(len(segments)))
    while True:
        remove = set()
        for leaf in [n for n, edges in adjacency.items() if len(edges) == 1]:
            node, previous, chain, length = leaf, None, [], 0.0
            while True:
                choices = adjacency[node] - ({previous} if previous is not None else set())
                if not choices:
                    break
                edge_id = min(choices)
                a, b, distance, _ = segments[edge_id]
                chain.append(edge_id)
                length += distance
                node = b if a == node else a
                previous = edge_id
                if len(adjacency[node]) != 2 or (not weak_pairs and length >= min_length_px):
                    break
            weak_attachment = (weak_pairs and chain and len(adjacency[node]) > 2
                               and segments[chain[-1]][:2] in weak_pairs)
            if length < min_length_px or weak_attachment:
                remove.update(chain)
        if not remove:
            break
        for edge_id in remove:
            a, b, _, _ = segments[edge_id]
            adjacency[a].discard(edge_id)
            adjacency[b].discard(edge_id)
        active.difference_update(remove)
    return [segments[i] for i in sorted(active)]


def pixel_to_xy(points, metadata):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    scale = metadata["resolution"] or 1.0
    local = np.column_stack((points[:, 0] + 0.5,
                             metadata["height"] - points[:, 1] - 0.5)) * scale
    x, y, yaw = metadata["origin"]
    c, s = math.cos(yaw), math.sin(yaw)
    return local @ np.array([[c, s], [-s, c]]) + [x, y]


def export_graph(vertices, segments, metadata, parameters):
    """Keep every ridge vertex so each exported edge is a checked straight line."""
    used = sorted({n for a, b, _, _ in segments for n in (a, b)})
    node_ids = {old: new for new, old in enumerate(used)}
    points = vertices[used]
    xy = pixel_to_xy(points, metadata)
    adjacency = [set() for _ in used]
    scale = metadata["resolution"] or 1.0
    edges = []
    for a, b, length, clearance in segments:
        a, b = node_ids[a], node_ids[b]
        adjacency[a].add(b)
        adjacency[b].add(a)
        edges.append({"source": a, "target": b, "length": length * scale,
                      "min_clearance": clearance * scale})
    components, component_id = {}, 0
    for node in range(len(used)):
        if node in components:
            continue
        components[node] = component_id
        stack = [node]
        while stack:
            for neighbor in adjacency[stack.pop()]:
                if neighbor not in components:
                    components[neighbor] = component_id
                    stack.append(neighbor)
        component_id += 1
    nodes = [{"id": i, "pixel": p.tolist(), "position": xy[i].tolist(),
              "degree": len(adjacency[i]), "component": components[i]}
             for i, p in enumerate(points)]
    return {"format_version": 1, "directed": False, "map": metadata,
            "parameters": parameters, "component_count": component_id,
            "nodes": nodes, "edges": edges}


def save_images(gray, free, graph, prefix, scale=2):
    """Save a visual overlay and a map-sized raster (255 = Voronoi pixels)."""
    resampling = getattr(Image, "Resampling", Image)
    overlay = Image.fromarray(gray).convert("RGB").resize(
        (gray.shape[1] * scale, gray.shape[0] * scale), resampling.NEAREST)
    raster = Image.new("L", (gray.shape[1], gray.shape[0]))
    draw, raster_draw = ImageDraw.Draw(overlay), ImageDraw.Draw(raster)
    for edge in graph["edges"]:
        points = [graph["nodes"][edge[key]]["pixel"] for key in ("source", "target")]
        draw.line([tuple((v + 0.5) * scale - 0.5 for v in p) for p in points],
                  fill=(230, 30, 60), width=max(1, scale // 2))
        raster_draw.line([tuple(int(math.floor(v + 0.5)) for v in p) for p in points],
                         fill=255, width=1)
    # A raster is for display; clipping prevents raster rounding into blocked cells.
    mask = np.asarray(raster).copy()
    mask[~free] = 0
    overlay.save(str(prefix) + "_overlay.png")
    Image.fromarray(mask).save(str(prefix) + "_skeleton.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("map", type=Path, help="8-bit PGM or ROS map YAML")
    parser.add_argument("--output-dir", type=Path, default=Path("output"))
    parser.add_argument("--resolution", type=finite_number, help="meters/pixel; overrides YAML")
    parser.add_argument("--origin", type=finite_number, nargs=3, metavar=("X", "Y", "YAW"))
    parser.add_argument("--free-thresh", type=finite_number, help="occupancy threshold; default 0.196")
    parser.add_argument("--negate", type=int, choices=(0, 1), help="override YAML pixel interpretation")
    parser.add_argument("--robot-radius", type=finite_number, default=0.0,
                        help="circular footprint radius in m (with resolution), otherwise px")
    parser.add_argument("--safety-margin", type=finite_number, default=0.0,
                        help="additional clearance, same units as robot radius")
    parser.add_argument("--min-branch-length", type=finite_number, default=0.0,
                        help="prune terminal branches shorter than this; m or px, default off")
    parser.add_argument("--min-site-angle", type=finite_number, default=45.0,
                        help="prune leaf branches with small attachment site angle; degrees, default 45")
    parser.add_argument("--image-scale", type=int, default=2, help="overlay magnification (1..8)")
    args = parser.parse_args()
    if min(args.robot_radius, args.safety_margin, args.min_branch_length) < 0:
        parser.error("Radius, margin, and branch length must be nonnegative.")
    if not 1 <= args.image_scale <= 8:
        parser.error("--image-scale must be between 1 and 8.")
    try:
        gray, free, metadata = load_map(args.map, args.resolution, args.origin,
                                         args.free_thresh, args.negate)
        scale = metadata["resolution"] or 1.0
        required = args.robot_radius + args.safety_margin
        vertices, segments = voronoi_segments(free, required / scale, args.min_site_angle)
        segments = prune_short_branches(segments, args.min_branch_length / scale)
        parameters = {"robot_radius": args.robot_radius, "safety_margin": args.safety_margin,
                      "min_branch_length": args.min_branch_length,
                      "min_site_angle_degrees": args.min_site_angle,
                      "clearance_method": "nearest boundary-cell center minus cell circumradius"}
        graph = export_graph(vertices, segments, metadata, parameters)
        args.output_dir.mkdir(parents=True, exist_ok=True)
        prefix = args.output_dir / (Path(metadata["image"]).stem + "_voronoi")
        save_images(gray, free, graph, prefix, args.image_scale)
        graph_path = Path(str(prefix) + "_graph.json")
        graph_path.write_text(json.dumps(graph, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except (OSError, ValueError, QhullError, yaml.YAMLError) as error:
        parser.exit(1, f"Error: {error}\n")
    print(f"Free: {free.sum():,}/{free.size:,} pixels | units: {metadata['units']}")
    print(f"Nodes: {len(graph['nodes']):,} | edges: {len(graph['edges']):,} | "
          f"components: {graph['component_count']}")
    for suffix in ("_overlay.png", "_skeleton.png", "_graph.json"):
        print(Path(str(prefix) + suffix).resolve())
    if not graph["edges"]:
        print("No roadmap edges remain. Check the free threshold and clearance settings.")
    if metadata["resolution"] is None:
        print("No resolution supplied: all distances are pixels. Use YAML or --resolution for meters.")


if __name__ == "__main__":
    main()
