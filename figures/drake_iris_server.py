#!/usr/bin/env python3
"""Serve the IRIS map designer and compute regions with Drake's standard IRIS.

The frontend is intentionally lightweight.  All certified region generation is
performed here with ``pydrake.geometry.optimization.Iris``.
"""

from __future__ import annotations

import json
import math
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import numpy as np

try:
    from shapely.geometry import Polygon as ShapelyPolygon  # type: ignore
    from shapely.ops import triangulate as shapely_triangulate  # type: ignore
except Exception:  # pragma: no cover - optional runtime dependency
    ShapelyPolygon = None
    shapely_triangulate = None

ROOT = Path(__file__).resolve().parents[1]
DRAKE_VENDOR = ROOT / ".python_drake"
HTML_PATH = ROOT / "figures" / "iris_map_designer.html"

if DRAKE_VENDOR.exists():
    sys.path.insert(0, str(DRAKE_VENDOR))

try:
    from pydrake.geometry.optimization import (  # type: ignore
        HPolyhedron,
        Iris,
        IrisOptions,
        VPolytope,
    )
except Exception as exc:  # pragma: no cover - surfaced through /api/health
    HPolyhedron = None
    Iris = None
    IrisOptions = None
    VPolytope = None
    DRAKE_IMPORT_ERROR = repr(exc)
else:
    DRAKE_IMPORT_ERROR = None


def _json_response(handler: BaseHTTPRequestHandler, status: int, payload: dict[str, Any]) -> None:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.send_header("Access-Control-Allow-Origin", "*")
    handler.send_header("Access-Control-Allow-Headers", "Content-Type")
    handler.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
    handler.end_headers()
    handler.wfile.write(body)


def _read_json(handler: BaseHTTPRequestHandler) -> dict[str, Any]:
    length = int(handler.headers.get("Content-Length", "0") or "0")
    raw = handler.rfile.read(length)
    if not raw:
        return {}
    return json.loads(raw.decode("utf-8"))


def _ordered_polygon(vertices: np.ndarray) -> list[list[float]]:
    pts = np.asarray(vertices, dtype=float)
    if pts.shape[0] == 2:
        pts = pts.T
    center = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    ordered = pts[np.argsort(angles)]
    return np.round(ordered, 6).tolist()


def _rect_to_points(obs: dict[str, Any]) -> list[dict[str, float]]:
    x = float(obs["x"])
    y = float(obs["y"])
    w = float(obs["w"])
    h = float(obs["h"])
    x0, x1 = sorted([x, x + w])
    y0, y1 = sorted([y, y + h])
    return [
        {"x": x0, "y": y0},
        {"x": x1, "y": y0},
        {"x": x1, "y": y1},
        {"x": x0, "y": y1},
    ]


def _obstacle_points(obs: dict[str, Any]) -> list[dict[str, float]]:
    if obs.get("type") == "rect":
        return _rect_to_points(obs)
    points = obs.get("points")
    if not isinstance(points, list) or len(points) < 3:
        raise ValueError("polygon obstacle must contain at least three points")
    return [{"x": float(p["x"]), "y": float(p["y"])} for p in points]


def _vpolytope_from_points(points: list[dict[str, float]]) -> Any:
    arr = np.array([[p["x"] for p in points], [p["y"] for p in points]], dtype=float)
    return VPolytope(np.asfortranarray(arr))


def _convex_obstacle_polytopes(points: list[dict[str, float]], margin: float) -> list[Any]:
    """Return convex obstacle pieces for Drake IRIS.

    Drake's ``Iris`` API expects convex obstacles. The designer, however, lets
    us draw a general polygon. We therefore inflate the polygon first and then
    triangulate it into convex pieces. This prevents a concave drawn obstacle
    from being passed as a single VPolytope and also makes the visual safety
    margin explicit.
    """

    if ShapelyPolygon is None or shapely_triangulate is None:
        return [_vpolytope_from_points(points)]

    poly = ShapelyPolygon([(p["x"], p["y"]) for p in points])
    if not poly.is_valid:
        poly = poly.buffer(0)
    if margin > 0.0:
        poly = poly.buffer(margin, join_style=2)

    pieces = []
    polygons = list(poly.geoms) if hasattr(poly, "geoms") else [poly]
    for polygon in polygons:
        for tri in shapely_triangulate(polygon):
            if tri.area < 1e-6:
                continue
            # ``triangulate`` is not a constrained triangulation: for concave
            # polygons, using only triangles whose representative point is
            # inside the polygon can under-approximate the obstacle and let
            # IRIS grow through a missing boundary sliver. Keep every triangle
            # with nonzero intersection instead; this over-approximates the
            # inflated obstacle, which is conservative for collision safety.
            if polygon.intersection(tri).area < 1e-7:
                continue
            coords = list(tri.exterior.coords)[:-1]
            tri_points = [{"x": float(x), "y": float(y)} for x, y in coords]
            pieces.append(_vpolytope_from_points(tri_points))
    return pieces


def _ellipse_payload(ellipsoid: Any) -> dict[str, Any]:
    # Drake represents E = {x | ||A (x - center)||_2 <= 1}.
    # The semi-axis lengths are the reciprocals of A's singular values.
    a_matrix = np.asarray(ellipsoid.A(), dtype=float)
    u, singular_values, _ = np.linalg.svd(a_matrix)
    radii = 1.0 / singular_values
    order = np.argsort(radii)[::-1]
    radii = radii[order]
    axes = u[:, order]
    theta = math.atan2(float(axes[1, 0]), float(axes[0, 0]))
    return {
        "center": np.round(ellipsoid.center(), 6).tolist(),
        "radii": np.round(radii, 6).tolist(),
        "theta": theta,
        "A": np.round(a_matrix, 9).tolist(),
        "volume": float(ellipsoid.CalcVolume()),
    }


def compute_drake_iris(payload: dict[str, Any]) -> dict[str, Any]:
    if DRAKE_IMPORT_ERROR:
        raise RuntimeError(f"pydrake import failed: {DRAKE_IMPORT_ERROR}")

    width = float(payload.get("width", 980))
    height = float(payload.get("height", 650))
    seed_payload = payload.get("seed")
    if not isinstance(seed_payload, dict):
        raise ValueError("seed is required")
    seed = np.array([float(seed_payload["x"]), float(seed_payload["y"])], dtype=float)
    if not (0.0 <= seed[0] <= width and 0.0 <= seed[1] <= height):
        raise ValueError("seed must lie inside the workspace domain")

    domain = HPolyhedron.MakeBox(np.array([0.0, 0.0]), np.array([width, height]))
    margin = float(payload.get("configuration_space_margin", 0.0) or 0.0)
    obstacles = []
    for obs in payload.get("obstacles", []):
        points = _obstacle_points(obs)
        obstacles.extend(_convex_obstacle_polytopes(points, margin))

    options = IrisOptions()
    options.require_sample_point_is_contained = True
    options.iteration_limit = int(payload.get("iteration_limit", 60))
    options.termination_threshold = float(payload.get("termination_threshold", 1e-3))
    options.relative_termination_threshold = float(payload.get("relative_termination_threshold", 1e-3))
    options.random_seed = int(payload.get("random_seed", 7))
    options.configuration_space_margin = 0.0

    region = Iris(obstacles, seed, domain, options)
    vertices = _ordered_polygon(VPolytope(region).vertices())
    ellipsoid = region.MaximumVolumeInscribedEllipsoid()
    chebyshev_center = np.round(region.ChebyshevCenter(), 6).tolist()

    return {
        "success": True,
        "backend": "pydrake.geometry.optimization.Iris",
        "drake_vendor": str(DRAKE_VENDOR),
        "seed": np.round(seed, 6).tolist(),
        "region": {
            "vertices": vertices,
            "A": np.round(region.A(), 9).tolist(),
            "b": np.round(region.b(), 9).tolist(),
            "chebyshev_center": chebyshev_center,
            "max_volume_inscribed_ellipsoid": _ellipse_payload(ellipsoid),
        },
        "options": {
            "require_sample_point_is_contained": True,
            "iteration_limit": options.iteration_limit,
            "termination_threshold": options.termination_threshold,
            "relative_termination_threshold": options.relative_termination_threshold,
            "random_seed": options.random_seed,
            "configuration_space_margin": options.configuration_space_margin,
            "inflated_obstacle_margin": margin,
            "convex_obstacle_count": len(obstacles),
        },
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "DrakeIrisMapDesigner/1.0"

    def do_OPTIONS(self) -> None:
        _json_response(self, 200, {"ok": True})

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path in {"/", "/iris_map_designer.html"}:
            body = HTML_PATH.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if parsed.path == "/api/health":
            _json_response(
                self,
                200,
                {
                    "ok": DRAKE_IMPORT_ERROR is None,
                    "backend": "pydrake.geometry.optimization.Iris",
                    "drake_vendor": str(DRAKE_VENDOR),
                    "drake_vendor_present": DRAKE_VENDOR.exists(),
                    "import_error": DRAKE_IMPORT_ERROR,
                },
            )
            return
        _json_response(self, 404, {"success": False, "error": "not found"})

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path != "/api/iris":
            _json_response(self, 404, {"success": False, "error": "not found"})
            return
        try:
            payload = _read_json(self)
            result = compute_drake_iris(payload)
        except Exception as exc:
            _json_response(self, 400, {"success": False, "error": str(exc)})
            return
        _json_response(self, 200, result)

    def log_message(self, fmt: str, *args: Any) -> None:
        sys.stderr.write("%s - - [%s] %s\n" % (self.address_string(), self.log_date_time_string(), fmt % args))


def main() -> None:
    host = "127.0.0.1"
    port = 8765
    if len(sys.argv) >= 2:
        port = int(sys.argv[1])
    server = ThreadingHTTPServer((host, port), Handler)
    print(f"Drake IRIS designer: http://{host}:{port}/")
    print(f"Using pydrake vendor path: {DRAKE_VENDOR}")
    server.serve_forever()


if __name__ == "__main__":
    main()
