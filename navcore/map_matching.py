"""Confidence-gated, offline OSM road matcher for navigation outputs.

The matcher keeps a separate map-adjusted track. It never edits the raw
GNSS/INS state used for dead-reckoning scores. Its HMM combines distance,
heading and short-range road-network continuity, and abstains when confidence
is insufficient.
"""

from __future__ import annotations

from dataclasses import dataclass
from collections import OrderedDict
import heapq
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


_ROAD_TYPES = {
    "motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link",
    "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified",
    "residential", "living_street", "service",
}
_EARTH_RADIUS_M = 6_378_137.0


@dataclass(frozen=True)
class MapObservation:
    timestamp_ns: int
    latitude_deg: float
    longitude_deg: float
    horizontal_sigma_m: float
    speed_mps: float | None = None
    heading_deg_north_clockwise: float | None = None


@dataclass(frozen=True)
class MapMatchResult:
    timestamp_ns: int
    raw_latitude_deg: float
    raw_longitude_deg: float
    matched_latitude_deg: float | None
    matched_longitude_deg: float | None
    road_id: str | None
    distance_to_road_m: float | None
    confidence: float
    accepted: bool
    reason: str


@dataclass(frozen=True)
class _RoadArc:
    segment_id: int
    way_id: str
    start_node: str
    end_node: str
    start_e_m: float
    start_n_m: float
    end_e_m: float
    end_n_m: float
    length_m: float
    bearing_deg: float


@dataclass(frozen=True)
class _Candidate:
    arc: _RoadArc
    fraction: float
    east_m: float
    north_m: float
    distance_m: float
    emission_log_score: float


class OfflineRoadNetwork:
    """Small/medium regional road graph loaded from an offline OSM XML extract."""

    def __init__(
        self,
        *,
        origin_latitude_deg: float,
        origin_longitude_deg: float,
        nodes: dict[str, tuple[float, float]],
        arcs: list[_RoadArc],
        cell_size_m: float = 60.0,
        attribution: str = "© OpenStreetMap contributors; ODbL 1.0",
    ) -> None:
        if not -90 <= origin_latitude_deg <= 90 or not -180 <= origin_longitude_deg <= 180:
            raise ValueError("map projection origin is outside valid latitude/longitude")
        if cell_size_m <= 0 or not math.isfinite(cell_size_m):
            raise ValueError("cell_size_m must be positive and finite")
        self.origin_lat = origin_latitude_deg
        self.origin_lon = origin_longitude_deg
        self.nodes = nodes
        self.arcs = arcs
        self.cell_size_m = cell_size_m
        self.attribution = attribution
        self.adjacency: dict[str, list[tuple[str, float]]] = {}
        self._spatial: dict[tuple[int, int], list[int]] = {}
        self._distance_cache: OrderedDict[tuple[str, str, int], float | None] = OrderedDict()
        for index, arc in enumerate(arcs):
            self.adjacency.setdefault(arc.start_node, []).append((arc.end_node, arc.length_m))
            min_x, max_x = sorted((arc.start_e_m, arc.end_e_m))
            min_y, max_y = sorted((arc.start_n_m, arc.end_n_m))
            for gx in range(self._cell(min_x), self._cell(max_x) + 1):
                for gy in range(self._cell(min_y), self._cell(max_y) + 1):
                    self._spatial.setdefault((gx, gy), []).append(index)

    @staticmethod
    def from_osm_xml(path: Path, *, cell_size_m: float = 60.0) -> "OfflineRoadNetwork":
        """Load routable car roads from a regional OSM .osm/.xml extract."""
        if not path.exists():
            raise FileNotFoundError(path)
        nodes_geo: dict[str, tuple[float, float]] = {}
        ways: list[tuple[str, list[str], dict[str, str]]] = []
        context = ET.iterparse(path, events=("start", "end"))
        _, root = next(context)
        for event, element in context:
            if event != "end":
                continue
            tag = element.tag.rsplit("}", 1)[-1]
            if tag == "node":
                try:
                    nodes_geo[element.attrib["id"]] = (float(element.attrib["lat"]), float(element.attrib["lon"]))
                except (KeyError, ValueError):
                    pass
                element.clear()
                root.clear()
            elif tag == "way":
                refs = [child.attrib["ref"] for child in element if child.tag.rsplit("}", 1)[-1] == "nd"]
                tags = {
                    child.attrib["k"]: child.attrib["v"]
                    for child in element if child.tag.rsplit("}", 1)[-1] == "tag"
                }
                access = tags.get("motor_vehicle", tags.get("vehicle", tags.get("access", ""))).lower()
                if tags.get("highway") in _ROAD_TYPES and len(refs) >= 2 and access not in {"no", "private"}:
                    ways.append((element.attrib.get("id", ""), refs, tags))
                element.clear()
                root.clear()
            elif tag == "osm":
                root.clear()

        if not nodes_geo or not ways:
            raise ValueError("OSM file has no usable node and car-road way data")
        origin_lat = sum(value[0] for value in nodes_geo.values()) / len(nodes_geo)
        origin_lon = sum(value[1] for value in nodes_geo.values()) / len(nodes_geo)
        nodes_local = {
            node_id: OfflineRoadNetwork._to_local(lat, lon, origin_lat, origin_lon)
            for node_id, (lat, lon) in nodes_geo.items()
        }
        arcs: list[_RoadArc] = []
        for way_id, refs, tags in ways:
            try:
                points = [(ref, nodes_local[ref]) for ref in refs]
            except KeyError:
                continue
            oneway = tags.get("oneway", "").lower()
            roundabout = tags.get("junction", "") == "roundabout"
            directions = (1,) if oneway in {"yes", "true", "1"} or roundabout else ((-1,) if oneway == "-1" else (1, -1))
            for first, second in zip(points[:-1], points[1:]):
                for direction in directions:
                    a, b = (first, second) if direction == 1 else (second, first)
                    de, dn = b[1][0] - a[1][0], b[1][1] - a[1][1]
                    length = math.hypot(de, dn)
                    if length < 0.2:
                        continue
                    bearing = (math.degrees(math.atan2(de, dn)) + 360.0) % 360.0
                    arcs.append(_RoadArc(
                        len(arcs), way_id, a[0], b[0], a[1][0], a[1][1], b[1][0], b[1][1], length, bearing
                    ))
        if not arcs:
            raise ValueError("No routable OSM segments could be built from this extract")
        return OfflineRoadNetwork(
            origin_latitude_deg=origin_lat,
            origin_longitude_deg=origin_lon,
            nodes=nodes_local,
            arcs=arcs,
            cell_size_m=cell_size_m,
        )

    @staticmethod
    def from_json(path: Path) -> "OfflineRoadNetwork":
        """Load a compact prepared road graph produced by ``prepare_offline_osm.py``."""
        document = json.loads(path.read_text(encoding="utf-8"))
        if document.get("schema_version") != 1 or document.get("format") != "seamlessnav-offline-roads-v1":
            raise ValueError("unsupported offline road database schema")
        arcs = [_RoadArc(**item) for item in document.get("arcs", [])]
        nodes = {str(key): tuple(value) for key, value in document.get("nodes", {}).items()}
        return OfflineRoadNetwork(
            origin_latitude_deg=float(document["origin_latitude_deg"]),
            origin_longitude_deg=float(document["origin_longitude_deg"]),
            nodes=nodes,
            arcs=arcs,
            cell_size_m=float(document.get("cell_size_m", 60.0)),
            attribution=str(document.get("attribution", "© OpenStreetMap contributors; ODbL 1.0")),
        )

    def write_json(self, path: Path) -> None:
        """Save the road graph as a portable, offline navigation resource."""
        path.parent.mkdir(parents=True, exist_ok=True)
        document = {
            "schema_version": 1,
            "format": "seamlessnav-offline-roads-v1",
            "origin_latitude_deg": self.origin_lat,
            "origin_longitude_deg": self.origin_lon,
            "cell_size_m": self.cell_size_m,
            "attribution": self.attribution,
            "nodes": self.nodes,
            "arcs": [arc.__dict__ for arc in self.arcs],
        }
        path.write_text(json.dumps(document, separators=(",", ":")), encoding="utf-8")

    @staticmethod
    def _to_local(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
        east = math.radians(lon - lon0) * _EARTH_RADIUS_M * math.cos(math.radians(lat0))
        north = math.radians(lat - lat0) * _EARTH_RADIUS_M
        return east, north

    def to_local(self, latitude_deg: float, longitude_deg: float) -> tuple[float, float]:
        return self._to_local(latitude_deg, longitude_deg, self.origin_lat, self.origin_lon)

    def to_geodetic(self, east_m: float, north_m: float) -> tuple[float, float]:
        lat = self.origin_lat + math.degrees(north_m / _EARTH_RADIUS_M)
        lon = self.origin_lon + math.degrees(east_m / (_EARTH_RADIUS_M * max(1e-6, math.cos(math.radians(self.origin_lat)))))
        return lat, lon

    def _cell(self, coordinate_m: float) -> int:
        return math.floor(coordinate_m / self.cell_size_m)

    def candidates(self, observation: MapObservation, *, max_candidates: int = 12) -> list[_Candidate]:
        east, north = self.to_local(observation.latitude_deg, observation.longitude_deg)
        sigma = min(max(float(observation.horizontal_sigma_m), 3.0), 50.0)
        radius = max(30.0, min(100.0, 3.0 * sigma))
        cells = math.ceil(radius / self.cell_size_m)
        indices: set[int] = set()
        base_x, base_y = self._cell(east), self._cell(north)
        for gx in range(base_x - cells, base_x + cells + 1):
            for gy in range(base_y - cells, base_y + cells + 1):
                indices.update(self._spatial.get((gx, gy), ()))
        result = []
        for index in indices:
            arc = self.arcs[index]
            dx, dy = arc.end_e_m - arc.start_e_m, arc.end_n_m - arc.start_n_m
            fraction = min(1.0, max(0.0, ((east - arc.start_e_m) * dx + (north - arc.start_n_m) * dy) / (arc.length_m**2)))
            snapped_e = arc.start_e_m + fraction * dx
            snapped_n = arc.start_n_m + fraction * dy
            distance = math.hypot(east - snapped_e, north - snapped_n)
            if distance > radius:
                continue
            score = -0.5 * (distance / sigma) ** 2 - math.log(sigma)
            if observation.speed_mps is not None and observation.speed_mps >= 2.0 and observation.heading_deg_north_clockwise is not None:
                error = abs((arc.bearing_deg - observation.heading_deg_north_clockwise + 180.0) % 360.0 - 180.0)
                heading_sigma = 35.0
                score -= 0.5 * (error / heading_sigma) ** 2
            result.append(_Candidate(arc, fraction, snapped_e, snapped_n, distance, score))
        return sorted(result, key=lambda candidate: candidate.emission_log_score, reverse=True)[:max_candidates]

    def route_distance_m(self, start: _Candidate, end: _Candidate, *, cutoff_m: float) -> float | None:
        """Shortest directed road distance between two projected road points."""
        if start.arc.segment_id == end.arc.segment_id and end.fraction >= start.fraction:
            return (end.fraction - start.fraction) * start.arc.length_m
        start_node, start_offset = start.arc.end_node, (1.0 - start.fraction) * start.arc.length_m
        end_node, end_offset = end.arc.start_node, end.fraction * end.arc.length_m
        remaining = cutoff_m - start_offset - end_offset
        if remaining < 0:
            return None
        cutoff_bucket = math.ceil(remaining / 25.0)
        cache_key = (start_node, end_node, cutoff_bucket)
        if cache_key in self._distance_cache:
            cached = self._distance_cache.pop(cache_key)
            self._distance_cache[cache_key] = cached
            return None if cached is None or cached > remaining else start_offset + cached + end_offset
        search_limit = cutoff_bucket * 25.0
        distances = {start_node: 0.0}
        queue = [(0.0, start_node)]
        while queue:
            distance, node = heapq.heappop(queue)
            if distance != distances[node]:
                continue
            if node == end_node:
                self._distance_cache[cache_key] = distance
                if len(self._distance_cache) > 20_000:
                    self._distance_cache.popitem(last=False)
                return start_offset + distance + end_offset
            if distance > search_limit:
                continue
            for neighbor, edge_length in self.adjacency.get(node, ()):
                new_distance = distance + edge_length
                if new_distance <= search_limit and new_distance < distances.get(neighbor, math.inf):
                    distances[neighbor] = new_distance
                    heapq.heappush(queue, (new_distance, neighbor))
        self._distance_cache[cache_key] = None
        if len(self._distance_cache) > 20_000:
            self._distance_cache.popitem(last=False)
        return None


@dataclass
class _BeamState:
    candidate: _Candidate
    log_score: float


class ConfidenceGatedMapMatcher:
    """Streaming HMM road matcher with abstention and map-free output preserved."""

    def __init__(
        self,
        road_network: OfflineRoadNetwork,
        *,
        confidence_threshold: float = 0.80,
        max_candidates: int = 12,
        beam_size: int = 16,
    ) -> None:
        if not 0.5 < confidence_threshold < 1.0:
            raise ValueError("confidence_threshold must be between 0.5 and 1")
        if max_candidates < 1 or beam_size < 1:
            raise ValueError("max_candidates and beam_size must be positive")
        self.network = road_network
        self.confidence_threshold = confidence_threshold
        self.max_candidates = max_candidates
        self.beam_size = beam_size
        self._beam: list[_BeamState] = []
        self._last_observation: MapObservation | None = None

    def reset(self) -> None:
        self._beam.clear()
        self._last_observation = None

    @staticmethod
    def _softmax(scores: list[float]) -> list[float]:
        top = max(scores)
        weights = [math.exp(max(-700.0, score - top)) for score in scores]
        total = sum(weights)
        return [weight / total for weight in weights]

    def update(self, observation: MapObservation) -> MapMatchResult:
        if observation.timestamp_ns < 0 or not all(math.isfinite(v) for v in (observation.latitude_deg, observation.longitude_deg, observation.horizontal_sigma_m)):
            raise ValueError("map observation requires finite position/accuracy and a non-negative timestamp")
        if not -90 <= observation.latitude_deg <= 90 or not -180 <= observation.longitude_deg <= 180:
            raise ValueError("map observation coordinate is outside valid latitude/longitude")
        if observation.horizontal_sigma_m <= 0:
            raise ValueError("horizontal_sigma_m must be positive")
        if observation.speed_mps is not None and (not math.isfinite(observation.speed_mps) or observation.speed_mps < 0):
            raise ValueError("speed_mps must be finite and non-negative when provided")
        if observation.heading_deg_north_clockwise is not None and not math.isfinite(observation.heading_deg_north_clockwise):
            raise ValueError("heading must be finite when provided")
        if self._last_observation is not None and observation.timestamp_ns <= self._last_observation.timestamp_ns:
            raise ValueError("map observation timestamps must be strictly increasing")
        candidates = self.network.candidates(observation, max_candidates=self.max_candidates)
        if not candidates:
            self._last_observation = observation
            return MapMatchResult(
                observation.timestamp_ns, observation.latitude_deg, observation.longitude_deg,
                None, None, None, None, 0.0, False, "no_road_candidate_within_search_radius",
            )

        next_states: list[_BeamState] = []
        previous_observation = self._last_observation
        if not self._beam or previous_observation is None:
            next_states = [_BeamState(candidate, candidate.emission_log_score) for candidate in candidates]
        else:
            dt = (observation.timestamp_ns - previous_observation.timestamp_ns) / 1e9
            prev_e, prev_n = self.network.to_local(previous_observation.latitude_deg, previous_observation.longitude_deg)
            cur_e, cur_n = self.network.to_local(observation.latitude_deg, observation.longitude_deg)
            observed_distance = math.hypot(cur_e - prev_e, cur_n - prev_n)
            expected_distance = max(
                observed_distance,
                max(0.0, float(observation.speed_mps or 0.0)) * dt,
            )
            beta = max(8.0, 0.35 * expected_distance + previous_observation.horizontal_sigma_m + observation.horizontal_sigma_m)
            cutoff = expected_distance + max(50.0, 3.0 * beta)
            for candidate in candidates:
                best = -math.inf
                for old in self._beam:
                    road_distance = self.network.route_distance_m(old.candidate, candidate, cutoff_m=cutoff)
                    if road_distance is None:
                        continue
                    transition_score = -abs(road_distance - observed_distance) / beta
                    score = old.log_score + transition_score + candidate.emission_log_score
                    if score > best:
                        best = score
                if math.isfinite(best):
                    next_states.append(_BeamState(candidate, best))
            if not next_states:
                # A gap or unmatched junction can break graph continuity. Start
                # a fresh hypothesis set but retain the raw fix and require confidence.
                next_states = [_BeamState(candidate, candidate.emission_log_score) for candidate in candidates]

        next_states.sort(key=lambda state: state.log_score, reverse=True)
        self._beam = next_states[:self.beam_size]
        self._last_observation = observation
        probabilities = self._softmax([state.log_score for state in self._beam])
        best = self._beam[0]
        confidence = probabilities[0]
        accepted = confidence >= self.confidence_threshold
        if not accepted:
            return MapMatchResult(
                observation.timestamp_ns, observation.latitude_deg, observation.longitude_deg,
                None, None, None, best.candidate.distance_m, confidence, False,
                "road_hypothesis_confidence_below_threshold",
            )
        matched_lat, matched_lon = self.network.to_geodetic(best.candidate.east_m, best.candidate.north_m)
        return MapMatchResult(
            observation.timestamp_ns, observation.latitude_deg, observation.longitude_deg,
            matched_lat, matched_lon, best.candidate.arc.way_id, best.candidate.distance_m,
            confidence, True, "confidence_gated_road_match",
        )

    @staticmethod
    def write_result(path: Path, result: MapMatchResult) -> None:
        """Append one JSONL map-match record without overwriting raw navigation."""
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as output:
            output.write(json.dumps(result.__dict__, separators=(",", ":")) + "\n")
