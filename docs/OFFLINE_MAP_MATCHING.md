# Offline road-map matching

`navcore/map_matching.py` adds a regional OSM road graph and a streaming HMM-style matcher. It scores proximity and travel direction, then checks road-network continuity. If the best road hypothesis does not exceed the configured confidence threshold, it abstains and returns only the raw coordinate. A match is a separate result object and never overwrites the raw GNSS/INS output used for drift scoring.

The Android app accepts the prepared `seamlessnav-offline-roads-v1` JSON using **Load offline road map**. It loads and spatially indexes the graph on the sensor worker, then shows the raw navigation state and a separate map hypothesis/confidence in the navigation panel. Android currently uses spatial candidates and distance/heading emission scores; it does not yet run the Python graph-continuity HMM, alter the filter, draw map tiles, or render a moving vehicle icon.

## Prepare local road data

Obtain a small OSM XML extract for the test route and retain OSM attribution. Convert it to a compact local graph:

```powershell
python scripts/prepare_offline_osm.py `
  --osm-xml data/raw/osm/test-area.osm `
  --output-json data/processed/osm/test-area-roads.json
```

The matcher supports `.osm`/XML extracts. It does not download maps or read PBF files directly; select a region-sized extract or convert a PBF to OSM XML before preparation. The generated JSON preserves the projection origin and OpenStreetMap attribution.

## Match a recorded GNSS/navigation CSV

```powershell
python scripts/map_match_trace.py `
  --offline-map-json data/processed/osm/test-area-roads.json `
  --track-csv "mobile captured details/open gps 2/seamlessnav_20260928_125036.csv" `
  --output-jsonl outputs/map_matching/gnss-map-matches.jsonl
```

By default the tool selects `sensor=gnss` rows and reads the SeamlessNav capture column names. It writes a separate JSONL record containing the raw fix, accepted matched fix (if any), candidate road, distance, confidence and abstention reason. With an application navigation-state CSV, pass its timestamp/coordinate/speed/heading column names explicitly.

## Current boundaries

- The Android matcher is a first display-only port; only the Python/edge matcher currently includes road-network continuity hypotheses.
- Map points use a local tangent-plane projection and are intended for regional driving maps, not continental-scale graphs.
- Road matching does not correct the fusion filter state or count as raw dead-reckoning accuracy.
- Candidate confidence thresholds and road filters need route-level validation, particularly at intersections, parallel roads, bridges and GNSS multipath.
- OSM coverage and licensing attribution are data concerns. This repository contains no road extract.
