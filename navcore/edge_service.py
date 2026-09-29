"""Local JSONL sensor service: python -m navcore.edge_service --config config.json.

One input event per line, one versioned navigation state per emitted IMU tick.
This is a process/pipe interface; no network server or uploaded sensor data.
GNSS timestamps must already use the normalized monotonic nanosecond clock.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time
from .edge import EdgeNavigationEngine, ExternalImuAdapter, ExternalImuConfig, ExternalImuSample
from .map_matching import OfflineRoadNetwork, ConfidenceGatedMapMatcher, MapObservation
from .portable_speed import PortableSpeedModel, PortableSpeedStream

class EdgeStream:
    def __init__(self, config, road_file=None, model_file=None):
        self.output_hz=float(config.get("output_hz",200))
        self.engine=EdgeNavigationEngine(ExternalImuAdapter(ExternalImuConfig(**config["imu"])),output_hz=self.output_hz)
        self.origin=None
        self.last_imu_ns=None
        self.last_gnss_observed_ns=None
        self.last_map_ns=None
        self.last_map=None
        self.matcher=ConfidenceGatedMapMatcher(OfflineRoadNetwork.from_json(Path(road_file))) if road_file else None
        self.model_stream=PortableSpeedStream(PortableSpeedModel.load(model_file)) if model_file else None

    def event(self, event):
        tick=time.perf_counter_ns()
        if event["type"]=="gnss":
            ns=int(event["timestamp_ns"])
            lat=float(event["latitude_deg"]); lon=float(event["longitude_deg"]); sigma=float(event["horizontal_sigma_m"])
            if not (-85 <= lat <= 85 and -180 <= lon <= 180 and 0 < sigma <= 35 and ns > 0):
                raise ValueError("invalid GNSS coordinate, timestamp or uncertainty")
            if self.last_gnss_observed_ns is not None and ns<=self.last_gnss_observed_ns:
                raise ValueError("GNSS timestamps must increase")
            if self.last_imu_ns is not None and not 0 <= self.last_imu_ns-ns <= 1_500_000_000:
                raise ValueError("GNSS timestamp stale or outside the IMU clock")
            for field in ('speed_mps','heading_deg_north_clockwise'):
                value=event.get(field)
                if value is not None and (not math.isfinite(float(value)) or (field=='speed_mps' and value<0)):
                    raise ValueError(f"invalid {field}")
            self.last_gnss_observed_ns=ns
            origin=self.origin or (lat,lon)
            e=math.radians(lon-origin[1])*6378137*math.cos(math.radians(origin[0]))
            n=math.radians(lat-origin[0])*6378137
            accepted=self.engine.update_gnss(ns,east_m=e,north_m=n,horizontal_sigma_m=sigma,
                speed_mps=event.get("speed_mps"),course_deg_north_clockwise=event.get("heading_deg_north_clockwise"))
            if accepted: self.origin=origin
            return {"schema":"seamlessnav-event-v1","type":"gnss_result","timestamp_ns":ns,"accepted":accepted}
        if event["type"]!="imu": raise ValueError("event type must be imu or gnss")
        quality=float(event.get("quality",1.0))
        if not 0<=quality<=1: raise ValueError("quality must lie in [0,1]")
        sample=ExternalImuSample(event["timestamp"],tuple(event["acceleration"]),tuple(event["angular_rate"]),
            tuple(event["gravity"]) if event.get("gravity") is not None else None)
        # The adapter enforces increasing IMU time and normalizes clocks/units.
        result=self.engine.process_imu(sample,quality=quality,normal_driving=bool(event.get("normal_driving",False)),stationary=bool(event.get("stationary",False)))
        self.last_imu_ns=self.engine.filter.last_timestamp_ns
        if self.model_stream:
            cfg=self.engine.adapter.config
            gravity=sample.gravity_specific_force_sensor or cfg.gravity_specific_force_sensor_mps2
            if gravity is not None:
                a_scale=9.80665 if cfg.acceleration_unit=='g' else 1.0
                g_scale=math.pi/180 if cfg.angular_rate_unit=='deg/s' else 1.0
                acceleration=[float(v)*a_scale for v in sample.acceleration]
                linear=acceleration if cfg.acceleration_is_gravity_compensated else [v-g for v,g in zip(acceleration,gravity)]
                included=[v+g for v,g in zip(acceleration,gravity)] if cfg.acceleration_is_gravity_compensated else acceleration
                self.model_stream.add_sample(self.last_imu_ns,included+list(gravity)+linear+[float(v)*g_scale for v in sample.angular_rate])
            # Candidate diagnostics only: no model is silently promoted on external sensors.
        if result is None: return None
        state=result.state
        lat=lon=None
        if self.origin is not None:
            lat=self.origin[0]+math.degrees(state.north_m/6378137)
            lon=self.origin[1]+math.degrees(state.east_m/(6378137*math.cos(math.radians(self.origin[0]))))
        heading=(90-math.degrees(state.heading_rad_from_east_ccw))%360
        if self.matcher and lat is not None and (self.last_map_ns is None or state.timestamp_ns-self.last_map_ns>=100_000_000):
            match=self.matcher.update(MapObservation(state.timestamp_ns,lat,lon,state.horizontal_sigma_m,state.speed_mps,heading))
            self.last_map={"timestamp_ns":state.timestamp_ns,"accepted":match.accepted,"confidence":match.confidence,
                "latitude_deg":match.matched_latitude_deg,"longitude_deg":match.matched_longitude_deg,"road_id":match.road_id,"reason":match.reason}
            self.last_map_ns=state.timestamp_ns
        fix=self.engine.filter.last_gnss_ns
        return {"schema":"seamlessnav-navigation-v1","source":"edge","timestamp_ns":state.timestamp_ns,
            "target_output_hz":self.output_hz,"position":{"latitude_deg":lat,"longitude_deg":lon,
                "east_m":state.east_m if self.origin else None,"north_m":state.north_m if self.origin else None},
            "velocity":{"east_mps":state.east_velocity_mps,"north_mps":state.north_velocity_mps,"speed_mps":state.speed_mps,
                "heading_deg_north_clockwise":heading},"uncertainty":{"horizontal_sigma_m":state.horizontal_sigma_m,"velocity_sigma_mps":state.velocity_sigma_mps},
            "mode":state.mode,"gnss_age_s":max(0,(state.timestamp_ns-fix)/1e9) if fix is not None else None,
            "last_gnss_accepted":self.engine.filter.last_gnss_accepted,"map_hypothesis":self.last_map,
            "learned_speed":self.model_stream.latest if self.model_stream else None,
            "learned_speed_applied":False,"calibration":{"source":"declared sensor_to_vehicle_rotation","ready":True},
            "diagnostics":asdict(result.diagnostics),"service_processing_ms":(time.perf_counter_ns()-tick)/1e6}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",type=Path,required=True)
    parser.add_argument("--roads",type=Path)
    parser.add_argument("--model",type=Path,help="Experimental learned-speed diagnostics, never fused by this service")
    args=parser.parse_args()
    stream=EdgeStream(json.loads(args.config.read_text()),args.roads,args.model)
    for line_number,line in enumerate(sys.stdin,1):
        if not line.strip(): continue
        try:
            event=json.loads(line,parse_constant=lambda value: (_ for _ in ()).throw(ValueError(f"invalid number {value}")))
            output=stream.event(event)
        except (ValueError,KeyError,TypeError,OverflowError) as exc:
            output={"schema":"seamlessnav-error-v1","line":line_number,"error":str(exc)}
        if output is not None: print(json.dumps(output,allow_nan=False,separators=(",",":")),flush=True)

if __name__=="__main__": main()
