"""Synthetic integration checks. Does NOT establish real 200 Hz/FOG performance."""
from pathlib import Path
import json
import math
import sys
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from navcore.edge_service import EdgeStream
from navcore.portable_speed import PortableSpeedModel, PortableSpeedStream
from scripts.summarize_navigation_jsonl import summarize

def main():
    output=ROOT/'outputs/architecture_validation';output.mkdir(parents=True,exist_ok=True)
    config=json.loads((ROOT/'config/edge.example.json').read_text())
    stream=EdgeStream(config,model_file=ROOT/'outputs/portable_speed_v1/model.json')
    states=[];latencies=[];gnss=[]
    for i in range(2401):
        ns=1_000_000_000+i*5_000_000
        before=time.perf_counter_ns()
        state=stream.event({'type':'imu','timestamp':ns,'acceleration':[0,0,9.80665],'angular_rate':[0,0,0],'normal_driving':True})
        latencies.append((time.perf_counter_ns()-before)/1e6)
        if state: states.append(state)
        if i in (0,200,1600,1800,2000,2200,2400):
            elapsed=i/200
            gnss.append(stream.event({'type':'gnss','timestamp_ns':ns,'latitude_deg':51.5+math.degrees(10*elapsed/6378137),
                'longitude_deg':-0.12,'horizontal_sigma_m':3,'speed_mps':10,'heading_deg_north_clockwise':0}))
    assert all(r['accepted'] for r in gnss)
    assert states[0]['position']['latitude_deg'] is None
    assert states[0]['mode']=='WAITING_FOR_GNSS'
    assert {'GNSS_AIDED','GNSS_DEGRADED','DEAD_RECKONING','GNSS_REACQUIRING'} <= {s['mode'] for s in states}
    assert states[-1]['mode']=='GNSS_AIDED'
    assert states[-1]['learned_speed'] is not None and not states[-1]['learned_speed_applied']
    assert abs(states[-1]['velocity']['speed_mps']-10)<0.01
    assert abs(states[-1]['diagnostics']['measured_input_hz']-200)<1e-6
    for bad in ({'type':'imu','timestamp':ns,'acceleration':[0,0,9.80665],'angular_rate':[0,0,0]},
                {'type':'gnss','timestamp_ns':ns+1,'latitude_deg':51.5,'longitude_deg':-0.12,'horizontal_sigma_m':3},
                {'type':'gnss','timestamp_ns':ns+1,'latitude_deg':51.5,'longitude_deg':-0.12,'horizontal_sigma_m':3,'heading_deg_north_clockwise':float('nan')}):
        try: stream.event(bad)
        except ValueError: pass
        else: raise AssertionError('Invalid timestamp/value was not rejected')
    path=output/'edge-synthetic.jsonl'
    path.write_text('\n'.join(json.dumps(s,allow_nan=False) for s in states),encoding='utf-8')
    report=summarize(path)
    report['synthetic_only']=True
    report['real_200hz_hardware_verified']=False
    report['desktop_event_latency_ms']={'p50':float(np.percentile(latencies,50)),'p95':float(np.percentile(latencies,95)),'max':float(max(latencies))}
    # Causal bucket parity against Android fixture and reset behavior.
    model=PortableSpeedModel.load(ROOT/'outputs/portable_speed_v1/model.json')
    bins=PortableSpeedStream(model);raw=[0,0,9.80665,0,0,9.80665,0,0,0,0,0,0]
    for i in range(200): assert bins.add_sample(1_000_000_000+i*10_000_000,raw) is None
    assert bins.add_sample(3_000_000_000,raw) is not None
    assert bins.add_sample(4_000_000_000,raw) is None and bins.latest is None
    (output/'edge-contract-report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))

if __name__=='__main__': main()
