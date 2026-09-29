"""Summarize actual navigation-state timestamps without inventing drift truth."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np

def summarize(path):
    rows=[json.loads(line) for line in Path(path).read_text(encoding='utf-8').splitlines() if line.strip()]
    rows=[r for r in rows if r.get('schema')=='seamlessnav-navigation-v1']
    if len(rows)<2: raise ValueError('Need at least two navigation states')
    times=np.array([r['timestamp_ns'] for r in rows],dtype=np.int64)
    intervals=np.diff(times)/1e6
    target=float(rows[0]['target_output_hz'])
    transitions=[]
    for old,new in zip(rows,rows[1:]):
        if old['mode']!=new['mode']:
            transitions.append({'elapsed_s':(new['timestamp_ns']-times[0])/1e9,'from':old['mode'],'to':new['mode']})
    return {'source':str(path),'states':len(rows),'duration_s':float((times[-1]-times[0])/1e9),
        'target_hz':target,'measured_timestamp_hz':float((len(rows)-1)*1e9/(times[-1]-times[0])) if times[-1]>times[0] else None,
        'non_increasing_timestamps':int(np.count_nonzero(intervals<=0)),
        'interval_ms':{'median':float(np.median(intervals)),'p95':float(np.percentile(intervals,95)),'maximum':float(intervals.max())},
        'intervals_over_1_5_periods':int(np.count_nonzero(intervals>1500/target)),
        'modes':dict(Counter(r['mode'] for r in rows)),'transitions':transitions,
        'map_accepted_states':sum(bool((r.get('map_hypothesis') or {}).get('accepted')) for r in rows),
        'raw_drift_percent':None,'note':'Cadence is based on source timestamps, not wall-clock delivery. Drift requires an independent reference trajectory; map positions are not truth.'}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('capture',type=Path);p.add_argument('--output',type=Path);a=p.parse_args()
    text=json.dumps(summarize(a.capture),indent=2)
    if a.output: a.output.write_text(text,encoding='utf-8')
    print(text)
