"""Train a small invariant IMU speed + learned-variance model on existing drive splits.
Select checkpoints on validation only. Export remains shadow-only until integrated
outage and device evaluations are recorded; regression alone never enables fusion.
"""
from __future__ import annotations
import os
os.environ.setdefault("OPENBLAS_NUM_THREADS","1")
import argparse, json, sys, hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from navcore.portable_speed import FEATURES, invariant_channels, PortableSpeedModel
from scripts.train_speed_delta_v1 import segments, window_features

def metrics(y,p):
    return {"samples":len(y),"mae_mps":float(np.abs(y-p).mean()),"rmse_mps":float(np.sqrt(np.mean((y-p)**2)))}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset",type=Path,default=ROOT/"data/processed/iovnbd/supervised_drives_v0.npz")
    parser.add_argument("--output",type=Path,default=ROOT/"outputs/portable_speed_v1")
    parser.add_argument("--epochs",type=int,default=20)
    args=parser.parse_args()
    data=np.load(args.dataset); xx=[]; yy=[]; ss=[]; dd=[]
    for drive in np.unique(data["drive_index"]):
        rows=np.flatnonzero(data["drive_index"]==drive)
        codes=np.unique(data["split_code"][rows]); assert len(codes)==1 and codes[0] in (0,1,2)
        for begin,end in segments(data["elapsed_s"][rows],0.2):
            r=rows[begin:end]
            if len(r)<20: continue
            f=window_features(invariant_channels(data["X"][r]).astype(np.float32),20)[::2]
            y=data["y_speed_mps"][r[19:]][::2]
            valid=np.isfinite(f).all(1)&np.isfinite(y)&(y>=0)&(y<=60)
            xx.append(f[valid]); yy.append(y[valid]); ss.append(np.full(valid.sum(),codes[0])); dd.append(np.full(valid.sum(),drive))
    x=np.concatenate(xx); y=np.concatenate(yy); s=np.concatenate(ss); drives=np.concatenate(dd)
    train=s==0; val=s==1; test=s==2
    mean=x[train].mean(0); scale=np.maximum(x[train].std(0),1e-4)
    z=np.clip((x-mean)/scale,-8,8).astype(np.float32)
    tm=float(y[train].mean()); ts=max(float(y[train].std()),1.0); target=(y-tm)/ts
    rng=np.random.default_rng(26168)
    w1=rng.normal(0,0.12,(20,24)).astype(np.float32); b1=np.zeros(24,dtype=np.float32)
    w2=rng.normal(0,0.1,24).astype(np.float32); b2=np.zeros(1,dtype=np.float32)
    params=[w1,b1,w2,b2]; moment=[np.zeros_like(p) for p in params]; velocity=[np.zeros_like(p) for p in params]
    best=float("inf"); checkpoint=None; step=0
    for epoch in range(args.epochs):
        indices=rng.permutation(np.flatnonzero(train))
        for start in range(0,len(indices),512):
            r=indices[start:start+512]; h=np.tanh(z[r]@w1+b1); error=(h@w2+b2[0])-target[r]
            g=2*error/len(r); gh=g[:,None]*w2[None,:]*(1-h*h)
            grads=[z[r].T@gh+1e-4*w1,gh.sum(0),h.T@g+1e-4*w2,np.array([g.sum()])]
            step+=1
            for p,m,v,gp in zip(params,moment,velocity,grads):
                m*=0.9; m+=0.1*gp; v*=0.999; v+=0.001*gp*gp
                p-=0.001*(m/(1-0.9**step))/(np.sqrt(v/(1-0.999**step))+1e-8)
        pred=(np.tanh(z[val]@w1+b1)@w2+b2[0])*ts+tm
        score=float(np.mean((pred-y[val])**2))
        if score<best: best=score; checkpoint=[p.copy() for p in params]
        if (epoch+1)%5==0: print(f"epoch {epoch+1}, validation RMSE {np.sqrt(score):.3f} m/s",flush=True)
    w1,b1,w2,b2=checkpoint
    pred=np.clip((np.tanh(z@w1+b1)@w2+b2[0])*ts+tm,0,60)
    design=np.column_stack([z[train],np.ones(train.sum())])
    noise=np.linalg.solve(design.T@design+10*np.eye(21),design.T@np.log((pred[train]-y[train])**2+0.25))
    var=np.exp(np.clip(z@noise[:-1]+noise[-1],-4,8))
    calibration=float(np.mean((pred[val]-y[val])**2/np.maximum(var[val],0.25)))
    doc={"format":"seamlessnav-invariant-mlp-v1","feature_names":FEATURES,"window_samples":20,"feature_rate_hz":10.0,
         "mean":mean.tolist(),"scale":scale.tolist(),"w1":w1.tolist(),"b1":b1.tolist(),"w2":w2.tolist(),"b2":float(b2[0]),
         "target_mean":tm,"target_scale":ts,"noise_coefficients":noise.tolist(),"variance_calibration":calibration,
         "eligible_for_fusion":False,"activation_reason":"Requires independent integrated-outage and real-device evidence; shadow only",
         "training_dataset_sha256":hashlib.sha256(args.dataset.read_bytes()).hexdigest(),"seed":26168}
    report={"model":"Invariant MLP speed plus learned measurement variance","selection":"minimum validation MSE; test used for final reporting only",
            "splits":{name:{"drives":int(len(np.unique(drives[mask]))),"model":metrics(y[mask],pred[mask]),
                            "constant_train_median":metrics(y[mask],np.full(mask.sum(),np.median(y[train])))} for name,mask in [("train",train),("validation",val),("test",test)]},
            "fusion_enabled":False,"limitations":["IMU-only speed is not universally observable","No real labelled pothole classifier","Validation variance is not a guaranteed integrity bound"]}
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/"model.json").write_text(json.dumps(doc,indent=2))
    (args.output/"metrics.json").write_text(json.dumps(report,indent=2))
    PortableSpeedModel(doc)
    print(json.dumps(report,indent=2))

if __name__=="__main__": main()
