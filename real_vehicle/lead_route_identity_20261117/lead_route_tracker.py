"""Read-only route-aware lead association. No actuator or production topic output.

Uses operator setup region + measured multiview shape as acquisition evidence.
After acquisition, position/size continuity dominates; raw cluster IDs may change.
Static-background semantics require the explicitly declared controlled scene.
"""
import argparse,copy,json,math
from collections import deque
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree


def principal_support(points, radius=.085):
    """Largest local support; unassigned returns remain in the obstacle input."""
    p=np.asarray(points,float)
    if len(p)<3:return p
    tree=cKDTree(p);remaining=set(range(len(p)));groups=[]
    while remaining:
        todo=[remaining.pop()];group=[]
        while todo:
            k=todo.pop();group.append(k)
            for j in tree.query_ball_point(p[k],radius):
                if j in remaining:remaining.remove(j);todo.append(j)
        groups.append(group)
    return p[max(groups,key=len)]


class Route:
    def __init__(self,points):
        self.p=np.asarray(points,float)[:,:2]
        self.e=np.diff(self.p,axis=0);self.length=np.linalg.norm(self.e,axis=1)
        if len(self.p)<2 or np.any(self.length<1e-8):raise ValueError('Invalid route')
        self.arc=np.r_[0.,np.cumsum(self.length)]
    def project(self,xy):
        q=np.asarray(xy,float);fraction=np.clip(np.sum((q-self.p[:-1])*self.e,axis=1)/self.length**2,0,1)
        foot=self.p[:-1]+fraction[:,None]*self.e;j=int(np.argmin(np.sum((foot-q)**2,axis=1)))
        tangent=self.e[j]/self.length[j];normal=np.array([-tangent[1],tangent[0]])
        return float(self.arc[j]+fraction[j]*self.length[j]),float((q-foot[j])@normal),tangent
    def at(self,s,d):
        s=float(np.clip(s,0,self.arc[-1]));j=min(len(self.e)-1,max(0,int(np.searchsorted(self.arc,s,side='right')-1)))
        t=self.e[j]/self.length[j];n=np.array([-t[1],t[0]])
        return self.p[j]+(s-self.arc[j])*t+d*n,t


class ShapeBank:
    def __init__(self,folder):
        self.templates=[]
        for file in sorted(Path(folder).glob('*_visible_template.json')):
            v=json.loads(file.read_text());p=principal_support(v['laser_frame_xy_m'])
            if len(p)>=6:self.templates.append((file.stem,p-p.mean(axis=0)))
        if len(self.templates)!=8:raise ValueError('Expected eight newly captured views')
    def score(self,points):
        p=principal_support(points)
        if len(p)<6:return float('inf'),None
        p=p-p.mean(axis=0);p=p[::max(1,len(p)//48)]
        best=(float('inf'),None)
        # Shape/viewpoint is auxiliary evidence, never a semantic guarantee.
        for name,t in self.templates:
            for angle in np.arange(0,360,22.5):
                a=math.radians(angle);rot=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
                q=t@rot.T;dist=np.linalg.norm(p[:,None,:]-q[None,:,:],axis=2)
                # Trim the worst 20% rather than forcing every isolated return
                # into the car template. They remain real obstacles elsewhere.
                v=np.r_[np.sort(dist.min(axis=1))[:max(3,int(.8*len(p)))],np.sort(dist.min(axis=0))[:max(3,int(.8*len(q)))]]
                error=float(np.mean(v))
                if error<best[0]:best=(error,name)
        return best


class LeadTracker:
    def __init__(self,route,bank,*,body_length=None,body_width=None):
        self.route=route;self.bank=bank
        self.body_length=body_length;self.body_width=body_width
        self.length_limit=min(.80,(body_length+.22) if body_length else .65)
        self.width_limit=min(.75,(body_width+.18) if body_width else .55)
        self.lead=None;self.pending=None;self.last_stamp=None;self.history=deque()
    @staticmethod
    def center(o):return np.array([(o['xmin']+o['xmax'])/2,(o['ymin']+o['ymax'])/2])
    def candidates(self,scene,ego):
        objects=scene.get('occupancies',[]);cloud=np.asarray(scene.get('obstacle_points',[]),float).reshape(-1,2)
        es,_,_=self.route.project(ego[:2]);eligible=[]
        for k,o in enumerate(objects):
            if o.get('visibility_hazard') or o.get('memory_state')!='observed' or o.get('hits',0)<4:continue
            size=np.array([o['xmax']-o['xmin'],o['ymax']-o['ymin']]);center=self.center(o);s,d,t=self.route.project(center)
            # Axis-aligned extents vary with yaw: use longest physical diagonal.
            if size.max()>math.hypot(self.length_limit,self.width_limit) or size.max()<.06:continue
            if self.lead is None and not (.60<=s-es<=3.30 and abs(d)<=.65):continue
            if self.lead is not None:
                age=max(0,scene['source_scan_stamp']-self.lead['stamp']);expected,_=self.route.at(self.lead['s']+self.lead['speed']*age,self.lead['d'])
                if np.linalg.norm(center-expected)>.24+.8*min(age,.8):continue
            eligible.append(([k],o))
        # Try neighbouring components together; retain their separate obstacle
        # geometry. Never use their union as permission to remove other returns.
        combinations=list(eligible)
        for ii,(a,oa) in enumerate(eligible[:12]):
            for b,ob in eligible[ii+1:12]:
                low=np.minimum([oa['xmin'],oa['ymin']],[ob['xmin'],ob['ymin']]);high=np.maximum([oa['xmax'],oa['ymax']],[ob['xmax'],ob['ymax']])
                gap=np.maximum(np.maximum(np.array([oa['xmin'],oa['ymin']])-np.array([ob['xmax'],ob['ymax']]),np.array([ob['xmin'],ob['ymin']])-np.array([oa['xmax'],oa['ymax']])),0)
                if np.linalg.norm(gap)>.20 or (high-low).max()>math.hypot(self.length_limit,self.width_limit):continue
                merged=dict(oa,xmin=float(low[0]),ymin=float(low[1]),xmax=float(high[0]),ymax=float(high[1]));combinations.append((a+b,merged))
        result=[]
        for ids,o in combinations:
            low=np.array([o['xmin'],o['ymin']]);high=np.array([o['xmax'],o['ymax']]);points=cloud[np.all((cloud>=low-.02)&(cloud<=high+.02),axis=1)]
            error,view=self.bank.score(points)
            if error>.060:continue
            center=self.center(o);s,d,t=self.route.project(center);cost=error
            if self.lead is not None:
                age=max(0,scene['source_scan_stamp']-self.lead['stamp']);expected,_=self.route.at(self.lead['s']+self.lead['speed']*age,self.lead['d'])
                cost+=.35*np.linalg.norm(center-expected)
            result.append(dict(indices=ids,center=center,s=s,d=d,tangent=t,shape_error=error,template=view,cost=float(cost)))
        return sorted(result,key=lambda x:x['cost'])
    def update(self,scene,ego,*,allow_acquisition=True):
        if not scene.get('valid',False):return {'valid':False,'reason':'observation_invalid'}
        stamp=float(scene['source_scan_stamp'])
        if self.last_stamp is not None and stamp<=self.last_stamp:return {'valid':False,'reason':'duplicate_or_old_scan'}
        self.last_stamp=stamp;c=self.candidates(scene,ego);chosen=c[0] if c else None
        if chosen:
            # Competing disjoint objects must not be resolved by arbitrary ID.
            rival=next((r for r in c[1:] if not set(r['indices'])&set(chosen['indices'])),None)
            if rival and rival['cost']-chosen['cost']<.025:chosen=None;reason='ambiguous_candidates'
            else:reason='candidate'
        else:reason='no_matching_support'
        if self.lead is None and chosen:
            if not allow_acquisition:chosen=None;reason='acquisition_disabled'
            else:
                if self.pending is None or stamp-self.pending['last_seen']>.25 or np.linalg.norm(chosen['center']-self.pending['center'])>.18:
                    self.pending=dict(center=chosen['center'],stamp=stamp,last_seen=stamp,count=1)
                else:self.pending['count']+=1;self.pending['center']=chosen['center'];self.pending['last_seen']=stamp
                if self.pending['count']<3 or stamp-self.pending['stamp']<.15:chosen=None;reason='confirming_candidate'
        selected=set();lead_out=None
        if chosen:
            old=self.lead
            self.history.append((stamp,chosen['s'],chosen['d']))
            while len(self.history)>1 and stamp-self.history[0][0]>.60:self.history.popleft()
            speed=0.;lateral=0.
            if len(self.history)>1 and stamp-self.history[0][0]>=.25:
                dt=stamp-self.history[0][0];speed=(chosen['s']-self.history[0][1])/dt;lateral=(chosen['d']-self.history[0][2])/dt
            # Do not force a crossing/reversing target onto the reference line.
            aligned=abs(lateral)<=.15 and -.08<=speed<=1.0
            if old:speed=.65*old['speed']+.35*speed
            ids=[scene['occupancies'][k]['id'] for k in chosen['indices']]
            self.lead=dict(stamp=stamp,s=chosen['s'],d=chosen['d'],speed=float(speed),lateral_speed=float(lateral),ids=ids,route_aligned=aligned)
            selected=set(chosen['indices']);reason='associated';lead_out=dict(self.lead,shape_error_m=chosen['shape_error'],matched_template=chosen['template'],route_aligned=aligned,state='observed')
        elif self.lead is not None:
            age=stamp-self.lead['stamp'];reason='temporarily_unobserved' if age<=.6 else 'identity_unresolved'
            lead_out=dict(self.lead,state=reason,age_s=age)
            # Keep the old identity, not a licence to select another distant car.
        prediction=[]
        if lead_out and stamp-lead_out['stamp']<=.6 and lead_out.get('route_aligned',True):
            age=stamp-lead_out['stamp']
            for dt in np.arange(0,1.61,.1):
                xy,_=self.route.at(lead_out['s']+max(0,lead_out['speed'])*(age+dt),lead_out['d']);prediction.append([float(dt),float(xy[0]),float(xy[1])])
        labelled=copy.deepcopy(scene.get('occupancies',[]));unresolved=[]
        for k,o in enumerate(labelled):
            if k in selected:o['semantic_role']='lead_vehicle';continue
            if self.lead is None and any(k in candidate['indices'] for candidate in c):
                o['semantic_role']='pending_lead_candidate';unresolved.append(o['id']);continue
            s,d,_=self.route.project(self.center(o));could_be_lead=False
            if self.lead is not None:
                age=stamp-self.lead['stamp'];expected,_=self.route.at(self.lead['s']+max(0,self.lead['speed'])*min(age,.6),self.lead['d'])
                could_be_lead=np.linalg.norm(self.center(o)-expected)<.35+.8*min(age,.8)
            if could_be_lead and not selected:
                o['semantic_role']='unresolved_near_lead';unresolved.append(o['id'])
            else:
                o['semantic_role']='declared_static_background';o['original_velocity']=[o['vx'],o['vy']]
                o.update(vx=0.,vy=0.,has_moved=False,fixed_world=True,stationary_history=True)
        return dict(valid=True,reason=reason,lead=lead_out,predicted_route_positions=prediction,occupancies=labelled,unresolved_ids=unresolved,identity_loss_requests_stop=False,physical_motion=False,mode='shadow_only',dimensions_source='operator_reported' if self.body_length is not None and self.body_width is not None else 'unverified_defaults')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--assets',required=True);p.add_argument('--templates',required=True);p.add_argument('--input',required=True);p.add_argument('--output',required=True);p.add_argument('--config',default=str(Path(__file__).with_name('config.json')));args=p.parse_args()
    config=json.loads(Path(args.config).read_text());tracker=LeadTracker(Route(json.loads(Path(args.assets).read_text())['route']),ShapeBank(args.templates),body_length=config['lead_length_m'],body_width=config['lead_width_m'])
    with open(args.input) as source,open(args.output,'x') as dest:
        for line in source:
            v=json.loads(line);out=tracker.update(v,v['scan_pose']);dest.write(json.dumps(out)+'\n')
if __name__=='__main__':main()
