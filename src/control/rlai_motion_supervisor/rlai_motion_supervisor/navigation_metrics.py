"""Independent simulated navigation scoring; missing truth never implies success."""
import bisect
import math


def interpolate_truth(samples,stamp,max_gap=.1):
    if not samples or not math.isfinite(stamp):
        return None
    times=[p['stamp'] for p in samples]
    index=bisect.bisect_left(times,stamp)
    if index<len(samples) and times[index]==stamp:
        value=samples[index]
        return dict(value) if all(math.isfinite(value[k]) for k in ['x','y','yaw']) else None
    if index==0 or index==len(samples):
        return None
    a,b=samples[index-1],samples[index]
    gap=b['stamp']-a['stamp']
    if not 0<gap<=max_gap or not all(math.isfinite(p[k]) for p in [a,b] for k in ['x','y','yaw']):
        return None
    fraction=(stamp-a['stamp'])/gap
    return {'stamp':stamp,'x':a['x']+fraction*(b['x']-a['x']),
        'y':a['y']+fraction*(b['y']-a['y']),
        'yaw':math.remainder(a['yaw']+fraction*math.remainder(b['yaw']-a['yaw'],2*math.pi),2*math.pi)}


def score_goal(status,goal,truth,xy_limit=.35,yaw_limit=.2):
    if len(goal)!=3 or not all(math.isfinite(v) for v in goal):
        raise ValueError('Benchmark goal must be a finite map pose')
    if truth is None or not all(math.isfinite(truth[k]) for k in ['x','y','yaw']):
        return {'passed':False,'reason':'TRUTH_UNAVAILABLE','xy_error_m':None,'yaw_error_rad':None}
    xy=math.hypot(truth['x']-goal[0],truth['y']-goal[1])
    yaw=abs(math.remainder(truth['yaw']-goal[2],2*math.pi))
    passed=status==4 and xy<=xy_limit and yaw<=yaw_limit
    return {'passed':passed,'reason':'PASSED' if passed else 'ACTION_NOT_SUCCEEDED' if status!=4 else 'TRUTH_GOAL_TOLERANCE',
        'xy_error_m':xy,'yaw_error_rad':yaw}
