"""Evidence checks for sampled passage clearance and sustained stopping."""
import math


def footprint_clearance(pose, *, axis, lower, upper, half_length, half_width):
    """Signed side-wall clearance of an oriented rectangular footprint."""
    if axis not in ('x','y') or not all(math.isfinite(v) for v in [lower,upper,half_length,half_width]) or lower>=upper or min(half_length,half_width)<=0:
        raise ValueError('Invalid passage or footprint geometry')
    try:
        x,y,yaw=(pose[name] for name in ['x','y','yaw'])
        if not all(math.isfinite(v) for v in [x,y,yaw]):return None
    except (KeyError,TypeError):
        return None
    c,s=abs(math.cos(yaw)),abs(math.sin(yaw))
    extent=half_length*c+half_width*s if axis=='x' else half_length*s+half_width*c
    center=x if axis=='x' else y
    return min(center-extent-lower,upper-center-extent)


def settled_latency(samples, *, onset, until, max_gap=.15, linear_tolerance=0., angular_tolerance=0.):
    """Start of sustained observed standstill; None means evidence is insufficient."""
    if not all(math.isfinite(v) for v in [onset,until,max_gap,linear_tolerance,angular_tolerance]) or until<=onset or max_gap<=0 or min(linear_tolerance,angular_tolerance)<0:
        raise ValueError('Invalid observation window or tolerance')
    try:
        if any(not all(math.isfinite(p[key]) for key in ['at','x','z']) for p in samples):return None
        values=[p for p in samples if onset<=p['at']<=until]
    except (KeyError,TypeError):
        return None
    if len(values)<3 or values[0]['at']-onset>max_gap or until-values[-1]['at']>max_gap:
        return None
    if any(not 0<=b['at']-a['at']<=max_gap for a,b in zip(values,values[1:])):
        return None
    moving=[i for i,p in enumerate(values) if abs(p['x'])>linear_tolerance or abs(p['z'])>angular_tolerance]
    first=(moving[-1]+1) if moving else 0
    return None if first>=len(values) else values[first]['at']-onset


def conservative_box_clearance(pose, *, center, size, half_length, half_width):
    """Lower bound on padded-rectangle clearance using its enclosing circle.

    The box is axis aligned; positive values prove sampled separation. A
    negative value can reflect conservative circle overlap, not contact.
    """
    cx,cy=center
    sx,sy=size
    if not all(math.isfinite(v) for v in [cx,cy,sx,sy,half_length,half_width]) or min(sx,sy,half_length,half_width)<=0:
        raise ValueError('Invalid box or footprint geometry')
    try:
        x,y=pose['x'],pose['y']
        if not all(math.isfinite(v) for v in [x,y]):return None
    except (KeyError,TypeError):
        return None
    dx,dy=abs(x-cx)-sx/2,abs(y-cy)-sy/2
    point_distance=math.hypot(max(dx,0),max(dy,0))+min(max(dx,dy),0)
    return point_distance-math.hypot(half_length,half_width)
