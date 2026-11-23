"""Conservative indoor development health admission, independent of ROS transport."""
import math


def valid_scan(ranges,minimum,maximum,angle_increment):
    if not ranges or not all(math.isfinite(x) for x in (minimum,maximum,angle_increment)) or not 0<=minimum<maximum or angle_increment<=0:
        return False
    hits=0
    for value in ranges:
        if value==math.inf:
            continue  # No return is unknown, not evidence of free space.
        if not math.isfinite(value) or not minimum<=value<=maximum:
            return False
        hits+=1
    return hits/len(ranges)>=.5


def valid_covariance(covariance,max_xy=.25,max_yaw=.25):
    return len(covariance)==36 and all(math.isfinite(x) for x in covariance) and all(covariance[i]>=0 for i in [0,7,14,21,28,35]) and max(covariance[0],covariance[7])<=max_xy and covariance[35]<=max_yaw


def valid_geometry(values,quaternion):
    return all(math.isfinite(x) for x in values) and all(math.isfinite(x) for x in quaternion) and abs(sum(x*x for x in quaternion)-1.)<=.01


class HealthMonitor:
    def __init__(self,max_age=.6,pose_max_age=5.):
        self.max_age=max_age
        self.pose_max_age=pose_max_age
        self.records={}
        self.clock_stamp=None
        self.clock_advanced=None
        self.rewound=False

    def clock(self,stamp,now):
        if self.clock_stamp is not None and stamp<self.clock_stamp:
            self.rewound=True
        if self.clock_stamp is None or stamp>self.clock_stamp:
            self.clock_advanced=now
        self.clock_stamp=stamp

    def observe(self,name,stamp,now,*,valid):
        previous=self.records.get(name)
        if not math.isfinite(stamp) or not valid:
            self.records[name]={'stamp':stamp,'received':now,'valid':False}
        elif previous is None or stamp>previous['stamp']:
            self.records[name]={'stamp':stamp,'received':now,'valid':True}
        elif stamp<previous['stamp']:
            self.records[name]['valid']=False
        # Repeated stamps never refresh liveness or clear an invalid observation.

    def age(self,name,now):
        value=self.records.get(name)
        return float('inf') if value is None else now-value['received']

    def reason(self,now,ros_now):
        if self.rewound:
            return 'CLOCK_REWOUND'
        if self.clock_advanced is None:
            return 'CLOCK_MISSING'
        if now-self.clock_advanced>self.max_age:
            return 'CLOCK_STALE'
        for name in ['scan','odom','map_tf','odom_tf','amcl']:
            item=self.records.get(name)
            if item is None:
                return name.upper()+'_MISSING'
            if not item['valid']:
                return name.upper()+'_INVALID'
            limit=self.pose_max_age if name=='amcl' else self.max_age
            if now-item['received']>limit:
                return name.upper()+'_STALE'
            future=1.1 if name=='map_tf' else .1
            if not -future<=ros_now-item['stamp']<=limit:
                return name.upper()+'_TIMESTAMP'
        return ''
