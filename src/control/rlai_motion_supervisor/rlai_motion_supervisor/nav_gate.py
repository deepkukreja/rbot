"""Admit a Nav2 velocity once, only during its goal, retaining source transit age."""
import math
import struct


class LifecycleGate:
    required=('planner_server','controller_server','behavior_server','bt_navigator')

    def __init__(self):
        self.states={}

    def observe(self,name,state,now):
        self.states[name]=(state,now)

    def reason(self,now):
        for name in self.required:
            value=self.states.get(name)
            if value is None or now-value[1]>1.5:
                return name.upper()+'_LIFECYCLE_STALE'
            if value[0]!=3:
                return name.upper()+'_NOT_ACTIVE'
        return ''


class VelocityGate:
    def __init__(self):
        self.active=False
        self.activation_stamp=0.
        self.last_stamp=None

    def begin(self,*,activation_stamp):
        self.active=True
        self.activation_stamp=activation_stamp
        self.last_stamp=None

    def end(self):
        self.active=False

    def admit(self,stamp,ros_now,received,twist):
        if not self.active:
            return None
        if not math.isfinite(stamp) or stamp<self.activation_stamp or (self.last_stamp is not None and stamp<=self.last_stamp):
            return None
        age=ros_now-stamp
        if not -.05<=age<.2:
            return None
        if len(twist)!=6 or not all(math.isfinite(v) for v in twist) or any(abs(twist[i])>1e-9 for i in [1,2,3,4]) or not -.35<=twist[0]<=.5 or abs(twist[5])>1.9:
            raise ValueError('Invalid planar Nav2 command')
        self.last_stamp=stamp
        return {'twist':tuple(twist),'age':max(0.,age),'received':received,'stamp':stamp}


def valid_occlusion_cloud(message):
    """Validate this producer's unordered, little-endian planar XYZ32 contract.

    Empty observations are legitimate replacements, not evidence of free space.
    """
    try:
        fields=[(p.name,p.offset,p.datatype,p.count) for p in message.fields]
        if (not message.header.frame_id or message.height!=1 or message.is_bigendian
                or message.point_step!=12 or message.row_step!=message.width*12
                or len(message.data)!=message.row_step
                or fields!=[('x',0,7,1),('y',4,7,1),('z',8,7,1)]):
            return False
        return all(math.isfinite(x) and math.isfinite(y) and z==0.
                   for x,y,z in struct.iter_unpack('<fff',bytes(message.data)))
    except (AttributeError,TypeError,ValueError,struct.error):
        return False


class OcclusionGate:
    """Fresh derived acquisition required by the opt-in navigation profile."""
    def __init__(self):
        self.stamp=None
        self.received=None
        self.invalid=False

    def observe(self,stamp,now,*,valid):
        if not math.isfinite(stamp) or stamp<=0. or not valid:
            self.invalid=True
        elif self.stamp is None or stamp>self.stamp:
            self.stamp,self.received,self.invalid=stamp,now,False
        elif stamp<self.stamp:
            self.invalid=True
        # Duplicate acquisitions cannot refresh a timeout or erase an invalid input.

    def reason(self,now,ros_now):
        if self.invalid:
            return 'SCAN_OCCLUSION_INVALID'
        if self.stamp is None:
            return 'SCAN_OCCLUSION_MISSING'
        if now-self.received>.6:
            return 'SCAN_OCCLUSION_STALE'
        if not math.isfinite(ros_now) or not -.1<=ros_now-self.stamp<=.6:
            return 'SCAN_OCCLUSION_TIMESTAMP'
        return ''


def ownership_loss_reason(status):
    """Name only authoritative ownerless expiry/rejection; never infer a new owner."""
    reason=getattr(status,'reason','')
    if (getattr(status,'state',None)=='IDLE' and not getattr(status,'owner',None)
            and reason in ('COMMAND_TIMEOUT','LEASE_EXPIRED','INVALID_COMMAND')):
        return 'OWNERSHIP_LOST_'+reason
    return 'OWNERSHIP_LOST'
