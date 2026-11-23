"""Monotonic lease/watchdog state machine, independent of ROS and hardware."""
from dataclasses import dataclass
import math
import re
import secrets


@dataclass(frozen=True)
class Grant:
    ok: bool
    token: str = ''
    reason: str = ''


@dataclass(frozen=True)
class Decision:
    linear: float
    angular: float
    state: str
    owner: str
    source: str
    reason: str
    lease_remaining: float
    sequence: int
    has_command: bool


class Supervisor:
    def __init__(self, *, command_timeout=.3, max_lease=2., max_forward=.5,
                 max_reverse=.35, max_angular=1.9):
        if not all(math.isfinite(v) and v>0 for v in
                   (command_timeout,max_lease,max_forward,max_reverse,max_angular)):
            raise ValueError('Limits and timeouts must be finite and positive')
        self.command_timeout=command_timeout
        self.max_lease=max_lease
        self.max_forward,self.max_reverse,self.max_angular=max_forward,max_reverse,max_angular
        self.stopped=False
        self.inhibit_reason=""
        self.clock_fault=False
        self.last_now=None
        self.last_wall=None
        self.last_wall_monotonic=None
        self._drop('STARTUP')

    def _drop(self, reason):
        self.token=''
        self.owner=''
        self.source=''
        self.lease_deadline=0.
        self.command_deadline=0.
        self.sequence=-1
        self.velocity=(0.,0.)
        self.reason=reason

    def _advance(self, now):
        if not math.isfinite(now) or (self.last_now is not None and now<self.last_now):
            self.clock_fault=True
            self.stopped=True
            self._drop('CLOCK_FAULT')
            return False
        self.last_now=now
        if self.token:
            if now>=self.lease_deadline:
                self._drop('LEASE_EXPIRED')
            elif now>=self.command_deadline:
                self._drop('COMMAND_TIMEOUT')
        return not self.clock_fault

    def _duration_valid(self, duration):
        return math.isfinite(duration) and 0<duration<=self.max_lease

    def acquire(self, *, owner, source, duration, now):
        if not self._advance(now) or self.stopped:
            return Grant(False,reason='STOPPED')
        if self.inhibit_reason:
            return Grant(False,reason='INHIBITED')
        if not re.fullmatch(r'[A-Za-z0-9_.-]{1,64}',owner) or source not in {'AUTONOMY','TELEOP'} or not self._duration_valid(duration):
            return Grant(False,reason='INVALID_REQUEST')
        if self.token:
            return Grant(False,reason='BUSY')
        self.token=secrets.token_urlsafe(24)
        self.owner,self.source=owner,source
        self.lease_deadline=now+duration
        self.command_deadline=now+self.command_timeout
        self.reason='ACQUIRED'
        return Grant(True,self.token,'ACQUIRED')

    def renew(self, token, *, duration, now):
        if not self._advance(now) or not self.token or token!=self.token or not self._duration_valid(duration):
            return False
        self.lease_deadline=now+duration
        return True

    def release(self, token, now):
        if not self._advance(now) or not self.token or token!=self.token:
            return False
        self._drop('RELEASED')
        return True

    def set_stop(self, engaged, now):
        self._advance(now)
        self.stopped=bool(engaged) or self.clock_fault
        self._drop('CLOCK_FAULT' if self.clock_fault else 'STOP_LATCHED' if engaged else 'STOP_CLEARED')

    def set_inhibit(self, reason, now):
        self._advance(now)
        if reason != self.inhibit_reason:
            self.inhibit_reason = reason
            self._drop('INHIBITED:'+reason if reason else 'INHIBIT_CLEARED')

    def command(self, *, token, sequence, twist, issued_at, valid_for, now, wall_now):
        if not self._advance(now) or self.stopped or not self.token or token!=self.token:
            return False
        if isinstance(sequence,bool) or not isinstance(sequence,int) or not 0<=sequence<2**64 or sequence<=self.sequence:
            return False
        if not all(math.isfinite(v) for v in (issued_at,valid_for,wall_now)) or not 0<valid_for<=self.command_timeout:
            return False
        if self.last_wall is not None and wall_now-self.last_wall < now-self.last_wall_monotonic-.05:
            self.clock_fault=True
            self.stopped=True
            self._drop('CLOCK_FAULT')
            return False
        self.last_wall,self.last_wall_monotonic=wall_now,now
        age=wall_now-issued_at
        if age<-.05 or age>=valid_for:
            return False
        if len(twist)!=6 or not all(math.isfinite(v) for v in twist) or any(abs(twist[i])>1e-9 for i in (1,2,3,4)) or not -self.max_reverse<=twist[0]<=self.max_forward or abs(twist[5])>self.max_angular:
            self._drop('INVALID_COMMAND')
            return False
        self.sequence=sequence
        self.velocity=(twist[0],twist[5])
        self.command_deadline=now+min(valid_for-age,self.command_timeout)
        self.reason='COMMAND_ACCEPTED'
        return True

    def tick(self, now):
        self._advance(now)
        state='STOPPED' if self.stopped else 'INHIBITED' if self.inhibit_reason else 'OWNED' if self.token and self.sequence>=0 else 'WAITING_COMMAND' if self.token else 'IDLE'
        return Decision(*self.velocity,state,self.owner,self.source,self.reason,
            max(0.,self.lease_deadline-now) if self.token else 0.,max(0,self.sequence),self.sequence>=0)
