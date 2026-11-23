"""Deterministic local ownership and watchdog behavior; no ROS/hardware imports."""
from importlib import import_module
import math

import pytest


@pytest.fixture
def supervisor():
    return import_module('rlai_motion_supervisor.core').Supervisor()


def acquire(s,now=0.,owner='nav',source='AUTONOMY',duration=1.):
    result=s.acquire(owner=owner,source=source,duration=duration,now=now)
    assert result.ok
    return result.token


def command(s,token,now=.01,sequence=1,**updates):
    args=dict(token=token,sequence=sequence,twist=(.2,0.,0.,0.,0.,.3),
              issued_at=100.,valid_for=.25,now=now,wall_now=100.+now)
    args.update(updates)
    return s.command(**args)


def test_no_owner_starts_with_zero(supervisor):
    result=supervisor.tick(0.)
    assert (result.linear,result.angular,result.state)==(0.,0.,'IDLE')


def test_competing_sources_cannot_acquire_or_preempt(supervisor):
    token=acquire(supervisor)
    other=supervisor.acquire(owner='operator',source='TELEOP',duration=1.,now=.01)
    assert not other.ok and other.reason=='BUSY' and not other.token
    assert command(supervisor,token)
    assert supervisor.tick(.02).owner=='nav'


def test_reacquiring_same_owner_requires_explicit_release(supervisor):
    acquire(supervisor)
    assert not supervisor.acquire(owner='nav',source='AUTONOMY',duration=1.,now=.01).ok


def test_wrong_lease_cannot_change_or_release_owner(supervisor):
    token=acquire(supervisor)
    assert command(supervisor,token)
    assert not command(supervisor,'wrong',sequence=2)
    assert not supervisor.release('wrong',.02)
    assert supervisor.tick(.03).linear==.2


def test_missing_initial_command_expires_ownership(supervisor):
    token=acquire(supervisor)
    result=supervisor.tick(.31)
    assert result.linear==0. and not result.owner and result.reason=='COMMAND_TIMEOUT'
    assert not command(supervisor,token,now=.32)


def test_command_watchdog_revokes_and_cannot_resume_without_new_lease(supervisor):
    old=acquire(supervisor)
    assert command(supervisor,old)
    result=supervisor.tick(.251)
    assert result.linear==0. and result.reason=='COMMAND_TIMEOUT'
    assert not command(supervisor,old,now=.26,sequence=2)
    new=acquire(supervisor,now=.27,owner='operator',source='TELEOP')
    assert new!=old
    assert supervisor.tick(.28).linear==0.


def test_lease_expires_even_with_a_fresh_command(supervisor):
    token=acquire(supervisor,duration=.15)
    assert command(supervisor,token,now=.1)
    result=supervisor.tick(.15)
    assert result.reason=='LEASE_EXPIRED' and result.linear==0.


def test_renewal_does_not_extend_command_deadline(supervisor):
    token=acquire(supervisor,duration=.15)
    assert command(supervisor,token)
    assert supervisor.renew(token,duration=1.,now=.1)
    assert supervisor.tick(.251).reason=='COMMAND_TIMEOUT'


def test_expired_lease_cannot_be_renewed(supervisor):
    token=acquire(supervisor,duration=.1)
    assert not supervisor.renew(token,duration=1.,now=.1)


def test_release_handover_requires_new_token_and_fresh_command(supervisor):
    old=acquire(supervisor)
    assert command(supervisor,old)
    assert supervisor.release(old,.05)
    assert supervisor.tick(.05).linear==0.
    new=acquire(supervisor,now=.06,owner='operator',source='TELEOP')
    assert new!=old and not command(supervisor,old,now=.07,sequence=2)
    assert supervisor.tick(.07).linear==0.
    assert command(supervisor,new,now=.08)


def test_stop_is_latched_idempotent_and_clear_does_not_resume(supervisor):
    token=acquire(supervisor)
    assert command(supervisor,token)
    supervisor.set_stop(True,.02)
    supervisor.set_stop(True,.03)
    assert supervisor.tick(.04).state=='STOPPED'
    assert not supervisor.acquire(owner='operator',source='TELEOP',duration=1.,now=.05).ok
    supervisor.set_stop(False,.06)
    assert not command(supervisor,token,now=.07,sequence=2)
    assert supervisor.tick(.07).linear==0.
    assert acquire(supervisor,now=.08)!=token


@pytest.mark.parametrize('updates',[
    {'issued_at':99.}, {'issued_at':101.}, {'valid_for':0.}, {'valid_for':.4},
    {'sequence':0}, {'sequence':2**64}, {'sequence':True},
])
def test_stale_future_invalid_lifetime_and_replays_cannot_refresh_output(supervisor,updates):
    token=acquire(supervisor)
    assert command(supervisor,token)
    assert not command(supervisor,token,now=.1,**updates)
    assert supervisor.tick(.251).linear==0.


def test_duplicate_or_out_of_order_commands_do_not_refresh_watchdog(supervisor):
    token=acquire(supervisor)
    assert command(supervisor,token,sequence=5)
    for seq in [5,4,3,5]:
        assert not command(supervisor,token,now=.2,sequence=seq)
    assert supervisor.tick(.251).reason=='COMMAND_TIMEOUT'


@pytest.mark.parametrize('twist',[
    (.51,0.,0.,0.,0.,0.),(-.36,0.,0.,0.,0.,0.),(.1,0.,0.,0.,0.,1.91),
    (.1,.01,0.,0.,0.,0.),(.1,0.,0.,.01,0.,0.),
    (math.nan,0.,0.,0.,0.,0.),(.1,0.,0.,0.,0.,math.inf),
])
def test_invalid_or_out_of_limits_owned_command_revokes_immediately(supervisor,twist):
    token=acquire(supervisor)
    assert command(supervisor,token)
    assert not command(supervisor,token,sequence=2,twist=twist)
    result=supervisor.tick(.02)
    assert result.linear==0. and not result.owner and result.reason=='INVALID_COMMAND'


@pytest.mark.parametrize('owner,source,duration',[
    ('','AUTONOMY',1.),('owner','UNKNOWN',1.),('owner','TELEOP',0.),
    ('owner','TELEOP',2.1),('owner','TELEOP',math.nan),('x'*65,'TELEOP',1.),
])
def test_invalid_ownership_requests_do_not_arm(supervisor,owner,source,duration):
    assert not supervisor.acquire(owner=owner,source=source,duration=duration,now=0.).ok
    assert supervisor.tick(.01).linear==0.


def test_command_expiry_accounts_for_transit_age(supervisor):
    token=acquire(supervisor)
    assert command(supervisor,token,now=.2,issued_at=100.,wall_now=100.2)
    assert supervisor.tick(.249).linear==.2
    assert supervisor.tick(.251).linear==0.


def test_new_process_rejects_old_lease(supervisor):
    old=acquire(supervisor)
    restarted=type(supervisor)()
    assert not command(restarted,old)
    assert acquire(restarted,now=.02)!=old


def test_backwards_monotonic_input_latches_a_clock_fault(supervisor):
    token=acquire(supervisor,now=1.)
    assert not command(supervisor,token,now=.9)
    assert supervisor.tick(1.01).state=='STOPPED'


def test_host_wall_clock_rollback_cannot_make_a_delayed_command_current(supervisor):
    token=acquire(supervisor)
    assert command(supervisor,token)
    assert not command(supervisor,token,now=.1,sequence=2,wall_now=99.5,issued_at=99.5)
    assert supervisor.tick(.11).state=='STOPPED'
