"""Ownership-loss diagnostics exercise actual supervisor state transitions."""
from rlai_motion_supervisor.core import Supervisor
from rlai_motion_supervisor.nav_gate import ownership_loss_reason


def acquired(duration=1.):
    s=Supervisor()
    grant=s.acquire(owner='nav',source='AUTONOMY',duration=duration,now=0.)
    assert grant.ok
    return s,grant.token


def test_expired_command_names_command_timeout_not_lease_expiry():
    s,token=acquired()
    assert s.command(token=token,sequence=1,twist=(.1,0.,0.,0.,0.,0.),
        issued_at=1000.,valid_for=.2,now=.01,wall_now=1000.)
    assert ownership_loss_reason(s.tick(.22))=='OWNERSHIP_LOST_COMMAND_TIMEOUT'


def test_expired_lease_names_lease_expiry():
    s,token=acquired(.1)
    assert ownership_loss_reason(s.tick(.11))=='OWNERSHIP_LOST_LEASE_EXPIRED'


def test_invalid_geometry_names_rejected_command():
    s,token=acquired()
    assert not s.command(token=token,sequence=1,twist=(.1,.2,0.,0.,0.,0.),
        issued_at=1000.,valid_for=.2,now=.01,wall_now=1000.)
    assert ownership_loss_reason(s.tick(.02))=='OWNERSHIP_LOST_INVALID_COMMAND'


def test_explicit_release_keeps_generic_loss_reason():
    s,token=acquired()
    assert s.release(token,.01)
    assert ownership_loss_reason(s.tick(.02))=='OWNERSHIP_LOST'


def test_other_owner_cannot_be_labelled_as_expired_command():
    s,token=acquired()
    assert s.release(token,.01)
    assert s.acquire(owner='other',source='TELEOP',duration=1.,now=.02).ok
    assert ownership_loss_reason(s.tick(.03))=='OWNERSHIP_LOST'
