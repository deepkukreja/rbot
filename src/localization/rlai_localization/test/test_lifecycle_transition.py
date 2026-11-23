"""A missing reply must never cause an implicit second state-change request."""
import asyncio
import pytest
from rlai_localization.lifecycle_transition import Dispatch, transition_once


def exercise(states, dispatch, transition=1):
    observations=iter(states)
    requests=[]
    async def get_state():return next(observations)
    async def change_state(value):
        requests.append(value)
        return dispatch
    result=asyncio.run(transition_once(transition,get_state,change_state))
    return result,requests


def test_lost_configure_reply_is_reconciled_from_fresh_inactive_state():
    result,requests=exercise([1,2],Dispatch(sent=True,reply=None))
    assert result.success and result.reconciled
    assert result.initial_state==1 and result.final_state==2
    assert requests==[1]


def test_lost_reply_without_completed_transition_is_failure_without_retry():
    result,requests=exercise([1,1],Dispatch(sent=True,reply=None))
    assert not result.success and not result.reconciled
    assert requests==[1]


@pytest.mark.parametrize('state',[None,0,10,3,4])
def test_unknown_or_conflicting_post_state_never_establishes_success(state):
    result,requests=exercise([1,state],Dispatch(sent=True,reply=None))
    assert not result.success and requests==[1]


def test_explicit_denial_is_not_reinterpreted_as_success():
    result,requests=exercise([1,2],Dispatch(sent=True,reply=False))
    assert not result.success and requests==[1]


@pytest.mark.parametrize('state',[None,0,2,3,4,10])
def test_missing_or_unexpected_initial_state_prevents_dispatch(state):
    result,requests=exercise([state],Dispatch(sent=True,reply=True))
    assert not result.success and not result.request_sent and requests==[]


def test_unsent_request_does_not_claim_recovery_from_another_transition():
    result,requests=exercise([1,2],Dispatch(sent=False,reply=None))
    assert not result.success and not result.request_sent
    assert requests==[1]


@pytest.mark.parametrize('transition,source,target',[(1,1,2),(2,2,1),(3,2,3),(4,3,2),(5,1,4),(6,2,4),(7,3,4)])
def test_acknowledged_lifecycle_operations_still_require_target_state(transition,source,target):
    result,requests=exercise([source,target],Dispatch(sent=True,reply=True),transition)
    assert result.success and not result.reconciled and requests==[transition]


def test_acknowledgement_with_wrong_state_is_failure():
    result,requests=exercise([2,2],Dispatch(sent=True,reply=True),3)
    assert not result.success and requests==[3]


def test_unknown_transition_is_never_dispatched():
    result,requests=exercise([],Dispatch(sent=True,reply=True),99)
    assert not result.success and requests==[]
