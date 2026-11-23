"""Single-dispatch lifecycle transitions with bounded transport supplied by caller."""
from dataclasses import dataclass


# lifecycle_msgs: transition -> (required initial primary state, target state).
STATES = {1: (1, 2), 2: (2, 1), 3: (2, 3), 4: (3, 2),
          5: (1, 4), 6: (2, 4), 7: (3, 4)}


@dataclass(frozen=True)
class Dispatch:
    sent: bool
    reply: bool | None


@dataclass(frozen=True)
class TransitionResult:
    success: bool
    reason: str
    request_sent: bool = False
    reconciled: bool = False
    initial_state: int | None = None
    final_state: int | None = None


async def transition_once(transition, get_state, change_state):
    """Send at most once; only a fresh target state can reconcile a lost reply."""
    if transition not in STATES:
        return TransitionResult(False, 'UNSUPPORTED_TRANSITION')
    source, target = STATES[transition]
    initial = await get_state()
    if initial != source:
        return TransitionResult(False, 'INITIAL_STATE_UNAVAILABLE_OR_UNEXPECTED',
                                initial_state=initial)
    dispatch = await change_state(transition)
    if not dispatch.sent:
        return TransitionResult(False, 'NOT_DISPATCHED', initial_state=initial)
    if dispatch.reply is False:
        return TransitionResult(False, 'DENIED', True, initial_state=initial)
    final = await get_state()
    if final != target:
        return TransitionResult(False, 'TARGET_STATE_UNCONFIRMED', True,
                                initial_state=initial, final_state=final)
    reconciled = dispatch.reply is None
    return TransitionResult(True, 'RECONCILED' if reconciled else 'ACKNOWLEDGED',
                            True, reconciled, initial, final)
