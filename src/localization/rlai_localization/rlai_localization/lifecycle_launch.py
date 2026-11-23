"""Launch-owned wall-clock deadline independent of the ROS executor and clock."""
import math

from launch.actions import OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessIO

READY_LINE = 'RLAI_LOCALIZATION_STARTUP_READY'


def bounded_startup_actions(guard, timeout, *, condition=None):
    observed = {'ready': False, 'buffer': ''}

    def stdout(event):
        observed['buffer'] += event.text.decode('utf-8', errors='replace')
        lines = observed['buffer'].split('\n')
        observed['buffer'] = lines.pop()[-4096:]
        for line in lines:
            if line.strip() == READY_LINE:
                observed['ready'] = True

    def exited(event, context):
        if not context.is_shutdown:
            raise RuntimeError('Localization lifecycle relay exited unexpectedly: '+str(event.returncode))

    def deadline(context):
        if not context.is_shutdown and not observed['ready']:
            raise RuntimeError('LOCALIZATION_STARTUP_TIMEOUT: manager and node readiness unconfirmed; shutting down owned launch')

    def start_timer(context):
        seconds = float(timeout.perform(context))
        if not math.isfinite(seconds) or not 5. <= seconds <= 120.:
            raise ValueError('localization_startup_timeout must be finite and in [5, 120] seconds')
        return [TimerAction(period=seconds, actions=[OpaqueFunction(function=deadline)])]

    return [
        RegisterEventHandler(OnProcessIO(target_action=guard, on_stdout=stdout)),
        RegisterEventHandler(OnProcessExit(target_action=guard, on_exit=exited)),
        OpaqueFunction(function=start_timer, condition=condition),
    ]
