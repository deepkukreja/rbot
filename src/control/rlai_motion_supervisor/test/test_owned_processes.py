import importlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


def module():
    return importlib.import_module('rlai_motion_supervisor.owned_processes')


def env():
    return {**os.environ,'ROS_DOMAIN_ID':'71','GZ_PARTITION':'cleanup_test_'+uuid.uuid4().hex}


def alive(pid):
    p=Path('/proc')/str(pid)/'stat'
    return p.exists() and p.read_text().rsplit(')',1)[1].split()[0]!='Z'


def test_cleanup_reaps_a_detached_child_after_its_launcher_exits(tmp_path):
    helper=module()
    marker=env()
    child_ready=tmp_path/'child_ready'
    child_pidfile=tmp_path/'child_pid'
    child_code="import signal,time;from pathlib import Path;signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN);Path("+repr(str(child_ready))+").write_text('ready');time.sleep(60)"
    parent_code="import subprocess,sys,time;from pathlib import Path;p=subprocess.Popen([sys.executable,'-c',"+repr(child_code)+"],start_new_session=True);Path("+repr(str(child_pidfile))+").write_text(str(p.pid));time.sleep(60)"
    parent=subprocess.Popen([sys.executable,'-c',parent_code],env=marker,start_new_session=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    child=None
    try:
        end=time.monotonic()+3
        while time.monotonic()<end and not child_ready.exists():
            time.sleep(.01)
        assert child_ready.exists()
        child=int(child_pidfile.read_text())
        tracked=helper.snapshot_owned([parent.pid])
        assert child in {r['pid'] for r in tracked}
        parent.terminate()
        parent.wait(timeout=2)
        result=helper.finish_owned_cleanup(tracked,domain='71',partition=marker['GZ_PARTITION'],grace=.05)
        assert result['clean'] and not alive(child)
        assert any(r['pid']==child and r['signal']=='SIGKILL' for r in result['signals'])
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait()
        if child and alive(child):
            os.kill(child,signal.SIGKILL)


def test_cleanup_does_not_signal_a_registered_process_with_different_markers():
    helper=module()
    marker=env()
    child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],env=marker,start_new_session=True)
    try:
        tracked=helper.snapshot_owned([child.pid])
        result=helper.finish_owned_cleanup(tracked,domain='71',partition='a_different_fixture',grace=.01)
        assert not result['clean'] and child.pid in result['marker_mismatch']
        assert not result['signals'] and child.poll() is None
    finally:
        child.kill()
        child.wait()


def test_cleanup_never_sweeps_an_unrelated_process_even_if_markers_match():
    helper=module()
    marker=env()
    first=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],env=marker,start_new_session=True,stderr=subprocess.DEVNULL)
    unrelated=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],env=marker,start_new_session=True,stderr=subprocess.DEVNULL)
    try:
        tracked=helper.snapshot_owned([first.pid])
        assert unrelated.pid not in {r['pid'] for r in tracked}
        result=helper.finish_owned_cleanup(tracked,domain='71',partition=marker['GZ_PARTITION'],grace=.05)
        first.wait(timeout=2)
        assert result['clean'] and unrelated.poll() is None
    finally:
        for child in [first,unrelated]:
            if child.poll() is None:
                child.kill()
            child.wait()
