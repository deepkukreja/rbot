"""Bounded Linux cleanup for recorded descendants of this fixture's launchers."""
import os
from pathlib import Path
import signal
import time


def _record(pid):
    path=Path('/proc')/str(pid)
    try:
        if path.stat().st_uid!=os.getuid():
            return None
        fields=path.joinpath('stat').read_text().rsplit(')',1)[1].split()
        if fields[0]=='Z':
            return None
        return {'pid':int(pid),'ppid':int(fields[1]),'start_ticks':fields[19],'uid':os.getuid()}
    except (OSError,ValueError,IndexError):
        return None


def snapshot_owned(launcher_pids):
    """Record current same-UID descendants before their launcher can exit."""
    records={}
    for path in Path('/proc').iterdir():
        if path.name.isdigit():
            value=_record(int(path.name))
            if value:
                records[value['pid']]=value
    selected={int(pid) for pid in launcher_pids if int(pid) in records}
    while True:
        children={pid for pid,value in records.items() if value['ppid'] in selected}
        added=children-selected
        if not added:
            break
        selected.update(added)
    return [records[pid] for pid in sorted(selected)]


def _same_process(reference):
    actual=_record(reference['pid'])
    return actual is not None and actual['start_ticks']==reference['start_ticks'] and actual['uid']==reference['uid']


def _matches_fixture(reference,domain,partition):
    if not _same_process(reference):
        return False
    try:
        data=Path('/proc',str(reference['pid']),'environ').read_bytes().split(b'\0')
        return ('ROS_DOMAIN_ID='+domain).encode() in data and ('GZ_PARTITION='+partition).encode() in data
    except OSError:
        return False


def finish_owned_cleanup(identities,*,domain,partition,grace=.5):
    """Recheck identity and fixture markers before every signal; never sweep by name."""
    if not 0<grace<=5:
        raise ValueError('Cleanup grace must be in (0,5] seconds')
    references={value['pid']:value for value in identities}
    result={'tracked_count':len(references),'signals':[],'marker_mismatch':[]}
    for value in [signal.SIGINT,signal.SIGTERM,signal.SIGKILL]:
        for reference in references.values():
            if not _same_process(reference):
                continue
            if not _matches_fixture(reference,str(domain),partition):
                if reference['pid'] not in result['marker_mismatch']:
                    result['marker_mismatch'].append(reference['pid'])
                continue
            try:
                os.kill(reference['pid'],value)
                result['signals'].append({'pid':reference['pid'],'signal':value.name})
            except ProcessLookupError:
                pass
        deadline=time.monotonic()+grace
        while time.monotonic()<deadline and any(_same_process(r) for r in references.values()):
            time.sleep(.01)
        if not any(_same_process(r) for r in references.values()):
            break
    result['remaining']=[r['pid'] for r in references.values() if _same_process(r)]
    result['clean']=not result['remaining']
    return result
