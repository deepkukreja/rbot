"""The local transport mode must be explicit and scoped to its child command."""
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest
from rlai_localization.loopback_udp import loopback_environment


@pytest.fixture
def profile(tmp_path):
    path=tmp_path/'loopback.xml'
    path.write_text('<dds/>')
    return path


def test_environment_is_copied_and_keeps_domain_and_partition(profile):
    original={'ROS_DOMAIN_ID':'81','GZ_PARTITION':'owned_fixture','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST','ROS_LOCALHOST_ONLY':'1','RMW_FASTRTPS_PUBLICATION_MODE':'ASYNCHRONOUS'}
    changed=loopback_environment(profile,original)
    assert changed['ROS_AUTOMATIC_DISCOVERY_RANGE']=='SYSTEM_DEFAULT'
    assert changed['FASTRTPS_DEFAULT_PROFILES_FILE']==str(profile.resolve())
    assert changed['ROS_DOMAIN_ID']=='81' and changed['GZ_PARTITION']=='owned_fixture'
    assert changed['RMW_FASTRTPS_PUBLICATION_MODE']=='ASYNCHRONOUS'
    assert 'ROS_LOCALHOST_ONLY' not in changed
    assert original['ROS_AUTOMATIC_DISCOVERY_RANGE']=='LOCALHOST' and original['ROS_LOCALHOST_ONLY']=='1'
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE' not in original


@pytest.mark.parametrize('rmw',['rmw_fastrtps_cpp','rmw_fastrtps_dynamic_cpp'])
def test_supported_middleware_selection_is_preserved(profile,rmw):
    assert loopback_environment(profile,{'RMW_IMPLEMENTATION':rmw})['RMW_IMPLEMENTATION']==rmw


def test_other_middleware_is_refused(profile):
    with pytest.raises(ValueError,match='Fast DDS'):
        loopback_environment(profile,{'RMW_IMPLEMENTATION':'rmw_cyclonedds_cpp'})


def test_missing_profile_is_refused(tmp_path):
    with pytest.raises(FileNotFoundError):loopback_environment(tmp_path/'missing.xml',{})


def test_inherited_custom_profile_requires_explicit_replacement(profile,tmp_path):
    other=tmp_path/'custom.xml';other.write_text('<dds/>')
    original={'FASTRTPS_DEFAULT_PROFILES_FILE':str(other)}
    with pytest.raises(ValueError,match='profile'):
        loopback_environment(profile,original)
    assert loopback_environment(profile,original,allow_profile_override=True)['FASTRTPS_DEFAULT_PROFILES_FILE']==str(profile)
    assert original['FASTRTPS_DEFAULT_PROFILES_FILE']==str(other)


def test_reapplying_same_profile_is_allowed(profile):
    previous=loopback_environment(profile,{})
    assert loopback_environment(profile,previous)==previous


def test_wrapper_executes_argv_without_shell_expansion_and_propagates_exit(profile):
    environment=os.environ.copy()
    environment.pop('FASTRTPS_DEFAULT_PROFILES_FILE',None)
    environment['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    environment['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
    literal='value with spaces; $(do_not_execute)'
    code='import json,os,sys; print(json.dumps([sys.argv[1],os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"],os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"]])); sys.exit(7)'
    result=subprocess.run([sys.executable,'-m','rlai_localization.loopback_udp','--profile',str(profile),'--',sys.executable,'-c',code,literal],env=environment,capture_output=True,text=True,timeout=5)
    assert result.returncode==7,result.stderr
    assert json.loads(result.stdout)==[literal,'SYSTEM_DEFAULT',str(profile)]
    assert environment['ROS_AUTOMATIC_DISCOVERY_RANGE']=='LOCALHOST'
