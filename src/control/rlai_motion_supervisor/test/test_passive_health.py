"""Health observation must not mutate the localization filter while idle."""
import ast
from pathlib import Path


def test_health_input_module_has_no_localization_service_client():
    path=Path(__file__).parents[1]/'rlai_motion_supervisor/health_ros.py'
    tree=ast.parse(path.read_text())
    calls=[node.func.attr for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)]
    assert 'create_client' not in calls
    assert 'call_async' not in calls
