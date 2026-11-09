"""Analisis estatico (ast) de navigation.launch.py. No requiere ROS."""

from __future__ import annotations

import ast
import re
from pathlib import Path
from typing import Any

import pytest

ROOT = Path(__file__).resolve().parents[2]
BRINGUP = ROOT / "src" / "hrobot_bringup"
NAV_LAUNCH = BRINGUP / "launch" / "navigation.launch.py"
PARAMS = BRINGUP / "config" / "nav2_params.yaml"
SRC = NAV_LAUNCH.read_text(encoding="utf-8")
TREE = ast.parse(SRC)
NAV_NODE_NAMES = ["controller_server", "planner_server", "velocity_smoother", "bt_navigator"]


def _strings() -> list[str]:
    return [
        n.value for n in ast.walk(TREE) if isinstance(n, ast.Constant) and isinstance(n.value, str)
    ]


def _declared() -> dict[str, dict[str, ast.expr]]:
    out: dict[str, dict[str, ast.expr]] = {}
    for n in ast.walk(TREE):
        is_decl = (
            isinstance(n, ast.Call)
            and isinstance(n.func, ast.Name)
            and n.func.id == "DeclareLaunchArgument"
        )
        if is_decl and isinstance(n, ast.Call) and isinstance(n.args[0], ast.Constant):
            out[str(n.args[0].value)] = {str(k.arg): k.value for k in n.keywords}
    return out


def _load_validator() -> Any:
    func = next(
        n
        for n in TREE.body
        if isinstance(n, ast.FunctionDef) and n.name == "validate_nav_cmd_vel_topic"
    )
    ns: dict[str, Any] = dict(_module_constants())
    # Se ejecuta solo la funcion pura extraida del AST propio del repo.
    exec(compile(ast.Module([func], []), "validator", "exec"), ns)  # noqa: S102
    return ns["validate_nav_cmd_vel_topic"]


def test_defines_generate_launch_description() -> None:
    names = [n.name for n in TREE.body if isinstance(n, ast.FunctionDef)]
    assert "generate_launch_description" in names


def test_declared_arguments() -> None:
    args = _declared()
    assert {"use_sim_time", "map", "nav_cmd_vel_topic", "params_file"} <= set(args)
    assert "default_value" not in args["map"]
    assert ast.literal_eval(args["use_sim_time"]["default_value"]) == "false"
    assert ast.literal_eval(args["nav_cmd_vel_topic"]["default_value"]) == "/cmd_vel_nav"
    assert "default_value" in args["params_file"]


def _module_constants() -> dict[str, str]:
    out: dict[str, str] = {}
    for n in TREE.body:
        if (
            isinstance(n, ast.Assign)
            and len(n.targets) == 1
            and isinstance(n.targets[0], ast.Name)
            and isinstance(n.value, ast.Constant)
            and isinstance(n.value.value, str)
        ):
            out[n.targets[0].id] = n.value.value
    return out


def _resolve(node: ast.expr) -> str:
    """Resuelve constantes, nombres de modulo y LaunchConfiguration("x") -> "<LC:x>"."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.Name):
        return _module_constants()[node.id]
    if (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "LaunchConfiguration"
        and isinstance(node.args[0], ast.Constant)
    ):
        return f"<LC:{node.args[0].value}>"
    raise AssertionError(f"destino de remapeo no resoluble: {ast.dump(node)}")


def _gen_func() -> ast.FunctionDef:
    return next(
        n
        for n in TREE.body
        if isinstance(n, ast.FunctionDef) and n.name == "generate_launch_description"
    )


def _remaps_by_node() -> dict[str, list[tuple[str, str]]]:
    out: dict[str, list[tuple[str, str]]] = {}
    for n in ast.walk(_gen_func()):
        if not (isinstance(n, ast.Call) and isinstance(n.func, ast.Name)):
            continue
        if n.func.id != "_nav_node":
            continue
        executable = _resolve(n.args[1])
        remaps: ast.expr | None = n.args[3] if len(n.args) > 3 else None
        for kw in n.keywords:
            if kw.arg == "remappings":
                remaps = kw.value
        pairs: list[tuple[str, str]] = []
        if remaps is not None:
            assert isinstance(remaps, ast.List)
            for tup in remaps.elts:
                assert isinstance(tup, ast.Tuple) and len(tup.elts) == 2
                pairs.append((_resolve(tup.elts[0]), _resolve(tup.elts[1])))
        out[executable] = pairs
    return out


def test_velocity_chain_remaps() -> None:
    raw = _module_constants()["CONTROLLER_RAW_TOPIC"]
    remaps = _remaps_by_node()
    assert remaps["controller_server"] == [("cmd_vel", raw)]
    assert sorted(remaps["velocity_smoother"]) == sorted(
        [("cmd_vel", raw), ("cmd_vel_smoothed", "<LC:nav_cmd_vel_topic>")]
    )
    for name in ("planner_server", "bt_navigator", "map_server", "amcl"):
        assert remaps.get(name, []) == []


def test_no_remap_targets_cmd_vel() -> None:
    all_pairs = [p for pairs in _remaps_by_node().values() for p in pairs]
    assert all_pairs
    for _src, dst in all_pairs:
        assert dst.strip("/") != "cmd_vel"
    assert _module_constants()["CONTROLLER_RAW_TOPIC"].strip("/") != "cmd_vel"


def test_check_topic_wired_before_nodes() -> None:
    elts = next(
        n.args[0]
        for n in ast.walk(_gen_func())
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name)
        and n.func.id == "LaunchDescription"
    )
    assert isinstance(elts, ast.List)
    opaque = [
        i
        for i, e in enumerate(elts.elts)
        if isinstance(e, ast.Call)
        and isinstance(e.func, ast.Name)
        and e.func.id == "OpaqueFunction"
        and any(
            k.arg == "function" and isinstance(k.value, ast.Name) and k.value.id == "_check_topic"
            for k in e.keywords
        )
    ]
    assert len(opaque) == 1
    starred = [i for i, e in enumerate(elts.elts) if isinstance(e, ast.Starred)]
    assert starred
    assert opaque[0] < min(starred)


def test_check_topic_calls_validator_and_raises() -> None:
    func = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == "_check_topic")
    calls = [
        n.func.id
        for n in ast.walk(func)
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
    ]
    assert "validate_nav_cmd_vel_topic" in calls
    raised = [
        n
        for n in ast.walk(func)
        if isinstance(n, ast.Raise)
        and isinstance(n.exc, ast.Call)
        and isinstance(n.exc.func, ast.Name)
        and n.exc.func.id == "RuntimeError"
    ]
    assert raised


EXPECTED_NODES = {
    "map_server": ("_nav_node", "nav2_map_server"),
    "amcl": ("_nav_node", "nav2_amcl"),
    "controller_server": ("_nav_node", "nav2_controller"),
    "planner_server": ("_nav_node", "nav2_planner"),
    "velocity_smoother": ("_nav_node", "nav2_velocity_smoother"),
    "bt_navigator": ("_nav_node", "nav2_bt_navigator"),
    "lifecycle_manager_localization": ("_lifecycle", None),
    "lifecycle_manager_navigation": ("_lifecycle", None),
}


def _node_calls() -> list[ast.Call]:
    return [
        n
        for n in ast.walk(_gen_func())
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name)
        and n.func.id in ("_nav_node", "_lifecycle")
    ]


def _lc_name(node: ast.expr) -> str | None:
    if (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "LaunchConfiguration"
        and isinstance(node.args[0], ast.Constant)
    ):
        return str(node.args[0].value)
    return None


def _params_is_shared(arg: ast.expr) -> bool:
    """True si el argumento es `params` o una lista que lo expande con `*params`."""
    if isinstance(arg, ast.Name):
        return arg.id == "params"
    if isinstance(arg, ast.List):
        return any(
            isinstance(e, ast.Starred) and isinstance(e.value, ast.Name) and e.value.id == "params"
            for e in arg.elts
        )
    return False


def _call_key(call: ast.Call) -> str:
    assert isinstance(call.func, ast.Name)
    if call.func.id == "_nav_node":
        return _resolve(call.args[1])
    return _resolve(call.args[0])


def _assign_value(func: ast.FunctionDef, name: str) -> ast.expr:
    return next(
        n.value
        for n in ast.walk(func)
        if isinstance(n, ast.Assign)
        and isinstance(n.targets[0], ast.Name)
        and n.targets[0].id == name
    )


def _node_kwargs(func: ast.FunctionDef) -> dict[str, ast.expr]:
    return {
        k.arg: k.value
        for n in ast.walk(func)
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "Node"
        for k in n.keywords
        if k.arg
    }


def test_expected_nodes_package_executable_and_params() -> None:
    calls = {_call_key(c): c for c in _node_calls()}
    assert set(calls) == set(EXPECTED_NODES)
    assert len(_node_calls()) == len(EXPECTED_NODES)
    for key, (kind, package) in EXPECTED_NODES.items():
        call = calls[key]
        assert isinstance(call.func, ast.Name)
        assert call.func.id == kind
        if kind == "_nav_node":
            assert _resolve(call.args[0]) == package
            assert _params_is_shared(call.args[2]), key
        else:
            assert _params_is_shared(call.args[1]), key


def test_shared_params_include_params_file_and_use_sim_time() -> None:
    value = _assign_value(_gen_func(), "params")
    assert isinstance(value, ast.List)
    assert _lc_name(value.elts[0]) == "params_file"
    d = value.elts[1]
    assert isinstance(d, ast.Dict)
    assert [k.value for k in d.keys if isinstance(k, ast.Constant)] == ["use_sim_time"]
    assert isinstance(d.values[0], ast.Name) and d.values[0].id == "use_sim_time"
    use_sim = _assign_value(_gen_func(), "use_sim_time")
    assert isinstance(use_sim, ast.Call)
    assert _lc_name(use_sim.args[0]) == "use_sim_time"


def test_map_server_yaml_filename_is_map_argument() -> None:
    call = next(c for c in _node_calls() if _call_key(c) == "map_server")
    extra = call.args[2]
    assert isinstance(extra, ast.List)
    dicts = [e for e in extra.elts if isinstance(e, ast.Dict)]
    assert len(dicts) == 1
    d = dicts[0]
    assert [k.value for k in d.keys if isinstance(k, ast.Constant)] == ["yaml_filename"]
    assert _lc_name(d.values[0]) == "map"


def test_helpers_forward_name_and_parameters() -> None:
    funcs = {n.name: n for n in TREE.body if isinstance(n, ast.FunctionDef)}
    nav_kw = _node_kwargs(funcs["_nav_node"])
    for key, var in (
        ("package", "package"),
        ("executable", "executable"),
        ("name", "executable"),
        ("parameters", "params"),
    ):
        assert isinstance(nav_kw[key], ast.Name) and nav_kw[key].id == var, key
    lc_kw = _node_kwargs(funcs["_lifecycle"])
    assert _resolve(lc_kw["package"]) == "nav2_lifecycle_manager"
    assert _resolve(lc_kw["executable"]) == "lifecycle_manager"
    assert isinstance(lc_kw["name"], ast.Name) and lc_kw["name"].id == "name"
    assert isinstance(lc_kw["parameters"], ast.Name) and lc_kw["parameters"].id == "params"


def test_sensors_and_safety_are_included() -> None:
    gen = _gen_func()
    includes = [
        n
        for n in ast.walk(gen)
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name)
        and n.func.id == "IncludeLaunchDescription"
    ]
    assert len(includes) == 1
    comp = next(n for n in ast.walk(gen) if isinstance(n, ast.ListComp))
    assert includes[0] in list(ast.walk(comp.elt))
    iterable = comp.generators[0].iter
    assert isinstance(iterable, ast.Tuple)
    assert [e.value for e in iterable.elts if isinstance(e, ast.Constant)] == ["sensors", "safety"]
    src = ast.get_source_segment(SRC, includes[0]) or ""
    assert ".launch.py" in src
    assert "use_sim_time" in src
    ld = next(
        n.args[0]
        for n in ast.walk(gen)
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name)
        and n.func.id == "LaunchDescription"
    )
    assert isinstance(ld, ast.List)
    assert any(
        isinstance(e, ast.Starred) and isinstance(e.value, ast.Name) and e.value.id == "includes"
        for e in ld.elts
    )


def test_check_topic_applies_normalized_value() -> None:
    func = next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == "_check_topic")
    rets = [n for n in ast.walk(func) if isinstance(n, ast.Return)]
    assert len(rets) == 1
    ret = rets[0].value
    assert isinstance(ret, ast.List) and len(ret.elts) == 1
    call = ret.elts[0]
    assert isinstance(call, ast.Call)
    assert isinstance(call.func, ast.Name) and call.func.id == "SetLaunchConfiguration"
    assert isinstance(call.args[0], ast.Constant) and call.args[0].value == "nav_cmd_vel_topic"
    assert isinstance(call.args[1], ast.Name) and call.args[1].id == "normalized"
    assigned = [
        n
        for n in ast.walk(func)
        if isinstance(n, ast.Assign)
        and isinstance(n.targets[0], ast.Name)
        and n.targets[0].id == "normalized"
        and isinstance(n.value, ast.Call)
        and isinstance(n.value.func, ast.Name)
        and n.value.func.id == "validate_nav_cmd_vel_topic"
    ]
    assert assigned


def test_verify_comment_present() -> None:
    assert "# VERIFY (human): ros2 topic info /cmd_vel -v" in SRC


@pytest.mark.parametrize("frag", ["/home/", "/Users/", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION"])
def test_no_forbidden_fragments(frag: str) -> None:
    assert frag not in SRC


def test_lifecycle_node_names_match_params() -> None:
    text = PARAMS.read_text(encoding="utf-8")
    section = text.split("lifecycle_manager_navigation:")[1]
    match = re.search(r"node_names:\s*\[(.*?)\]", section)
    assert match
    names = [s.strip().strip("\"'") for s in match.group(1).split(",")]
    assert names == NAV_NODE_NAMES
    assert all(name in _strings() for name in names)


def _bt_navigator_params() -> ast.List:
    call = next(c for c in _node_calls() if _call_key(c) == "bt_navigator")
    assert isinstance(call.args[2], ast.List)
    return call.args[2]


def _bt_xml_dict() -> ast.Dict:
    dicts = [e for e in _bt_navigator_params().elts if isinstance(e, ast.Dict)]
    assert len(dicts) == 1
    return dicts[0]


def test_bt_navigator_receives_bt_xml_from_share() -> None:
    d = _bt_xml_dict()
    keys = [k.value for k in d.keys if isinstance(k, ast.Constant)]
    assert keys == ["default_nav_to_pose_bt_xml"]
    seg = ast.get_source_segment(SRC, d.values[0]) or ""
    assert "bringup" in seg
    assert '"config"' in seg
    assert "BT_XML_NAME" in seg
    gen_src = ast.get_source_segment(SRC, _gen_func()) or ""
    assert 'Path(get_package_share_directory("hrobot_bringup"))' in gen_src


def test_bt_xml_name_and_file_exists() -> None:
    assert _module_constants()["BT_XML_NAME"] == "bt_navigate_no_recovery.xml"
    assert (BRINGUP / "config" / "bt_navigate_no_recovery.xml").is_file()


def test_bt_xml_param_after_params_file() -> None:
    elts = _bt_navigator_params().elts
    shared = next(
        i
        for i, e in enumerate(elts)
        if isinstance(e, ast.Starred) and isinstance(e.value, ast.Name) and e.value.id == "params"
    )
    override = next(i for i, e in enumerate(elts) if isinstance(e, ast.Dict))
    assert shared < override


def test_create3_namespace_declared_default_r2() -> None:
    arg = _declared()["create3_namespace"]
    assert ast.literal_eval(arg["default_value"]) == "/r2"


def test_odom_override_after_params_file() -> None:
    gen_src = ast.get_source_segment(SRC, _gen_func()) or ""
    assert 'odom = {"odom_topic": LaunchConfiguration(ODOM_CONFIG)}' in gen_src
    for key in ("velocity_smoother", "bt_navigator"):
        call = next(c for c in _node_calls() if _call_key(c) == key)
        assert isinstance(call.args[2], ast.List)
        elts = call.args[2].elts
        shared = next(i for i, e in enumerate(elts) if isinstance(e, ast.Starred))
        odom = next(i for i, e in enumerate(elts) if isinstance(e, ast.Name) and e.id == "odom")
        assert shared < odom
    assert 'namespaced(namespace, "/odom")' in SRC


def _func(name: str) -> ast.FunctionDef:
    return next(n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == name)


def test_tf_remappings_helper_maps_tf_and_tf_static() -> None:
    ret = next(n for n in ast.walk(_func("_tf_remappings")) if isinstance(n, ast.Return))
    assert isinstance(ret.value, ast.List)
    pairs = []
    for tup in ret.value.elts:
        assert isinstance(tup, ast.Tuple)
        pairs.append((_resolve(tup.elts[0]), ast.get_source_segment(SRC, tup.elts[1])))
    assert pairs == [
        ("tf", "LaunchConfiguration(TF_CONFIG)"),
        ("tf_static", "LaunchConfiguration(TF_STATIC_CONFIG)"),
    ]


def test_all_nodes_receive_tf_remappings() -> None:
    # Cada Node(...) del launch pasa por _nav_node o _lifecycle; ambos anaden los remapeos TF.
    for helper in ("_nav_node", "_lifecycle"):
        remap = _node_kwargs(_func(helper))["remappings"]
        assert "_tf_remappings()" in (ast.get_source_segment(SRC, remap) or ""), helper
    direct = [
        n
        for n in ast.walk(_gen_func())
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "Node"
    ]
    assert direct == []
    assert len(_node_calls()) == len(EXPECTED_NODES)


def test_tf_topics_set_from_namespace() -> None:
    src = ast.get_source_segment(SRC, _func("_set_odom_topic")) or ""
    assert "namespaced(namespace, TF_TOPIC)" in src
    assert "namespaced(namespace, TF_STATIC_TOPIC)" in src
    assert _module_constants()["TF_CONFIG"] == "create3_tf_topic"
    assert _module_constants()["TF_STATIC_CONFIG"] == "create3_tf_static_topic"


def test_check_topic_forbids_namespaced_cmd_vel() -> None:
    assert 'namespaced(namespace, "/cmd_vel")' in SRC
    assert "create3_namespace" in (ast.get_source_segment(SRC, _gen_func()) or "")


def test_validator_rejects_namespaced_cmd_vel() -> None:
    forbidden = ("/cmd_vel", "/r2/cmd_vel")
    for topic in ("/r2/cmd_vel", "r2/cmd_vel", " /r2/cmd_vel/"):
        with pytest.raises(ValueError):
            _load_validator()(topic, forbidden=forbidden)
    assert _load_validator()("/cmd_vel_nav", forbidden=forbidden) == "/cmd_vel_nav"
    assert _load_validator()("/r2/cmd_vel") == "/r2/cmd_vel"  # un solo argumento: sin cambios


@pytest.mark.parametrize("topic", ["/cmd_vel_nav", "cmd_vel_nav"])
def test_validator_accepts(topic: str) -> None:
    assert _load_validator()(topic) == topic


@pytest.mark.parametrize(
    ("raw", "expected"),
    [(" /cmd_vel_nav", "/cmd_vel_nav"), ("cmd_vel_nav \n", "cmd_vel_nav")],
)
def test_validator_returns_normalized(raw: str, expected: str) -> None:
    assert _load_validator()(raw) == expected


@pytest.mark.parametrize(
    "topic",
    [
        "/cmd_vel",
        "cmd_vel",
        "//cmd_vel",
        "/cmd_vel/",
        "",
        "  ",
        "/",
        " /cmd_vel",
        "cmd_vel_nav_raw",
        "/cmd_vel_nav_raw",
        " /cmd_vel_nav_raw ",
    ],
)
def test_validator_rejects(topic: str) -> None:
    with pytest.raises(ValueError):
        _load_validator()(topic)
