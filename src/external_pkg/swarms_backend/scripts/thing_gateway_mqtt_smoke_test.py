#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
thing_gateway 单一 MQTT 冒烟脚本：services 全量用例 + 可选 ROS 云台 8 元校验。

覆盖:
  - stream_start / stream_stop、target_types_config（多 ID、重复 ID、[] 全检→ROS [-1]、负例单元素 -1 / 混用 / 超界 128）
  - task_control（共用 task_bid）+ alarm_set（data.common.enabled）
  - payload_control：云台角速度（roll_rate/…）/reset/zoom/wiper、录像、灯、喊话器（on/off）
  - 负例：未知 method、空参、enabled_types 非法、zoom 非数字、pos_flag 禁止、action 越界等

依赖: pip install paho-mqtt

  python3 scripts/thing_gateway_mqtt_smoke_test.py -b 127.0.0.1
  python3 scripts/thing_gateway_mqtt_smoke_test.py -s dev -u user -P pass

可选（需 source ROS2 + rclpy）：对标记的云台用例校验网关发布的 Float32MultiArray(8) 与 MQTT control_params 一致

  python3 scripts/thing_gateway_mqtt_smoke_test.py --verify-ros --ros-topic /{ns}/perception/input/ptz_ctrl

云台 ROS 8 元: [home, pos_flag, roll, pitch, yaw, zoom_flag, zoom, wiper]。
  MQTT 非回中：home=-1，pos_flag=-2，三槽为 roll_rate/pitch_rate/yaw_rate；禁止 MQTT `home` 键。
  MQTT `reset:true` 须独占 control_params；回中 ROS home=1，pos_flag=-2（与角速度帧一致）。
  zoom 缺省或 0 表示不变倍（zoom_flag=-1）。
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("请先安装: pip install paho-mqtt", file=sys.stderr)
    sys.exit(1)

RESULT_SUCCESS = 0
RESULT_INVALID_PARAM = 1001
RESULT_UNKNOWN = 1999


@dataclass(frozen=True)
class SmokeCase:
    name: str
    method: str
    common: dict
    expect: int
    bid: Optional[str] = None
    # 为 True 时：若启用 --verify-ros，则要求 common 为 payload_type=1 且校验 ptz_ctrl 8 元
    verify_ptz_ros8: bool = False
    # 若非空：与 verify_ptz_ros8 联用时，直接比对此 8 元（用于 reset 回中帧，避免走 expected_ros_ptz8）
    ptz_ros8_expected: Optional[List[float]] = None


def _make_client(client_id: str, userdata: dict) -> mqtt.Client:
    try:
        from paho.mqtt.enums import CallbackAPIVersion  # type: ignore

        return mqtt.Client(CallbackAPIVersion.VERSION2, client_id=client_id, userdata=userdata)
    except Exception:
        return mqtt.Client(client_id=client_id, userdata=userdata)


def _on_connect_v1(client: mqtt.Client, userdata: Any, flags: Dict, rc: int) -> None:
    if rc == 0:
        client.subscribe(userdata["reply_topic"], qos=1)
    else:
        print(f"[MQTT] connect failed rc={rc}", file=sys.stderr)


def _on_connect_v2(client: mqtt.Client, userdata: Any, flags: Any, reason_code: Any, properties: Any = None) -> None:
    if getattr(reason_code, "is_failure", False):
        print(f"[MQTT] connect failed: {reason_code}", file=sys.stderr)
        return
    client.subscribe(userdata["reply_topic"], qos=1)


def _on_message(client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage) -> None:
    try:
        root = json.loads(msg.payload.decode("utf-8"))
    except Exception as e:
        print(f"[reply] JSON 解码失败: {e} raw={msg.payload!r}", file=sys.stderr)
        return
    tid = root.get("tid")
    if not tid:
        return
    lock: threading.Lock = userdata["reply_lock"]
    replies: Dict[str, dict] = userdata["replies"]
    with lock:
        replies[str(tid)] = root


def device_obj(sn: str) -> dict:
    return {
        "sn": sn,
        "type": 3,
        "sub_type": 0,
        "vendor": "test_vendor",
        "name": "test_name",
    }


def make_body(tid: str, method: str, sn: str, common: dict, bid: Optional[str]) -> dict:
    body: Dict[str, Any] = {
        "tid": tid,
        "ts": int(time.time() * 1000),
        "method": method,
        "device": device_obj(sn),
        "data": {"common": common},
    }
    if bid is not None:
        body["bid"] = bid
    return body


class SmokeSession:
    def __init__(self, broker: str, port: int, sn: str, username: Optional[str], password: Optional[str]):
        self.sn = sn
        self.services_topic = f"thing/product/{sn}/services"
        self.reply_topic = f"thing/product/{sn}/services_reply"
        self._reply_lock = threading.Lock()
        self._replies: Dict[str, dict] = {}
        ud = {"reply_lock": self._reply_lock, "replies": self._replies, "reply_topic": self.reply_topic}
        self._client = _make_client(f"py-smoke-{uuid.uuid4().hex[:8]}", ud)
        self._client.on_connect = _on_connect_v2 if hasattr(mqtt, "CallbackAPIVersion") else _on_connect_v1  # type: ignore
        self._client.on_message = _on_message
        if username:
            self._client.username_pw_set(username, password or "")
        self._client.connect(broker, port, keepalive=60)
        self._client.loop_start()
        time.sleep(0.4)

    def close(self) -> None:
        self._client.loop_stop()
        try:
            self._client.disconnect()
        except Exception:
            pass

    def publish_json(self, body: dict) -> str:
        tid = str(body["tid"])
        self._client.publish(self.services_topic, json.dumps(body, ensure_ascii=False), qos=1)
        return tid

    def wait_reply(self, tid: str, timeout: float) -> Optional[dict]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._reply_lock:
                if tid in self._replies:
                    return self._replies.pop(tid)
            time.sleep(0.05)
        return None


def reply_result(root: dict) -> Optional[int]:
    try:
        return int(root["data"]["common"]["result"])
    except Exception:
        return None


def expected_ros_ptz8(control_params: dict) -> List[float]:
    """与 C++ handle_ptz_control 一致（非 reset）：至少含 roll_rate/pitch_rate/yaw_rate 之一。"""
    if not any(k in control_params for k in ("roll_rate", "pitch_rate", "yaw_rate")):
        raise ValueError("expected_ros_ptz8: need roll_rate/pitch_rate/yaw_rate")

    if "zoom" not in control_params or control_params["zoom"] is None:
        zoom_flag, zoom = -1.0, 1.0
    else:
        z = control_params["zoom"]
        if not isinstance(z, (int, float)):
            raise ValueError("zoom must be number")
        zf = float(z)
        if zf == 0.0:
            zoom_flag, zoom = -1.0, 1.0
        else:
            zoom_flag, zoom = 1.0, zf

    def opt_float(key: str, default: float) -> float:
        if key not in control_params:
            return default
        v = control_params[key]
        if isinstance(v, bool):
            return default
        if isinstance(v, (int, float)):
            return float(v)
        return default

    home = -1.0
    wiper = opt_float("wiper", -1.0)
    pos_flag = -2.0
    roll = opt_float("roll_rate", 0.0)
    pitch = opt_float("pitch_rate", 0.0)
    yaw = opt_float("yaw_rate", 0.0)

    return [home, pos_flag, roll, pitch, yaw, zoom_flag, zoom, wiper]


def _float_vec_close(a: List[float], b: List[float], eps: float = 1e-4) -> bool:
    return len(a) == len(b) and all(abs(x - y) <= eps for x, y in zip(a, b))


class RosPtzSniffer:
    """主线程 spin，在收到 services_reply 后匹配下一条 ptz_ctrl Float32MultiArray(8)。"""

    def __init__(self, topic: str) -> None:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray

        rclpy.init(args=None)
        self._rclpy = rclpy
        self._node = Node("thing_gateway_smoke_ptz_sniff")
        self._q: queue.Queue = queue.Queue()
        self._node.create_subscription(Float32MultiArray, topic, self._cb, 10)

    def _cb(self, msg: Any) -> None:
        self._q.put([float(x) for x in msg.data])

    def drain(self) -> None:
        while True:
            try:
                self._q.get_nowait()
            except queue.Empty:
                break

    def expect_match(self, want: List[float], timeout: float) -> None:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._rclpy.spin_once(self._node, timeout_sec=0.05)
            try:
                while True:
                    got = self._q.get_nowait()
                    if len(got) == 8 and _float_vec_close(got, want):
                        return
            except queue.Empty:
                pass
        raise AssertionError(f"{timeout}s 内未收到匹配的 8 元 ptz_ctrl，期望 {want}")

    def close(self) -> None:
        self._node.destroy_node()
        self._rclpy.shutdown()


def run_case(
    session: SmokeSession,
    case: SmokeCase,
    sn: str,
    timeout: float,
    ros_sniffer: Optional[RosPtzSniffer],
    ros_wait: float,
) -> bool:
    if case.verify_ptz_ros8:
        if case.method != "payload_control" or case.common.get("payload_type") != 1:
            print(f"FAIL {case.name}: 内部错误 verify_ptz_ros8 仅用于 payload_type=1")
            return False
        if "control_params" not in case.common and case.ptz_ros8_expected is None:
            print(f"FAIL {case.name}: 缺少 control_params")
            return False

    tid = str(uuid.uuid4())
    body = make_body(tid, case.method, sn, case.common, case.bid)
    req_method = case.method

    if ros_sniffer is not None and case.verify_ptz_ros8:
        ros_sniffer.drain()

    session.publish_json(body)
    rep = session.wait_reply(tid, timeout=timeout)
    if rep is None:
        print(f"FAIL {case.name}: 超时未收到 tid={tid} 的 services_reply")
        return False
    if str(rep.get("tid", "")) != tid:
        print(f"FAIL {case.name}: tid 不一致 {json.dumps(rep, ensure_ascii=False)}")
        return False
    if rep.get("method") != req_method:
        print(f"FAIL {case.name}: method 期望 {req_method!r} 得到 {rep.get('method')!r}")
        return False
    r = reply_result(rep)
    if r != case.expect:
        print(f"FAIL {case.name}: expect result={case.expect}, got {r} {json.dumps(rep, ensure_ascii=False)}")
        return False

    if ros_sniffer is not None and case.verify_ptz_ros8 and case.expect == RESULT_SUCCESS:
        try:
            if case.ptz_ros8_expected is not None:
                want = case.ptz_ros8_expected
            else:
                want = expected_ros_ptz8(case.common["control_params"])
            ros_sniffer.expect_match(want, ros_wait)
        except Exception as e:
            print(f"FAIL {case.name}: ROS ptz_ctrl 校验: {e}")
            return False

    print(f"OK   {case.name}: result={r}")
    return True


def build_cases(task_bid: str, include_negative: bool) -> List[SmokeCase]:
    """感知 → 任务 → 告警 → 云台(多组) → 其它载荷 → 任务结束 → 负例。"""
    cases: List[SmokeCase] = [
        SmokeCase("stream_start", "stream_start", {"push_url": "rtsp://test/stream"}, RESULT_SUCCESS),
        SmokeCase("stream_stop", "stream_stop", {"stream_id": "s1"}, RESULT_SUCCESS),
        SmokeCase("target_types_config [2]", "target_types_config", {"enabled_types": [2]}, RESULT_SUCCESS),
        SmokeCase("target_types_config [0,1]", "target_types_config", {"enabled_types": [0, 1]}, RESULT_SUCCESS),
        SmokeCase(
            "target_types_config 重复 ID [2,2]（原样下发）",
            "target_types_config",
            {"enabled_types": [2, 2]},
            RESULT_SUCCESS,
        ),
        SmokeCase(
            "target_types_config 空数组全检（ROS [-1]）",
            "target_types_config",
            {"enabled_types": []},
            RESULT_SUCCESS,
        ),
        SmokeCase(
            "target_types_config [3] BALLOON（附录B合法类型）",
            "target_types_config",
            {"enabled_types": [3]},
            RESULT_SUCCESS,
        ),
        SmokeCase(
            "target_types_config [4] TRASH（附录B合法类型）",
            "target_types_config",
            {"enabled_types": [4]},
            RESULT_SUCCESS,
        ),
        SmokeCase(
            "target_types_config [0,1,3,4] 全部合法类型",
            "target_types_config",
            {"enabled_types": [0, 1, 3, 4]},
            RESULT_SUCCESS,
        ),
        SmokeCase(
            "payload_control ptz reset:true（§4.2.6 回中）",
            "payload_control",
            {"payload_type": 1, "control_params": {"reset": True}},
            RESULT_SUCCESS,
            verify_ptz_ros8=True,
            ptz_ros8_expected=[1.0, -2.0, 0.0, 0.0, 0.0, -1.0, 1.0, -1.0],
        ),
        SmokeCase(
            "task_control start",
            "task_control",
            {"task_action": 1},
            RESULT_SUCCESS,
            bid=task_bid,
        ),
        SmokeCase("alarm_set", "alarm_set", {"enabled": False}, RESULT_SUCCESS),
        # 云台 payload_type=1（verify_ptz_ros8 在 --verify-ros 时校验 ROS 8 元）
        SmokeCase(
            "payload_control ptz 角速度 roll_rate/pitch_rate/yaw_rate",
            "payload_control",
            {
                "payload_type": 1,
                "control_params": {"roll_rate": 0.1, "pitch_rate": -0.2, "yaw_rate": 3.14},
            },
            RESULT_SUCCESS,
            verify_ptz_ros8=True,
        ),
        SmokeCase(
            "payload_control ptz 仅 roll_rate 单轴",
            "payload_control",
            {
                "payload_type": 1,
                "control_params": {"roll_rate": 0.5, "pitch_rate": 0.0, "yaw_rate": 0.0},
            },
            RESULT_SUCCESS,
            verify_ptz_ros8=True,
        ),
        SmokeCase(
            "payload_control ptz 仅 pitch_rate 单轴",
            "payload_control",
            {
                "payload_type": 1,
                "control_params": {"roll_rate": 0.0, "pitch_rate": 0.5, "yaw_rate": 0.0},
            },
            RESULT_SUCCESS,
            verify_ptz_ros8=True,
        ),
        SmokeCase(
            "payload_control ptz 仅 yaw_rate 单轴",
            "payload_control",
            {
                "payload_type": 1,
                "control_params": {"roll_rate": 0.0, "pitch_rate": 0.0, "yaw_rate": 0.5},
            },
            RESULT_SUCCESS,
            verify_ptz_ros8=True,
        ),
        SmokeCase(
            "payload_control ptz 角速度 + zoom",
            "payload_control",
            {
                "payload_type": 1,
                "control_params": {"roll_rate": 0.0, "pitch_rate": 0.0, "yaw_rate": 0.0, "zoom": 2.5},
            },
            RESULT_SUCCESS,
            verify_ptz_ros8=True,
        ),
        SmokeCase(
            "payload_control ptz 角速度 + zoom/wiper（ROS home 恒 -1）",
            "payload_control",
            {
                "payload_type": 1,
                "control_params": {
                    "roll_rate": 0.0,
                    "pitch_rate": 0.0,
                    "yaw_rate": 0.0,
                    "zoom": 1.0,
                    "wiper": 1.0,
                },
            },
            RESULT_SUCCESS,
            verify_ptz_ros8=True,
        ),
        SmokeCase(
            "payload_control ptz 角速度 + wiper off",
            "payload_control",
            {
                "payload_type": 1,
                "control_params": {"roll_rate": 0.0, "pitch_rate": 0.0, "yaw_rate": 0.0, "wiper": 0.0},
            },
            RESULT_SUCCESS,
            verify_ptz_ros8=True,
        ),
        SmokeCase(
            "payload_control ptz 角速度 + zoom（非零 rate）",
            "payload_control",
            {
                "payload_type": 1,
                "control_params": {
                    "roll_rate": 0.0,
                    "pitch_rate": 0.5,
                    "yaw_rate": -0.25,
                    "zoom": 1.2,
                },
            },
            RESULT_SUCCESS,
            verify_ptz_ros8=True,
        ),
        SmokeCase(
            "payload_control video_record on",
            "payload_control",
            {"payload_type": 2, "control_params": {"action": 1}},
            RESULT_SUCCESS,
        ),
        SmokeCase(
            "payload_control video_record off",
            "payload_control",
            {"payload_type": 2, "control_params": {"action": 0}},
            RESULT_SUCCESS,
        ),
        SmokeCase(
            "payload_control lamp on",
            "payload_control",
            {"payload_type": 3, "control_params": {"action": 1}},
            RESULT_SUCCESS,
        ),
        SmokeCase(
            "payload_control lamp off",
            "payload_control",
            {"payload_type": 3, "control_params": {"action": 0}},
            RESULT_SUCCESS,
        ),
        SmokeCase(
            "payload_control megaphone on",
            "payload_control",
            {"payload_type": 4, "control_params": {"action": 1}},
            RESULT_SUCCESS,
        ),
        SmokeCase(
            "payload_control megaphone off",
            "payload_control",
            {"payload_type": 4, "control_params": {"action": 0}},
            RESULT_SUCCESS,
        ),
        SmokeCase(
            "task_control stop",
            "task_control",
            {"task_action": 0},
            RESULT_SUCCESS,
            bid=task_bid,
        ),
    ]

    if include_negative:
        cases.extend(
            [
                SmokeCase("unknown_method", "no_such_method_xyz", {}, RESULT_UNKNOWN),
                SmokeCase("stream_start missing push_url", "stream_start", {}, RESULT_INVALID_PARAM),
                SmokeCase("stream_stop missing stream_id", "stream_stop", {}, RESULT_INVALID_PARAM),
                SmokeCase(
                    "stream_stop empty stream_id",
                    "stream_stop",
                    {"stream_id": ""},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "target_types_config 单元素 [-1] 非法（全检须 MQTT []）",
                    "target_types_config",
                    {"enabled_types": [-1]},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "target_types_config -1 与具体 ID 混用",
                    "target_types_config",
                    {"enabled_types": [-1, 0]},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "target_types_config [-2] 非法（负数超出范围）",
                    "target_types_config",
                    {"enabled_types": [-2]},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "target_types_config [-100] 非法（负数超出范围）",
                    "target_types_config",
                    {"enabled_types": [-100]},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "target_types_config 元素非整数（字符串）",
                    "target_types_config",
                    {"enabled_types": ["invalid"]},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "target_types_config 元素为浮点",
                    "target_types_config",
                    {"enabled_types": [1.5]},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "target_types_config 非数组（字符串）",
                    "target_types_config",
                    {"enabled_types": "not_array"},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "target_types_config id 128 out of 0..127",
                    "target_types_config",
                    {"enabled_types": [128]},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "task_control start without bid",
                    "task_control",
                    {"task_action": 1},
                    RESULT_SUCCESS,
                    bid=None,
                ),
                SmokeCase(
                    "task_control invalid action",
                    "task_control",
                    {"task_action": 2},
                    RESULT_INVALID_PARAM,
                    bid=str(uuid.uuid4()),
                ),
                SmokeCase(
                    "task_action 负数非法",
                    "task_control",
                    {"task_action": -1},
                    RESULT_INVALID_PARAM,
                    bid=str(uuid.uuid4()),
                ),
                SmokeCase(
                    "task_action 非整数非法",
                    "task_control",
                    {"task_action": 1.5},
                    RESULT_INVALID_PARAM,
                    bid=str(uuid.uuid4()),
                ),
                SmokeCase(
                    "task_action 缺失字段",
                    "task_control",
                    {},
                    RESULT_INVALID_PARAM,
                    bid=str(uuid.uuid4()),
                ),
                # 注意：接口文档 §4.2.1 规定"本项目不定义 `true` 的云端开启告警"
                # 所以不再测试 enabled: true 的成功场景
                SmokeCase(
                    "alarm_set missing enabled",
                    "alarm_set",
                    {},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control unsupported payload_type",
                    "payload_control",
                    {"payload_type": 99, "control_params": {}},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control ptz empty control_params",
                    "payload_control",
                    {"payload_type": 1, "control_params": {}},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control ptz angle keys only (reject)",
                    "payload_control",
                    {"payload_type": 1, "control_params": {"roll": 0.1, "pitch": 0.0, "yaw": 0.0}},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control ptz reset:true 与 zoom 混用",
                    "payload_control",
                    {"payload_type": 1, "control_params": {"reset": True, "zoom": 2.0}},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control ptz 禁止 MQTT home 键",
                    "payload_control",
                    {
                        "payload_type": 1,
                        "control_params": {
                            "home": 0.0,
                            "roll_rate": 0.0,
                            "pitch_rate": 0.0,
                            "yaw_rate": 0.0,
                        },
                    },
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control ptz zoom only no rates",
                    "payload_control",
                    {"payload_type": 1, "control_params": {"zoom": 2.0}},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control ptz zoom 非数字",
                    "payload_control",
                    {
                        "payload_type": 1,
                        "control_params": {"roll_rate": 0.0, "pitch_rate": 0.0, "yaw_rate": 0.0, "zoom": "bad"},
                    },
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control ptz 禁止 pos_flag（须用角速度键）",
                    "payload_control",
                    {
                        "payload_type": 1,
                        "control_params": {"pos_flag": 1.0, "pitch_rate": 0.1, "yaw_rate": 0.0, "roll_rate": 0.0},
                    },
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control 缺 control_params",
                    "payload_control",
                    {"payload_type": 1},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control 缺 payload_type",
                    "payload_control",
                    {"control_params": {"action": 1}},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control payload_type=0",
                    "payload_control",
                    {"payload_type": 0, "control_params": {"action": 1}},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control video_record 缺 action",
                    "payload_control",
                    {"payload_type": 2, "control_params": {}},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control lamp 缺 action",
                    "payload_control",
                    {"payload_type": 3, "control_params": {}},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control megaphone 缺 action",
                    "payload_control",
                    {"payload_type": 4, "control_params": {}},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "stream_start push_url 非字符串",
                    "stream_start",
                    {"push_url": 12345},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "alarm_set enabled 非布尔",
                    "alarm_set",
                    {"enabled": 1},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "alarm_set enabled null 非法",
                    "alarm_set",
                    {"enabled": None},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "alarm_set enabled 空字符串非法",
                    "alarm_set",
                    {"enabled": ""},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control video_record action 非法",
                    "payload_control",
                    {"payload_type": 2, "control_params": {"action": 2}},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control lamp action 非法",
                    "payload_control",
                    {"payload_type": 3, "control_params": {"action": -1}},
                    RESULT_INVALID_PARAM,
                ),
                SmokeCase(
                    "payload_control megaphone action 非法",
                    "payload_control",
                    {"payload_type": 4, "control_params": {"action": 99}},
                    RESULT_INVALID_PARAM,
                ),
            ]
        )

    return cases


def main() -> int:
    ap = argparse.ArgumentParser(description="thing_gateway MQTT services 全量冒烟（单脚本）")
    ap.add_argument("-b", "--broker", default="127.0.0.1")
    ap.add_argument("-p", "--port", type=int, default=1883)
    ap.add_argument("-s", "--sn", default="usv0", help="与节点 usv_sn 一致（默认usv0，与C++节点自动生成的SN匹配）")
    ap.add_argument("-u", "--username", default=None)
    ap.add_argument("-P", "--password", default=None)
    ap.add_argument("--sleep", type=float, default=0.25, help="用例间隔（秒）")
    ap.add_argument("--timeout", type=float, default=6.0, help="等待 services_reply 超时（秒）")
    ap.add_argument("--no-negative", action="store_true", help="只跑成功路径")
    ap.add_argument(
        "--verify-ros",
        action="store_true",
        help="对云台用例额外订阅 ptz_ctrl 校验 8 元数组（需 ROS2 + rclpy）",
    )
    ap.add_argument(
        "--ros-topic",
        default="",
        help="与 thing_gateway.ros_topic_ptz_ctrl 一致；--verify-ros 时必填",
    )
    ap.add_argument("--ros-wait", type=float, default=3.0, help="单条云台 ROS 校验超时（秒）")
    args = ap.parse_args()

    ros_sniffer: Optional[RosPtzSniffer] = None
    if args.verify_ros:
        if not args.ros_topic:
            print("错误: --verify-ros 须同时指定 --ros-topic", file=sys.stderr)
            return 2
        try:
            ros_sniffer = RosPtzSniffer(args.ros_topic)
        except Exception as e:
            print(f"错误: 无法初始化 rclpy（是否已 source ROS？）: {e}", file=sys.stderr)
            return 2

    task_bid = str(uuid.uuid4())
    cases = build_cases(task_bid, include_negative=not args.no_negative)

    session = SmokeSession(args.broker, args.port, args.sn, args.username, args.password)
    ok_all = True
    try:
        for c in cases:
            if not run_case(session, c, args.sn, args.timeout, ros_sniffer, args.ros_wait):
                ok_all = False
            time.sleep(args.sleep)
    finally:
        session.close()
        if ros_sniffer is not None:
            ros_sniffer.close()

    if ok_all:
        print(f"\n全部通过（共 {len(cases)} 条）。")
        return 0
    print("\n存在失败用例。", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
