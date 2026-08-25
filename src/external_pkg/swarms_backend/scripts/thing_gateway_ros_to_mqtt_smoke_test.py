#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
thing_gateway ROS 相关冒烟（单脚本、两阶段可拆）

阶段 A — ROS→MQTT（osd / events）:
  - ptz_status 六元 ×2（第二次在 --osd-gimbal-pad 后）：payload_type=1 仅 roll…wiper + state（无 lamp；第 6 元本地用）
  - 录像 OSD：payload_type=2 仅 attributes.state（1=录，0=停）
  - lamp_status（Int8）→ type=3；megaphone_status → type=4
  - events_bridge（std_msgs/Bool）→ MQTT events，`method`=alarm_state，`data.common.enabled`

阶段 B — MQTT services→ROS:
  - stream_start/stop、label_filter（Int8MultiArray；MQTT [] 全检→ROS [-1]、重复 ID）、云台 reset/角速度、录像/灯/喊话器
  - 负例：zoom 非数字、pos_flag、缺字段、payload_type=0、stream_start/alarm 类型错误、video action 越界等
  - task_control + alarm_set → from_mqtt/alarm_set（std_msgs/Bool，`enabled`）

依赖: pip install paho-mqtt；source ROS2 + rclpy、std_msgs。

重要:
  - **感知话题** 仍由 **`-s` → `/{sn}/perception/...`**。
  - **网关 ROS 两话题**：**`/dev/usv_manager/output/alarm_event`**、**`/dev/usv_manager/input/alarm_set`**；若与运行中节点不一致，请用 **`--events-topic`** / **`--alert-ros-topic`** 覆盖。
  - --osd-settle：相机/灯/喊话器「第二条」osd 前的间隔（需 ≥ 节点对应 osd_throttle_*）。
  - --osd-gimbal-pad：步骤 1 与「第二次云台 OSD」之间垫时，避免 gimbal 节流丢包（默认 2.1s；节点节流为 0 可设 0）。
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
import uuid
from typing import Any, Callable, Dict, List, Optional, Tuple

# 每步发布后 spin 次数，给网关订阅回调时间
ROS_PUMP_ITERS = 8
ROS_PUMP_PERIOD_S = 0.05
# 与网关 thing_json::result_code::kInvalidParam 一致
RESULT_INVALID_PARAM = 1001

# 网关 ROS 两话题（与C++代码一致）：/{sn}/usv_manager/...
# 这些会根据实际SN动态调整，可通过 --events-topic / --alert-ros-topic 覆盖
HARDCODED_GATEWAY_EVENTS_TOPIC_TEMPLATE = "/{sn}/usv_manager/output/alarm_event"
HARDCODED_GATEWAY_ALARM_TOPIC_TEMPLATE = "/{sn}/usv_manager/input/alarm_set"

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("请先安装: pip install paho-mqtt", file=sys.stderr)
    sys.exit(1)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Float32, Float32MultiArray, Int8, Int8MultiArray
except ImportError:
    print("请 source ROS2 工作空间并确保已安装 rclpy、std_msgs", file=sys.stderr)
    sys.exit(1)

# 与 ProductGatewayNode 中 intra_process 下 create_publisher 使用的 QoS 对齐（RELIABLE + VOLATILE + depth10）
_DOWNLINK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def _make_mqtt_client(client_id: str, userdata: dict) -> mqtt.Client:
    try:
        from paho.mqtt.enums import CallbackAPIVersion  # type: ignore

        return mqtt.Client(CallbackAPIVersion.VERSION2, client_id=client_id, userdata=userdata)
    except Exception:
        return mqtt.Client(client_id=client_id, userdata=userdata)


def _on_connect_multi_v1(client: mqtt.Client, userdata: Any, flags: Dict, rc: int) -> None:
    if rc == 0:
        for t in userdata["mqtt_topics"]:
            client.subscribe(t, qos=0)
    else:
        print(f"[MQTT] connect failed rc={rc}", file=sys.stderr)


def _on_connect_multi_v2(client: mqtt.Client, userdata: Any, flags: Any, reason_code: Any, properties: Any = None) -> None:
    if getattr(reason_code, "is_failure", False):
        print(f"[MQTT] connect failed: {reason_code}", file=sys.stderr)
        return
    for t in userdata["mqtt_topics"]:
        client.subscribe(t, qos=0)


def _check_product_event_envelope(parsed: dict, expect_sn: str, expect_enabled: bool) -> bool:
    for k in ("tid", "ts", "method", "device", "data"):
        if k not in parsed:
            return False
    tid = str(parsed.get("tid", ""))
    if len(tid) != 36 or tid.count("-") != 4:
        return False
    if parsed.get("method") != "alarm_state":
        return False
    dev = parsed.get("device")
    if not isinstance(dev, dict) or str(dev.get("sn", "")) != str(expect_sn):
        return False
    data = parsed.get("data")
    if not isinstance(data, dict):
        return False
    common = data.get("common")
    if not isinstance(common, dict):
        return False
    if common.get("enabled") is not expect_enabled:
        return False
    return True


def _on_message_osd_events(client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage) -> None:
    q: queue.Queue = userdata["queue"]
    try:
        text = msg.payload.decode("utf-8")
    except Exception:
        text = str(msg.payload)
    q.put((msg.topic, text))


class MqttDualSniffer:
    def __init__(self, broker: str, port: int, osd_topic: str, events_mqtt_topic: str, username: Optional[str], password: Optional[str]):
        self._osd_topic = osd_topic
        self._events_mqtt_topic = events_mqtt_topic
        self._q: queue.Queue = queue.Queue()
        ud = {"mqtt_topics": [osd_topic, events_mqtt_topic], "queue": self._q}
        self._client = _make_mqtt_client(f"py-ros2mqtt-{uuid.uuid4().hex[:8]}", ud)
        self._client.on_connect = _on_connect_multi_v2 if hasattr(mqtt, "CallbackAPIVersion") else _on_connect_multi_v1  # type: ignore
        self._client.on_message = _on_message_osd_events
        if username:
            self._client.username_pw_set(username, password or "")
        self._client.connect(broker, port, keepalive=60)
        self._client.loop_start()
        time.sleep(0.5)

    def drain(self) -> None:
        while True:
            try:
                self._q.get_nowait()
            except queue.Empty:
                break

    def wait_payload_status(
        self,
        payload_type: int,
        predicate: Callable[[dict], bool],
        timeout: float,
    ) -> Optional[dict]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                topic, text = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            if topic != self._osd_topic:
                continue
            try:
                root = json.loads(text)
            except json.JSONDecodeError:
                continue
            if root.get("method") != "payload_status":
                continue
            com = root.get("data", {}).get("common", {})
            if com.get("payload_type") != payload_type:
                continue
            if predicate(root):
                return root
        return None

    def wait_events_raw(self, timeout: float) -> Optional[str]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                topic, text = self._q.get(timeout=0.2)
            except queue.Empty:
                continue
            if topic != self._events_mqtt_topic:
                continue
            return text
        return None

    def close(self) -> None:
        self._client.loop_stop()
        try:
            self._client.disconnect()
        except Exception:
            pass


def _on_connect_reply_v1(client: mqtt.Client, userdata: Any, flags: Dict, rc: int) -> None:
    if rc == 0:
        client.subscribe(userdata["reply_topic"], qos=1)
    else:
        print(f"[MQTT services] connect failed rc={rc}", file=sys.stderr)


def _on_connect_reply_v2(client: mqtt.Client, userdata: Any, flags: Any, reason_code: Any, properties: Any = None) -> None:
    if getattr(reason_code, "is_failure", False):
        print(f"[MQTT services] connect failed: {reason_code}", file=sys.stderr)
        return
    client.subscribe(userdata["reply_topic"], qos=1)


def _on_message_reply(client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage) -> None:
    try:
        root = json.loads(msg.payload.decode("utf-8"))
    except Exception:
        return
    tid = root.get("tid")
    if not tid:
        return
    lock: threading.Lock = userdata["reply_lock"]
    replies: Dict[str, dict] = userdata["replies"]
    with lock:
        replies[str(tid)] = root


class ServicesMqttClient:
    """仅 services / services_reply，用于阶段 B。"""

    def __init__(self, broker: str, port: int, sn: str, username: Optional[str], password: Optional[str]):
        self.services_topic = f"thing/product/{sn}/services"
        self.reply_topic = f"thing/product/{sn}/services_reply"
        self._reply_lock = threading.Lock()
        self._replies: Dict[str, dict] = {}
        ud = {"reply_topic": self.reply_topic, "reply_lock": self._reply_lock, "replies": self._replies}
        self._client = _make_mqtt_client(f"py-svc-{uuid.uuid4().hex[:8]}", ud)
        self._client.on_connect = _on_connect_reply_v2 if hasattr(mqtt, "CallbackAPIVersion") else _on_connect_reply_v1  # type: ignore
        self._client.on_message = _on_message_reply
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

    def publish_json(self, body: dict) -> None:
        self._client.publish(self.services_topic, json.dumps(body, ensure_ascii=False), qos=1)

    def wait_reply(self, tid: str, timeout: float) -> Optional[dict]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._reply_lock:
                if tid in self._replies:
                    return self._replies.pop(tid)
            time.sleep(0.05)
        return None


def device_obj(sn: str) -> dict:
    return {"sn": sn, "type": 3, "sub_type": 0, "vendor": "test_vendor", "name": "test_name"}


def make_service_body(tid: str, method: str, sn: str, common: dict, bid: Optional[str] = None) -> dict:
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


def expected_ros_ptz8(control_params: dict) -> List[float]:
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


def osd_attrs_match_ptz_status(attrs: dict, arr6: List[float], eps: float = 0.02) -> bool:
    """云台 OSD：state=1 + roll…wiper 与 ROS 六元前 5 项一致；MQTT 不含 lamp 键。"""
    if int(attrs.get("state", -99)) != 1:
        return False
    if "lamp" in attrs:
        return False
    if len(arr6) != 6:
        return False
    keys = ("roll", "pitch", "yaw", "zoom", "wiper")
    for k, val in zip(keys, arr6[:5]):
        try:
            if abs(float(attrs.get(k, 1e9)) - float(val)) > eps:
                return False
        except (TypeError, ValueError):
            return False
    return True


def _parse_osd_common(root: dict) -> dict:
    return root.get("data", {}).get("common", {})


def _pump_node(node: Node, iterations: int = ROS_PUMP_ITERS) -> None:
    for _ in range(iterations):
        rclpy.spin_once(node, timeout_sec=ROS_PUMP_PERIOD_S)


def _wait_osd_ptz6(sniff: MqttDualSniffer, arr6: List[float], wait: float) -> Optional[dict]:
    """等待 payload_type=1 且六元与 arr6 一致。"""

    def pred(root: dict) -> bool:
        attrs = _parse_osd_common(root).get("attributes", {})
        return osd_attrs_match_ptz_status(attrs, arr6)

    return sniff.wait_payload_status(1, pred, wait)


def _wait_osd_payload_int_state(
    sniff: MqttDualSniffer,
    payload_type: int,
    want_state: int,
    wait: float,
) -> Optional[dict]:
    def pred(root: dict, ws: int = want_state) -> bool:
        attrs = _parse_osd_common(root).get("attributes", {})
        try:
            return int(attrs.get("state", -999)) == ws
        except (TypeError, ValueError):
            return False

    return sniff.wait_payload_status(payload_type, pred, wait)


def _drain_q(q: queue.Queue) -> None:
    while True:
        try:
            q.get_nowait()
        except queue.Empty:
            break


def _wait_queue_pred(
    node: Node,
    q: queue.Queue,
    timeout: float,
    pred: Callable[[Any], bool],
) -> Tuple[bool, Any]:
    deadline = time.time() + timeout
    last: Any = None
    while time.time() < deadline:
        try:
            item = q.get(timeout=0.1)
            last = item
            if pred(item):
                return True, item
        except queue.Empty:
            pass
        rclpy.spin_once(node, timeout_sec=0.02)
    return False, last


class RosDownlinkTap:
    """订阅网关会发布的 ROS topic，用于阶段 B。"""

    def __init__(self, node: Node, paths: Dict[str, str]) -> None:
        self._node = node
        self.paths = dict(paths)
        self.q_rtsp: queue.Queue = queue.Queue()
        self.q_label: queue.Queue = queue.Queue()
        self.q_ptz: queue.Queue = queue.Queue()
        self.q_lamp: queue.Queue = queue.Queue()
        self.q_megaphone: queue.Queue = queue.Queue()
        self.q_vrec: queue.Queue = queue.Queue()
        self.q_alert: queue.Queue = queue.Queue()

        qos = _DOWNLINK_QOS
        node.create_subscription(Float32, paths["rtsp"], lambda m: self.q_rtsp.put(float(m.data)), qos)
        node.create_subscription(Int8MultiArray, paths["label"], lambda m: self.q_label.put([int(x) for x in m.data]), qos)
        node.create_subscription(Float32MultiArray, paths["ptz_ctrl"], lambda m: self.q_ptz.put([float(x) for x in m.data]), qos)
        node.create_subscription(Int8, paths["lamp_ctrl"], lambda m: self.q_lamp.put(int(m.data)), qos)
        node.create_subscription(Int8, paths["megaphone_ctrl"], lambda m: self.q_megaphone.put(int(m.data)), qos)
        node.create_subscription(Int8, paths["video_ctrl"], lambda m: self.q_vrec.put(int(m.data)), qos)
        node.create_subscription(Bool, paths["alert"], lambda m: self.q_alert.put(bool(m.data)), qos)

    def drain_all(self) -> None:
        for q in (
            self.q_rtsp,
            self.q_label,
            self.q_ptz,
            self.q_lamp,
            self.q_megaphone,
            self.q_vrec,
            self.q_alert,
        ):
            _drain_q(q)


def run_phase_uplink(
    args: argparse.Namespace,
    sniff: MqttDualSniffer,
    node: Node,
    pub_ptz,
    pub_vrec,
    pub_lamp,
    pub_megaphone,
    pub_ev,
) -> bool:
    ok = True
    wait = args.wait
    sleep_t = args.sleep
    P = f"/{args.sn.strip().strip('/')}"
    ptz_out = args.ptz_status_topic or f"{P}/perception/output/ptz_status"

    print("======== 阶段 A：ROS → MQTT（osd / events）========")

    # 1) 云台：六元与 osd attributes 逐项一致
    arr = [0.11, 0.22, 0.33, 1.25, 42.0, 0.0]
    sniff.drain()
    m = Float32MultiArray()
    m.data = [float(x) for x in arr]
    pub_ptz.publish(m)
    _pump_node(node)

    got = _wait_osd_ptz6(sniff, arr, wait)
    if got is None:
        print(f"FAIL [A] 云台 osd: {wait}s 内未收到与六元一致的 payload_type=1")
        ok = False
    else:
        print("OK   [A] 云台 osd（roll/pitch/yaw/zoom/wiper + state，无 lamp）")
        print(json.dumps(got, ensure_ascii=False, indent=2))

    time.sleep(sleep_t)

    # 1b) 第二次云台六元：在 --osd-gimbal-pad 垫时后触发，避免与首次共用 gimbal 节流被丢包
    gpad = float(getattr(args, "osd_gimbal_pad", 0.0) or 0.0)
    time.sleep(gpad)
    arr_b = [0.55, 0.66, 0.77, 2.5, 7.0, 0.0]
    sniff.drain()
    mb = Float32MultiArray()
    mb.data = [float(x) for x in arr_b]
    pub_ptz.publish(mb)
    _pump_node(node)
    got_b = _wait_osd_ptz6(sniff, arr_b, wait)
    if got_b is None:
        print(
            f"FAIL [A] 云台 osd 第二次: {wait}s 内未收到与六元一致的 payload_type=1 "
            f"（节点 gimbal 节流>0 时请加大 --osd-gimbal-pad，当前 {gpad}s）"
        )
        ok = False
    else:
        print(f"OK   [A] 云台 osd 第二次（垫时 {gpad}s 后）")

    time.sleep(sleep_t)

    # 2) 录像 on / off（同类 payload_type=2 连续上云受 osd_throttle_camera_sec 限制，默认需间隔数秒）
    settle = float(args.osd_settle)
    for i, (val, want_rec) in enumerate(((1, 1), (0, 0))):
        if i == 1 and settle > 0:
            time.sleep(settle)
        sniff.drain()
        pub_vrec.publish(Int8(data=val))
        _pump_node(node)

        wr = want_rec

        def pred_cam(root: dict, ww: int = wr) -> bool:
            attrs = _parse_osd_common(root).get("attributes", {})
            if "recording" in attrs:
                return False
            try:
                return int(attrs.get("state", -99)) == ww
            except (TypeError, ValueError):
                return False

        got = sniff.wait_payload_status(2, pred_cam, wait)
        tag = f"state(rec)={want_rec}"
        if got is None:
            print(f"FAIL [A] 相机 osd Int8({val})→{tag}")
            ok = False
        else:
            print(f"OK   [A] 相机 osd Int8({val}) → {tag}")

    time.sleep(sleep_t)

    # 3) 接口文档：灯状态仅 payload_type=3（lamp_status），云台 OSD 不上报 lamp

    # 3b) lamp_status Int8 → payload_type=3；ROS 关常用 -1，MQTT state 规范为 0（附录 D.1.1：1/0）
    for i, (ros_i8, want_state) in enumerate(((1, 1), (-1, 0))):
        if i == 1 and settle > 0:
            time.sleep(settle)
        sniff.drain()
        pub_lamp.publish(Int8(data=ros_i8))
        _pump_node(node)

        got = _wait_osd_payload_int_state(sniff, 3, want_state, wait)
        if got is None:
            print(f"FAIL [A] 灯 osd lamp_status Int8({ros_i8}) 未收到 type=3 state={want_state}")
            ok = False
        else:
            print(f"OK   [A] 灯 osd lamp_status Int8({ros_i8}) → type=3 state={want_state}")

    time.sleep(sleep_t)

    # 3c) 喊话器：ROS 1/-1 → MQTT state 1/0（同灯）
    for i, (ros_i8, want_state) in enumerate(((1, 1), (-1, 0))):
        if i == 1 and settle > 0:
            time.sleep(settle)
        sniff.drain()
        pub_megaphone.publish(Int8(data=ros_i8))
        _pump_node(node)

        got = _wait_osd_payload_int_state(sniff, 4, want_state, wait)
        if got is None:
            print(f"FAIL [A] 喊话器 osd Int8({ros_i8}) 未收到 type=4 state={want_state}")
            ok = False
        else:
            print(f"OK   [A] 喊话器 osd Int8({ros_i8}) → type=4 state={want_state}")

    time.sleep(sleep_t)

    # 4) events：ROS Bool → MQTT alarm_state + data.common.enabled
    for en, label in ((True, "true"), (False, "false")):
        sniff.drain()
        pub_ev.publish(Bool(data=en))
        _pump_node(node)
        raw = sniff.wait_events_raw(wait)
        if raw is None:
            print(f"FAIL [A] events enabled={label} 未收到 MQTT")
            ok = False
        else:
            try:
                parsed = json.loads(raw)
            except json.JSONDecodeError:
                parsed = None
            if not isinstance(parsed, dict) or not _check_product_event_envelope(parsed, args.sn, en):
                print(f"FAIL [A] events enabled={label} 信封不符，收到 {raw!r}")
                ok = False
            else:
                print(f"OK   [A] MQTT events alarm_state + enabled={label}")

    print(f"\n阶段 A 结束（ROS 发布侧含 {ptz_out} 等）。\n")
    return ok


def _hint_alarm_topic_mismatch(tap: RosDownlinkTap) -> None:
    sub = tap.paths.get("alert", "?")
    print(f"      当前订阅: {sub}")
    print(
        "      提示: 本脚本默认告警下发为 /{sn}/usv_manager/input/alarm_set"
        "；若节点发布在别的话题，请 `ros2 topic list | grep alarm` 核对，并用 --alert-ros-topic 覆盖。"
    )


def run_phase_downlink(args: argparse.Namespace, mqtt_svc: ServicesMqttClient, tap: RosDownlinkTap, node: Node) -> bool:
    print("======== 阶段 B：MQTT services → ROS 发布 ==========")
    ok = True
    sn = args.sn
    tw = args.svc_timeout
    node_ref = node

    def call_expect_result(common: dict, method: str, expect: int, bid: Optional[str] = None) -> bool:
        tid = str(uuid.uuid4())
        body = make_service_body(tid, method, sn, common, bid)
        mqtt_svc.publish_json(body)
        rep = mqtt_svc.wait_reply(tid, tw)
        if rep is None:
            print(f"FAIL [B] {method}: 超时无 services_reply")
            return False
        try:
            r = int(rep["data"]["common"]["result"])
        except Exception:
            print(f"FAIL [B] {method}: 无 result {rep!r}")
            return False
        if r != expect:
            print(f"FAIL [B] {method}: expect result={expect} got {r}")
            return False
        return True

    # 1) rtsp
    tap.drain_all()
    if not call_expect_result({"push_url": "rtsp://smoke/test"}, "stream_start", 0):
        ok = False
    else:
        good, v = _wait_queue_pred(node_ref, tap.q_rtsp, tw, lambda x: abs(x - 1.0) < 0.05)
        print(f"{'OK  ' if good else 'FAIL'} [B] stream_start → rtsp_flag≈1 ({v!r})")
        ok = good and ok

    tap.drain_all()
    if not call_expect_result({"stream_id": "s_smoke"}, "stream_stop", 0):
        ok = False
    else:
        good, v = _wait_queue_pred(node_ref, tap.q_rtsp, tw, lambda x: abs(x + 1.0) < 0.05)
        print(f"{'OK  ' if good else 'FAIL'} [B] stream_stop → rtsp_flag≈-1 ({v!r})")
        ok = good and ok

    time.sleep(args.sleep)

    # 2) label_filter 单类别 ID
    tap.drain_all()
    if not call_expect_result({"enabled_types": [2]}, "target_types_config", 0):
        ok = False
    else:
        good, v = _wait_queue_pred(node_ref, tap.q_label, tw, lambda a: a == [2])
        print(f"{'OK  ' if good else 'FAIL'} [B] target_types [2] → label_filter Int8 {v!r}")
        ok = good and ok

    tap.drain_all()
    if not call_expect_result({"enabled_types": [0, 1]}, "target_types_config", 0):
        ok = False
    else:
        good, v = _wait_queue_pred(node_ref, tap.q_label, tw, lambda a: a == [0, 1])
        print(f"{'OK  ' if good else 'FAIL'} [B] target_types [0,1] → label_filter {v!r}")
        ok = good and ok

    tap.drain_all()
    if not call_expect_result({"enabled_types": [2, 2]}, "target_types_config", 0):
        ok = False
    else:
        good, v = _wait_queue_pred(node_ref, tap.q_label, tw, lambda a: a == [2, 2])
        print(f"{'OK  ' if good else 'FAIL'} [B] target_types [2,2] → label_filter 原样 {v!r}")
        ok = good and ok

    tap.drain_all()
    if not call_expect_result({"enabled_types": []}, "target_types_config", 0):
        ok = False
    else:
        good, v = _wait_queue_pred(node_ref, tap.q_label, tw, lambda a: a == [-1])
        print(f"{'OK  ' if good else 'FAIL'} [B] target_types [] → label_filter [-1]（全检） {v!r}")
        ok = good and ok

    # 补充：附录B合法类型 [3], [4], [0,1,3,4]
    tap.drain_all()
    if not call_expect_result({"enabled_types": [3]}, "target_types_config", 0):
        ok = False
    else:
        good, v = _wait_queue_pred(node_ref, tap.q_label, tw, lambda a: a == [3])
        print(f"{'OK  ' if good else 'FAIL'} [B] target_types [3] BALLOON → label_filter {v!r}")
        ok = good and ok

    tap.drain_all()
    if not call_expect_result({"enabled_types": [4]}, "target_types_config", 0):
        ok = False
    else:
        good, v = _wait_queue_pred(node_ref, tap.q_label, tw, lambda a: a == [4])
        print(f"{'OK  ' if good else 'FAIL'} [B] target_types [4] TRASH → label_filter {v!r}")
        ok = good and ok

    tap.drain_all()
    if not call_expect_result({"enabled_types": [0, 1, 3, 4]}, "target_types_config", 0):
        ok = False
    else:
        good, v = _wait_queue_pred(node_ref, tap.q_label, tw, lambda a: a == [0, 1, 3, 4])
        print(f"{'OK  ' if good else 'FAIL'} [B] target_types [0,1,3,4] → label_filter {v!r}")
        ok = good and ok

    time.sleep(args.sleep)

    # 2a) enabled_types 非法：不应下发 ROS（队列应无新消息）
    for bad_types, tag in (((-1,), "[-1]"), ((-1, 0), "[-1,0]"), ((128,), "[128]"), ((-2,), "[-2]"), ((-100,), "[-100]")):
        tap.drain_all()
        if call_expect_result({"enabled_types": list(bad_types)}, "target_types_config", RESULT_INVALID_PARAM):
            print(f"OK   [B] target_types {tag} → result={RESULT_INVALID_PARAM}")
        else:
            ok = False
        # 不应因失败请求误发 label：短暂等待后队列仍空或仍为上一次的值
        time.sleep(0.15)
        _pump_node(node_ref)
        try:
            tap.q_label.get_nowait()
            print(f"FAIL [B] target_types {tag} 不应产生 label_filter 消息")
            ok = False
        except queue.Empty:
            pass

    # 非数组元素和非数组测试
    for bad_config, tag in ((["invalid"], "元素非整数"), ([1.5], "元素浮点"), ("not_array", "非数组")):
        tap.drain_all()
        if call_expect_result({"enabled_types": bad_config}, "target_types_config", RESULT_INVALID_PARAM):
            print(f"OK   [B] target_types {tag} → result={RESULT_INVALID_PARAM}")
        else:
            ok = False
        time.sleep(0.15)
        _pump_node(node_ref)
        try:
            tap.q_label.get_nowait()
            print(f"FAIL [B] target_types {tag} 不应产生 label_filter 消息")
            ok = False
        except queue.Empty:
            pass

    time.sleep(args.sleep)

    # 2b) payload_control 云台 reset:true → 8 元 [1,-2,0,0,0,-1,1,-1]（home=1 回中，pos_flag=-2）
    want_home8 = [1.0, -2.0, 0.0, 0.0, 0.0, -1.0, 1.0, -1.0]
    tap.drain_all()
    if not call_expect_result(
        {"payload_type": 1, "control_params": {"reset": True}},
        "payload_control",
        0,
    ):
        ok = False
    else:
        good, v = _wait_queue_pred(
            node_ref,
            tap.q_ptz,
            tw,
            lambda a: len(a) == 8 and all(abs(a[i] - want_home8[i]) < 0.05 for i in range(8)),
        )
        print(f"{'OK  ' if good else 'FAIL'} [B] payload_control reset → ptz_ctrl 8 元 home=1 pos_flag=-2 ({v!r})")
        ok = good and ok

    time.sleep(args.sleep)

    # 3) ptz_ctrl：MQTT 仅角速度键 + zoom/wiper → pos_flag=-2
    cp = {
        "roll_rate": 0.5,
        "pitch_rate": -0.25,
        "yaw_rate": 1.0,
        "zoom": 3.0,
        "wiper": 2.0,
    }
    want8 = expected_ros_ptz8(cp)
    tap.drain_all()
    if not call_expect_result({"payload_type": 1, "control_params": cp}, "payload_control", 0):
        ok = False
    else:
        good, v = _wait_queue_pred(
            node_ref,
            tap.q_ptz,
            tw,
            lambda a: len(a) == 8 and all(abs(a[i] - want8[i]) < 0.05 for i in range(8)),
        )
        print(f"{'OK  ' if good else 'FAIL'} [B] payload_control ptz 角速度+zoom/wiper → ptz_ctrl 8 元 {v!r}")
        ok = good and ok

    time.sleep(args.sleep)

    cp_vel = {"roll_rate": 0.0, "pitch_rate": 0.5, "yaw_rate": -0.25, "zoom": 1.2}
    want_vel = expected_ros_ptz8(cp_vel)
    tap.drain_all()
    if not call_expect_result({"payload_type": 1, "control_params": cp_vel}, "payload_control", 0):
        ok = False
    else:
        good, v = _wait_queue_pred(
            node_ref,
            tap.q_ptz,
            tw,
            lambda a: len(a) == 8 and all(abs(a[i] - want_vel[i]) < 0.05 for i in range(8)),
        )
        print(f"{'OK  ' if good else 'FAIL'} [B] payload_control ptz 角速度+zoom（变体） {v!r}")
        ok = good and ok

    time.sleep(args.sleep)

    # 3x) payload_control 负例：result=1001 且不应下发 ptz_ctrl
    for tag, common in (
        (
            "ptz zoom 非数字",
            {
                "payload_type": 1,
                "control_params": {"roll_rate": 0.0, "pitch_rate": 0.0, "yaw_rate": 0.0, "zoom": "bad"},
            },
        ),
        (
            "ptz 禁止 pos_flag",
            {
                "payload_type": 1,
                "control_params": {"pos_flag": 1.0, "roll_rate": 0.0, "pitch_rate": 0.1, "yaw_rate": 0.0},
            },
        ),
        ("ptz 缺 control_params", {"payload_type": 1}),
        ("缺 payload_type", {"control_params": {"action": 1}}),
        ("payload_type=0", {"payload_type": 0, "control_params": {"action": 1}}),
    ):
        tap.drain_all()
        if not call_expect_result(common, "payload_control", RESULT_INVALID_PARAM):
            ok = False
            print(f"FAIL [B] {tag}: 期望 result={RESULT_INVALID_PARAM}")
        else:
            print(f"OK   [B] {tag} → result={RESULT_INVALID_PARAM}")
        time.sleep(0.12)
        _pump_node(node_ref)
        try:
            stray = tap.q_ptz.get_nowait()
            print(f"FAIL [B] {tag}: 不应产生 ptz_ctrl，收到 {stray!r}")
            ok = False
        except queue.Empty:
            pass

    tap.drain_all()
    if not call_expect_result({"push_url": 999}, "stream_start", RESULT_INVALID_PARAM):
        ok = False
        print(f"FAIL [B] stream_start push_url 非字符串 → result={RESULT_INVALID_PARAM}")
    else:
        print(f"OK   [B] stream_start push_url 非字符串 → result={RESULT_INVALID_PARAM}")

    tap.drain_all()
    if not call_expect_result({"enabled": 1}, "alarm_set", RESULT_INVALID_PARAM):
        ok = False
        print(f"FAIL [B] alarm_set enabled 非布尔 → result={RESULT_INVALID_PARAM}")
    else:
        print(f"OK   [B] alarm_set enabled 非布尔 → result={RESULT_INVALID_PARAM}")

    tap.drain_all()
    if not call_expect_result(
        {"payload_type": 2, "control_params": {"action": 2}},
        "payload_control",
        RESULT_INVALID_PARAM,
    ):
        ok = False
        print(f"FAIL [B] video_record action=2 → result={RESULT_INVALID_PARAM}")
    else:
        print(f"OK   [B] video_record action=2 → result={RESULT_INVALID_PARAM}")
    time.sleep(0.12)
    _pump_node(node_ref)
    try:
        stray_v = tap.q_vrec.get_nowait()
        print(f"FAIL [B] video_record action=2 不应发 ctrl_flag，收到 {stray_v!r}")
        ok = False
    except queue.Empty:
        pass

    time.sleep(args.sleep)

    # 4) video_record on/off
    for action, expect_i8 in ((1, 1), (0, -1)):
        tap.drain_all()
        if not call_expect_result(
            {"payload_type": 2, "control_params": {"action": action}},
            "payload_control",
            0,
        ):
            ok = False
        else:
            good, v = _wait_queue_pred(node_ref, tap.q_vrec, tw, lambda x, e=expect_i8: x == e)
            print(f"{'OK  ' if good else 'FAIL'} [B] video_record action={action} → ctrl Int8 {v!r}")
            ok = good and ok

    time.sleep(args.sleep)

    # 5) lamp on/off
    for action, expect_i8 in ((1, 1), (0, -1)):
        tap.drain_all()
        if not call_expect_result(
            {"payload_type": 3, "control_params": {"action": action}},
            "payload_control",
            0,
        ):
            ok = False
        else:
            good, v = _wait_queue_pred(node_ref, tap.q_lamp, tw, lambda x, e=expect_i8: x == e)
            print(f"{'OK  ' if good else 'FAIL'} [B] lamp action={action} → lamp_ctrl Int8 {v!r}")
            ok = good and ok

    time.sleep(args.sleep)

    # 6) 喊话器 on/off → megaphone_ctrl Int8（与灯相同 action 1/0 → ROS 1/-1）
    for action, expect_i8 in ((1, 1), (0, -1)):
        tap.drain_all()
        if not call_expect_result(
            {"payload_type": 4, "control_params": {"action": action}},
            "payload_control",
            0,
        ):
            ok = False
        else:
            good, v = _wait_queue_pred(node_ref, tap.q_megaphone, tw, lambda x, e=expect_i8: x == e)
            print(f"{'OK  ' if good else 'FAIL'} [B] megaphone action={action} → megaphone_ctrl Int8 {v!r}")
            ok = good and ok

    time.sleep(args.sleep)

    # 6a) task_control 负例测试（负数、非整数、缺失）
    for bad_task, tag in ((-1, "负数-1"), (1.5, "非整数1.5"), (2, "非法值2"), ({}, "缺失字段")):
        tap.drain_all()
        if call_expect_result({"task_action": bad_task} if isinstance(bad_task, int) or isinstance(bad_task, float) else {}, "task_control", RESULT_INVALID_PARAM, bid=str(uuid.uuid4())):
            print(f"OK   [B] task_control {tag} → result={RESULT_INVALID_PARAM}")
        else:
            print(f"FAIL [B] task_control {tag} 期望 result={RESULT_INVALID_PARAM}")
            ok = False
        time.sleep(0.1)
        _pump_node(node_ref)

    # 7) task + alarm_set → Bool（须 task 启动成功后再测告警）
    # 注意：接口文档 §4.2.1 规定"本项目不定义 `true` 的云端开启告警"
    #      此处仅测试文档定义的场景：enabled: false → 关闭告警 → ROS False
    task_bid = str(uuid.uuid4())
    tap.drain_all()
    task_started = call_expect_result({"task_action": 1}, "task_control", 0, bid=task_bid)
    if not task_started:
        ok = False
    else:
        # 测试 enabled: false（文档定义的唯一场景）
        tap.drain_all()
        if not call_expect_result({"enabled": False}, "alarm_set", 0):
            ok = False
        else:
            _pump_node(node_ref, iterations=40)
            good, v = _wait_queue_pred(node_ref, tap.q_alert, tw, lambda b: not bool(b))
            print(f"{'OK  ' if good else 'FAIL'} [B] alarm_set enabled=False → Bool False")
            if good:
                print(f"      data={v!r}")
            if not good:
                _hint_alarm_topic_mismatch(tap)
            ok = good and ok

    # 7a) alarm_set 负例测试（null、空字符串、非布尔、缺失）
    for bad_enabled, tag in ((None, "null"), ("", "空字符串"), (1, "非布尔1"), ("true", "非布尔字符串"), ({}, "缺失字段")):
        tap.drain_all()
        if call_expect_result({"enabled": bad_enabled} if bad_enabled is not None and bad_enabled != {} else {}, "alarm_set", RESULT_INVALID_PARAM):
            print(f"OK   [B] alarm_set {tag} → result={RESULT_INVALID_PARAM}")
        else:
            print(f"FAIL [B] alarm_set {tag} 期望 result={RESULT_INVALID_PARAM}")
            ok = False
        time.sleep(0.1)
        _pump_node(node_ref)

    tap.drain_all()
    if not call_expect_result({"task_action": 0}, "task_control", 0, bid=task_bid):
        ok = False

    print("\n阶段 B 结束。\n")
    return ok


def build_paths(args: argparse.Namespace) -> Tuple[Dict[str, str], Dict[str, str], str]:
    P = f"/{args.sn.strip().strip('/')}"

    events_bridge = args.events_topic.strip() or HARDCODED_GATEWAY_EVENTS_TOPIC_TEMPLATE.replace("{sn}", args.sn.strip().strip('/'))
    alert_ros = args.alert_ros_topic.strip() or HARDCODED_GATEWAY_ALARM_TOPIC_TEMPLATE.replace("{sn}", args.sn.strip().strip('/'))

    uplink = {
        "ptz_status": args.ptz_status_topic or f"{P}/perception/output/ptz_status",
        "video_state": args.video_state_topic or f"{P}/perception/output/video_record/state",
        "lamp_status": args.lamp_status_topic or f"{P}/perception/output/lamp_status",
        "megaphone_status": args.megaphone_status_topic.strip()
        or f"{P}/usv_manager/output/megaphone_status",
        "events_bridge": events_bridge,
    }

    downlink = {
        "rtsp": args.rtsp_topic.strip() or f"{P}/perception/input/rtsp_flag",
        "label": args.label_topic.strip() or f"{P}/perception/input/label_filter",
        "ptz_ctrl": args.ptz_ctrl_topic.strip() or f"{P}/perception/input/ptz_ctrl",
        "lamp_ctrl": args.lamp_ctrl_topic.strip() or f"{P}/perception/input/lamp_ctrl",
        "megaphone_ctrl": args.megaphone_ctrl_topic.strip() or f"{P}/usv_manager/output/megaphone_ctrl",
        "video_ctrl": args.video_ctrl_topic.strip() or f"{P}/perception/input/video_record/ctrl_flag",
        "alert": alert_ros,
    }
    gw_note = f"{events_bridge} ； {alert_ros}"
    return uplink, downlink, gw_note


def main() -> int:
    ap = argparse.ArgumentParser(description="thing_gateway ROS 双向冒烟（上行 osd/events + 下行 ROS 发布）")
    ap.add_argument("-b", "--broker", default="127.0.0.1")
    ap.add_argument("-p", "--port", type=int, default=1883)
    ap.add_argument("-s", "--sn", default="usv0", help="设备 SN（与 thing_gateway usv_sn 一致；默认usv0，与C++节点自动生成的SN匹配；默认 ROS 感知话题前缀 /{sn}/…）")
    ap.add_argument("-u", "--username", default=None)
    ap.add_argument("-P", "--password", default=None)
    ap.add_argument("--only-uplink", action="store_true", help="只跑阶段 A")
    ap.add_argument("--only-downlink", action="store_true", help="只跑阶段 B")
    ap.add_argument("--wait", type=float, default=12.0, help="阶段 A 等 osd/events 超时（秒）")
    ap.add_argument("--svc-timeout", type=float, default=8.0, help="阶段 B 等 services_reply / ROS 超时（秒）")
    ap.add_argument("--sleep", type=float, default=0.4)
    ap.add_argument(
        "--osd-settle",
        type=float,
        default=5.5,
        help="阶段 A：相机/灯/喊话器第二条 osd 前额外等待秒数，需大于节点 osd_throttle_camera_sec/lamp_sec（默认 5）；节点节流为 0 时可设 0",
    )
    ap.add_argument(
        "--osd-gimbal-pad",
        type=float,
        default=2.1,
        help="阶段 A：步骤 3 发 ptz 前额外等待秒数，避免与步骤 1 共用 gimbal 节流时丢包（默认略大于常见 osd_throttle_gimbal_sec=2；节点 gimbal 节流为 0 时可设 0）",
    )
    # uplink overrides（默认 /{sn}/usv_manager/output/alarm_event）
    ap.add_argument("--events-topic", default="", help="覆盖默认 /{sn}/usv_manager/output/alarm_event")
    ap.add_argument("--ptz-status-topic", default="")
    ap.add_argument("--video-state-topic", default="")
    ap.add_argument("--lamp-status-topic", default="")
    ap.add_argument("--megaphone-status-topic", default="")
    # downlink overrides
    ap.add_argument("--rtsp-topic", default="")
    ap.add_argument("--label-topic", default="")
    ap.add_argument("--ptz-ctrl-topic", default="")
    ap.add_argument("--lamp-ctrl-topic", default="")
    ap.add_argument("--megaphone-ctrl-topic", default="")
    ap.add_argument("--video-ctrl-topic", default="")
    ap.add_argument("--alert-ros-topic", default="", help="覆盖默认 /{sn}/usv_manager/input/alarm_set")
    args = ap.parse_args()

    if args.only_uplink and args.only_downlink:
        print("不能同时指定 --only-uplink 与 --only-downlink", file=sys.stderr)
        return 2

    uplink_paths, downlink_paths, gw_note = build_paths(args)
    osd_mqtt = f"thing/product/{args.sn}/osd"
    events_mqtt = f"thing/product/{args.sn}/events"

    print("路径摘要:")
    print("  [网关 ROS]", gw_note)
    print("  [A 发布] ", json.dumps(uplink_paths, ensure_ascii=False, indent=4))
    print("  [B 订阅] ", json.dumps(downlink_paths, ensure_ascii=False, indent=4))
    print("  [A 订阅 MQTT]", osd_mqtt, events_mqtt)
    print()

    sniff: Optional[MqttDualSniffer] = None
    mqtt_svc: Optional[ServicesMqttClient] = None
    ok_uplink = True
    ok_down = True

    rclpy.init(args=None)
    node = Node(f"thing_gateway_ros_smoke_{uuid.uuid4().hex[:6]}")

    pub_ptz = node.create_publisher(Float32MultiArray, uplink_paths["ptz_status"], 10)
    pub_vrec = node.create_publisher(Int8, uplink_paths["video_state"], 10)
    pub_lamp = node.create_publisher(Int8, uplink_paths["lamp_status"], 10)
    pub_megaphone = node.create_publisher(Int8, uplink_paths["megaphone_status"], 10)
    pub_ev = node.create_publisher(Bool, uplink_paths["events_bridge"], 10)
    tap = RosDownlinkTap(node, downlink_paths)

    try:
        if not args.only_downlink:
            sniff = MqttDualSniffer(args.broker, args.port, osd_mqtt, events_mqtt, args.username, args.password)
            ok_uplink = run_phase_uplink(args, sniff, node, pub_ptz, pub_vrec, pub_lamp, pub_megaphone, pub_ev)

        if not args.only_uplink:
            mqtt_svc = ServicesMqttClient(args.broker, args.port, args.sn, args.username, args.password)
            ok_down = run_phase_downlink(args, mqtt_svc, tap, node)

    finally:
        if sniff is not None:
            sniff.close()
        if mqtt_svc is not None:
            mqtt_svc.close()
        node.destroy_node()
        rclpy.shutdown()

    all_ok = ok_uplink and ok_down
    if all_ok:
        print("全部阶段通过。")
        return 0
    print("存在失败阶段（A 上行 / B 下行）。", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
