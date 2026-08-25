import socket
import time
from pymavlink.dialects.v20 import common as mavlink2
from pymavlink import mavutil
import src
import time

# def pack_mavlink_message(msg):
#     """ 将 MAVLink 消息对象打包为可发送的字节串 """
#     # 获取消息的字节串
#     #msgbuf = mavutil.mavlink_connection('udpout:0.0.0.0:14892', source_system=1, source_component=0)
#     #return msgbuf
#
#
#
# master = mavutil.mavlink_connection('udpout:0.0.0.0:14892', source_system=1, source_component=0)
# # 初始化MAVLink参数，这些值通常由飞控硬件决定
# system_id = 1  # 通常为1
# component_id = 0  # 通常为1
# target_system = 1
# target_component = 0
#
# # 创建 MAVLink takeoff 消息对象
# # takeoff_msg = mavlink2.MAVLink_command_long_message(
# #     target_system,
# #     target_component,
# #     mavlink2.MAV_CMD_NAV_TAKEOFF,
# #     0,  # Confirmation
# #     0,  # Param1
# #     0,  # Param2
# #     0,  # Param3
# #     0,  # Param4
# #     0,  # Param5
# #     0,  # Param6
# #     10  # Param7 (altitude)
# # )
#
# takeoff_msg_1 = src.MAVLink_uav_mavlink_command_message(
#     target_system,
#     target_component,
#     src.UAV_CMD_LAND
# )
#
#
#
# current_time_usec = int(time.time() * 1e6)
#
# move_msg_1 = src.MAVLink_move_base_goal_message(
#     time_usec=current_time_usec, px=1.2, py=2.2, pz=3.2, row=4.5, pitch=5.6, yaw=6.9
# )
#
# # 打包消息
# # packed_takeoff_msg = pack_mavlink_message(takeoff_msg)
# packed_takeoff_msg_1 = pack_mavlink_message(takeoff_msg_1)
#
# packed_move_msg_1 = pack_mavlink_message(move_msg_1)
# # UDP通信参数
# udp_ip = '127.0.0.1'
# udp_ports = [14890, 14891, 14892, 14893, 14894, 14895, 14896, 14897, 14898, 14899]
#
# # 发送消息
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# while True:
#     for udp_port in udp_ports:
#         target_address = (udp_ip, udp_port)
#         sock.sendto(packed_takeoff_msg_1, target_address)
#         time.sleep(0.1)
#         sock.sendto(packed_move_msg_1, target_address)
# sock.close()
#
# print("Takeoff message sent")


master = mavutil.mavlink_connection("udpout:10.88.105.142:14890", source_system=10, source_component=10)

current_time_usec = int(time.time() * 1e6)

target_system = 3
target_component = 3

move_msg = src.MAVLink_move_base_goal_message(
    time_usec=current_time_usec, px=1.2, py=2.2, pz=3.2,
    row=4.5, pitch=5.6, yaw=6.9
)

takeoff_msg = src.MAVLink_uav_mavlink_command_message(
    src.UAV_CMD_LAND
)

armed_msg = src.MAVLink_armed_signal_message(
    time_usec=current_time_usec,
    armed_value=1,
)

while True:
    master.mav.send(move_msg)
    master.mav.send(takeoff_msg)
    master.mav.send(armed_msg)
    time.sleep(0.1)
    print("Takeoff message sent")
