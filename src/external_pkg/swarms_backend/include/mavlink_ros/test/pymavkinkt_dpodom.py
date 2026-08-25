import socket
import time
import src
import array
import struct
from pymavlink import mavutil

# 设置UDP端口和地址
UDP_IP = "10.88.105.142"
UDP_PORT = 13550

# 创建 UDP 套接字
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# 设置所需监听的系统和组件ID
system_id = 1
companion_id = 0

# 创建一个mavlink解析器实例，假设有类似pymavlink的接口
mavlink_parser = src.MAVLink(3, 0)

print("Waiting for data on UDP port {}".format(UDP_PORT))

while True:
    try:
        # 接收数据
        data, addr = sock.recvfrom(1024)  # buffer size is 1024 bytes
        # msgs = mavlink_parser.parse_char(data)
        # 使用src解析数据
        msgs = mavlink_parser.parse_buffer(data)
        for msg in msgs:
            if msg is not None:
                # 检查消息类型和系统ID
                if msg.get_msgId() == src.MAVLINK_MSG_ID_MAVROS_LOCAL_POSITION_ODOM:
                    # 解析odom消息
                    odom = msg
                    print(msg.get_srcSystem())
                    print(msg.get_srcComponent())
                    print(f"Position: x={odom.px}, y={odom.py}, z={odom.pz}")
                    print(f"Angular Velocity: x={odom.angular_x}, y={odom.angular_y}, z={odom.angular_z}")
                    print("==========================================================================")
                elif msg.get_msgId() == src.MAVLINK_MSG_ID_PX4_STATE:
                    px4State = msg
                    print(msg.vehicle_id)
                    print(f"mode: {px4State.mode}")
                    print("-------------------------------------------------------------------------")
                elif msg.get_msgId() == src.MAVLINK_MSG_ID_CAMERA_STATE_DISPLAY:
                    print(msg.get_msgId())
                    print(msg.vehicle_id)
                    print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
                elif msg.get_msgId() == src.MAVLINK_MSG_ID_ROBOTMAJOR_STATE_DISPLAY:
                    print(msg.vehicle_id)
                    print(f"robotMajor_state: {msg.robotMajor_state}")
                    print("****************************************************************************")
                elif msg.get_msgId() == src.MAVLINK_MSG_ID_MAVROS_WORLD_ODOM:
                    print(msg.vehicle_id)
                    print(f" MAVLINK_MSG_ID_MAVROS_WORLD_ODOM Position: x={msg.px}, y={msg.py}, z={msg.pz}")
                    print(f" MAVLINK_MSG_ID_MAVROS_WORLD_ODOM Angular Velocity: x={msg.angular_x}, y={msg.angular_y}, z={msg.angular_z}")
                    print("///////////////////////////////////////////////////////////////////////////")
                elif msg.get_msgId() == src.MAVLINK_MSG_ID_MESSAGE_DISPLAY:
                    print(msg.vehicle_id)
                    print(f" MAVLINK_MSG_ID_MESSAGE_DISPLAY: {msg.message_content}")
                    print("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||")
                elif msg.get_msgId() == src.MAVLINK_MSG_ID_VEHICLE_INFO:
                    print(f" MAVLINK_MSG_ID_VEHICLE_INFO: {msg.uav_ip_address}")
                    print(f" MAVLINK_MSG_ID_VEHICLE_INFO: {msg.uav_udp_port}")
    except Exception as e:
        print(e)
        break

    time.sleep(0.1)  # 简单的延时等待下一条消息
