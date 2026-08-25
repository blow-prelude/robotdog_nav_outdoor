from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def calculate_identification(context):
    """只计算 namespace 和 curr_drone_id"""
    usv_sn = context.launch_configurations.get('usv_sn', 'dev')
    vehicle_id = int(context.launch_configurations.get('vehicle_id', '1'))
    prefix = context.launch_configurations.get('prefix', 'usv')

    # 计算逻辑
    if usv_sn and usv_sn != 'dev':
        # 设备序列号模式：ROS 话题加前缀使其合法
        final_namespace = prefix + usv_sn.replace('-', '_')
        final_curr_drone_id = '/' + prefix + usv_sn.replace('-', '_')
        print(f"[launch] Device SN mode: {usv_sn}, ROS topics: {final_curr_drone_id}/..., MQTT SN: {usv_sn}")
    else:
        # 传统模式：使用 vehicle_id
        final_namespace = f'{prefix}{vehicle_id}'
        final_curr_drone_id = f'/{prefix}{vehicle_id}'
        print(f"[launch] Traditional mode: vehicle_id={vehicle_id}, ROS topics: {final_curr_drone_id}/...")

    return final_namespace, final_curr_drone_id


def launch_setup(context):
    """启动节点"""
    # 计算 namespace 和 curr_drone_id
    namespace, curr_drone_id = calculate_identification(context)

    # 返回节点，其他参数使用 LaunchConfiguration
    return [
        Node(
            package='backend_nodes',
            executable='backend_nodes_launcher',
            namespace=namespace,
            parameters=[{
                "prefix": LaunchConfiguration('prefix', default='usv'),
                "pub_rate": LaunchConfiguration('pub_rate', default='5.0'),
                'curr_drone_id': curr_drone_id,
                'address': LaunchConfiguration('address', default='tcp://127.0.0.1:1883'),
                'mqtt_username': LaunchConfiguration('mqtt_username', default='admin'),
                'mqtt_passwd': LaunchConfiguration('mqtt_passwd', default='nanhu@2020'),
                'vehicle_id': LaunchConfiguration('vehicle_id', default='1'),
                'mqtt_timeout': LaunchConfiguration('mqtt_timeout', default='60'),
                'mqtt_keep_alive_interval': LaunchConfiguration('mqtt_keep_alive_interval', default='100'),
                'mqtt_buffer_size': LaunchConfiguration('mqtt_buffer_size', default='62914560'),
                'mqtt_max_inflight': LaunchConfiguration('mqtt_max_inflight', default='100'),
                'is_summary_send': LaunchConfiguration('is_summary_send', default='true'),

                'udp_remote_url': LaunchConfiguration('udp_remote_url', default='10.88.105.142:13550'),
                'serial_url': LaunchConfiguration('serial_url', default='/dev/ttyUSB0'),
                'udp_local_port': LaunchConfiguration('udp_local_port', default='14890'),
                'serial_baudrate': LaunchConfiguration('serial_baudrate', default='115200'),
                'udp_use': LaunchConfiguration('udp_use', default='true'),
                'debug': LaunchConfiguration('debug', default='false'),

                'usv_sn': str(context.launch_configurations.get('usv_sn', 'dev')),  # 确保字符串类型
                'usv_sn_config_file': LaunchConfiguration('usv_sn_config_file', default=''),
                'device_name': LaunchConfiguration('device_name', default=''),
                'device_vendor': LaunchConfiguration('device_vendor', default=''),
                'device_type': LaunchConfiguration('device_type', default='3'),
                'device_sub_type': LaunchConfiguration('device_sub_type', default='0'),
                'thing_gateway.relax_device_sn': LaunchConfiguration('thing_gateway_relax_device_sn', default='false'),
            }],
            output='screen'
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('usv_sn', default_value='dev',
                              description='Device serial number (use vehicle-based if "dev")'),
        DeclareLaunchArgument('vehicle_id', default_value='1'),
        DeclareLaunchArgument('prefix', default_value='usv'),
        DeclareLaunchArgument('address', default_value='tcp://127.0.0.1:1883'),
        DeclareLaunchArgument('mqtt_username', default_value='admin'),
        DeclareLaunchArgument('mqtt_passwd', default_value='nanhu@2020'),
        DeclareLaunchArgument('pub_rate', default_value='5.0'),
        DeclareLaunchArgument('mqtt_timeout', default_value='60'),
        DeclareLaunchArgument('mqtt_keep_alive_interval', default_value='100'),
        DeclareLaunchArgument('mqtt_buffer_size', default_value='62914560'),
        DeclareLaunchArgument('mqtt_max_inflight', default_value='100'),
        DeclareLaunchArgument('is_summary_send', default_value='true'),
        DeclareLaunchArgument('udp_remote_url', default_value='10.88.105.142:13550'),
        DeclareLaunchArgument('serial_url', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('udp_local_port', default_value='14890'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('udp_use', default_value='true'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('usv_sn_config_file', default_value=''),
        DeclareLaunchArgument('device_name', default_value=''),
        DeclareLaunchArgument('device_vendor', default_value=''),
        DeclareLaunchArgument('device_type', default_value='3'),
        DeclareLaunchArgument('device_sub_type', default_value='0'),
        DeclareLaunchArgument('thing_gateway_relax_device_sn', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
