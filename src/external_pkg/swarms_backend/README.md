
# 无人机集群指控后端项目(XZ指控后端项目) [ROS2 [ foxy (ubuntu20.04) ] 版本]

## 项目地址
`https://git.cnaeit.com/XJSF/software_group/amg0523062/swarms_backend`

## 环境安装

### mqtt环境

#### emqx(版本5.0.26) deb包安装
    sudo dpkg -i emqx-5.0.26-ubuntu20.04-arm64.deb (nx上是arm版本，ubuntu上是amd版本)
    systemctl start emqx(开机启动)
    systemctl status emqx（查看当前状态）

#### paho.mqtt.c(版本1.3.13)[针对MQTT V5.0]
    git clone https://github.com/eclipse/paho.mqtt.c.git
    cd paho.mqtt.c && mkdir build
    
    cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON \
    -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON
    sudo cmake --build build/ --target install
    sudo ldconfig

#### paho.mqtt.cpp(版本1.3.2)[针对MQTT V5.0]
    git clone https://github.com/eclipse/paho.mqtt.cpp
    cd paho.mqtt.cpp && mkdir build
    
    cmake -Bbuild -H. -DPAHO_BUILD_STATIC=ON \
    -DPAHO_BUILD_DOCUMENTATION=FALSE -DPAHO_BUILD_SAMPLES=TRUE
    sudo cmake --build build/ --target install
    sudo ldconfig

### jsoncpp(版本1.9.5)环境（非必要的环境，ubuntu系统自带1.7.4版本的jsoncpp）

    git clone https://github.com/open-source-parsers/jsoncpp.git  
    cd jsoncpp && mkdir build 
    cd build && cmake ..  
    sudo make && sudo make install  
    sudo ldconfig

### yaml-cpp(版本0.6.3)环境 (非必要的环境，ros的rviz会依赖0.6.2版本的yaml)

    源码安装0.6.3版本命令：
        网页下载的源码地址（不要使用git）：https://github.com/jbeder/yaml-cpp/releases/tag/yaml-cpp-0.6.3
        cd yaml-cpp-yaml-cpp-0.6.3
        mkdir build
        cd build
        cmake .. -DYAML_BUILD_SHARED_LIBS=ON （on代表着编译为动态库）
        make
        sudo make install
        sudo ldconfig

## 快速开始
1. 进行环境安装  
   因为本项目为ROS2项目，因此需要首先安装ROS2和MAVROS2
2. 从项目地址下载本项目后, 再从 `https://git.cnaeit.com/XJSF/product_group/core-library/msg_set_msgs` 下载最新的 `msg_set_msgs`, 将该 `msg_set_msgs` 放在和 `mqttnode` 同级的目录下
3. 按照下面步骤build并启动本项目
   * DEBUG 模式 (目前XZ通过bash脚本启动后端程序时默认使用DEBUG模式, 单独运行各package)
       ```bash
       # 首先确保当前环境为 foxy ros 环境
       # 设置环境变量 CMAKE_BUILD_TYPE 
       # DEBUG 模式 使用.launch.py文件启动,适合调试使用
       export CMAKE_BUILD_TYPE=DEBUG # 如果不执行该命令，默认为DEBUG模式!!!

       # 编译安装 package (backend_ros_foxy, mqttnode_foxy, msg_set_msgs)
       cd mqtt_ws
       colcon build --symlink-install

       # source 环境
       source install/setup.bash

       # 运行 package (同时运行 backend_ros_foxy, mqttnode_foxy)
       # 常用参数包括:
       # vehicle_id: 无人机ID, 默认为 0
       # address: mqtt 服务器地址, 默认为 tcp://127.0.0.1:1883
       # mqtt_buffer_size: mqtt 缓存大小, 默认为 62914560 (60MB), 可以修改为31457280 (30MB)
       # is_summary_send: 是否发送汇总消息, false时表示分别发送无人机各个状态消息, true时表示发送汇总消息, 不发送无人机各个状态消息, 默认为 true
       # prefix: ros消息前缀, 默认为 drone_, 一般不改
       # pub_rate: 无人机状态消息发布频率, 默认为 5.0(HZ), 一般不改
       # mqtt_username: mqtt 服务用户名, 默认为 admin, 一般不改
       # mqtt_passwd: mqtt 服务密码, 默认为 nanhu@2020, 一般不改
       # mqtt_timeout: mqtt 连接broker超时时间, 默认为 60 (单位为秒), 一般不改
       # mqtt_keep_alive_interval: mqtt 与 broker 心跳间隔, broker 在该时长的 1.5 倍时间内没有收到来自客户端的 MQTT 控制报文，关闭与客户端的网络连接, 默认为 100 (单位为秒), 一般不改
       ros2 launch mqttnode_foxy backend_nodes.launch.py vehicle_id:=2 address:=tcp://192.168.1.100:1883 mqtt_buffer_size:=31457280 is_summary_send:=true

       # 单独运行 backend_ros_foxy (纯ROS2节点, 不包含mqtt)
       # 常用参数包括:
       # vehicle_id: 无人机ID, 默认为 0
       # prefix: ros消息前缀, 默认为 drone_, 一般不改
       # pub_rate: 无人机状态消息发布频率, 默认为 5.0(HZ), 一般不改
       ros2 launch mqttnode_foxy backend_ros.launch.py vehicle_id:=2

       # 单独运行 mqttnode_foxy (ROS2节点, 包含mqtt)
       # 常用参数包括:
       # vehicle_id: 无人机ID, 默认为 0
       # address: mqtt服务器地址, 默认为 tcp://127.0.0.1:1883
       # mqtt_buffer_size: mqtt 缓存大小, 默认为 62914560 (60MB), 可以修改为31457280 (30MB)
       # is_summary_send: 是否发送汇总消息, false时表示分别发送无人机各个状态消息, true时表示发送汇总消息, 不发送无人机各个状态消息, 默认为 true
       # prefix: ros消息前缀, 默认为 drone_, 一般不改
       # mqtt_username: mqtt 服务用户名, 默认为 admin, 一般不改
       # mqtt_passwd: mqtt 服务密码, 默认为 nanhu@2020, 一般不改
       # mqtt_timeout: mqtt 连接broker超时时间, 默认为 60 (单位为秒), 一般不改
       # mqtt_keep_alive_interval: mqtt 与 broker 心跳间隔, broker 在该时长的 1.5 倍时间内没有收到来自客户端的 MQTT 控制报文，关闭与客户端的网络连接, 默认为 100 (单位为秒), 一般不改
       ros2 launch mqttnode_foxy mqttnode.launch.py vehicle_id:=2 address:=tcp://192.168.1.100:1883 mqtt_buffer_size:=31457280 is_summary_send:=true
       ```
   * RELEASE 模式
       ```bash
       # 首先确保当前环境为 foxy ros 环境
       # 设置环境变量 CMAKE_BUILD_TYPE 
       # RELEASE 模式 使用可执行文件+参数yaml文件启动,在部署时使用
       export CMAKE_BUILD_TYPE=RELEASE

       # 编译安装 package (backend_ros_foxy, mqttnode_foxy, msg_set_msgs)
       cd mqtt_ws
       colcon build --symlink-install

       # 修改 config/mqttnode.yaml 中的参数取值
       # 将 config/mqttnode.yaml 拷贝到 /etc 目录下
       sudo cp config/mqttnode.yaml /etc

       # 单独运行 backend_ros_foxy
       source install/setup.bash
       cd install/backend_ros_foxy/lib/backend_ros_foxy
       ./backend_ros_foxy_node

       # 单独运行 mqttnode_foxy
       source install/setup.bash
       cd install/mqttnode_foxy/lib/mqttnode_foxy
       ./mqttnode_foxy_node
       ```
   * 调试运行快捷方式(DEBUG 模式)
       ```bash
       cd mqtt_ws/bash

       # 编译项目: 运行 /mqtt/bash/mqttnode_debug_build.sh
       ./mqttnode_debug_build.sh

       # 运行项目: 运行 ./mqttnode_debug_run.sh
       # 参数 $1: vehicle_id 默认为 0
       # 参数 $2: prefix 默认为 drone_
       # 参数 $3: mqtt_buffer_size 默认为 31457280 (30MB)
       # 参数 $4: domain_id 默认为 0
       # 参数 $5: is_summary_send 默认为 true
       ./mqttnode_debug_run.sh
       ```
   * 部署运行快捷方式(RELEASE 模式)
       ```bash
       cd mqtt_ws/bash

       # 编译安装项目: 运行 mqtt/bash/mqttnode_build.sh
       # 参数 $1: 可执行文件安装位置 默认为 '/bin'
       ./mqttnode_build.sh

       # 运行项目: 运行 mqtt/bash/mqttnode_run.sh
       # 参数 $1: 可执行文件安装位置 默认为 '/bin'
       # 参数 $2: package 'msg_set_msgs' setup.bash 所在的位置, 默认为本项目 setup.bash 所在位置( 编译时本项目包含了 msg_set_msgs )
       # 参数 $3: ROS_DOMAIN_ID, 默认为 0
       ./mqttnode_run.sh
       ```
4. 当需要快捷关闭启动的节点时,可以运行 `mqtt_ws/bash/stop.bash`
5. 当需要对项目进行DEBUG时, 可以配置以下两个文件:  
   (1) `.vscode/tasks.json` 配置编译任务  
   (2) `.vscode/launch.json` 配置调试任务 可以更换launch文件进行调试


## 更新日至

### 2024.10.31
<details open>
<summary><b style="font-size: 24px;">v1.0.0</b></summary>
<ul>
<li>接收ros2话题：targetsGps_sub_topic_，控制发送频率，原有基础上添加id字段 </li>
<li>接收ros2话题：attack_confirm_sub_topic_</li>
</ul>
</details>

### 2024.11.4
<details open>
<summary><b style="font-size: 24px;">v1.0.1</b></summary>
<ul>
<li>去掉ros2的工作空间部分，只上传ros2功能包</li>
<li>尝试解决no more message buffer问题</li>

    const auto opts = mqtt::create_options_builder()
            .send_while_disconnected(false, false)
            .max_buffered_messages(100)
            .delete_oldest_messages()
            .mqtt_version(MQTTVERSION_5)
            .finalize();
    client_ = std::make_shared<mqtt::async_client>(mqtt_address_, client_id_, opts);

</ul>
</details>

### 2024.11.5
<details open>
<summary><b style="font-size: 24px;">v1.0.2</b></summary>
<ul>
<li>新增发送云台控制mqtt话题：MqttTopic::mqtt_ptz_control+std::to_string(vehicle_id_)</li>
<li>新增发送无人机是否保存图片mqtt话题： MqttTopic::mqtt_image_get+std::to_string(vehicle_id_)</li>
</ul>
</details>

### 2024.11.7
<details open>
<summary><b style="font-size: 24px;">v1.0.3</b></summary>
<ul>
<li>新增发送多点发布mqtt话题: MqttTopic::mqtt_request_multi_goal + std::to_string(vehicle_id_)</li>
</ul>
</details>


### 2024.11.12
<details open>
<summary><b style="font-size: 24px;">v1.0.4</b></summary>
<ul>
<li>更改项目结构，1个进程跑多个ros2节点</li>
<li>每个节点构造函数中，intra_process_comms值不同，对应的qos策略不同</li>
<li>此版本功能待测试</li>
</ul>
</details>