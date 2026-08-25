#include "mavlink_ros/udp_port.h"



UDP_Port::UDP_Port(boost::asio::io_service &io_service, const char *target_ip_, int rx_port_, int tx_port_, bool debug)
        : socket_(io_service), local_endpoint_(boost::asio::ip::address::from_string("0.0.0.0"), rx_port_),
          buff{}, buff_ptr(0), buff_len(0), debug(debug) {
    socket_.open(boost::asio::ip::udp::v4());
    if (!socket_.is_open()) {
        throw std::runtime_error("Failed to open UDP socket");
    }
    // 设置地址重用选项
    boost::asio::socket_base::reuse_address option(true);
    socket_.set_option(option);

    boost::system::error_code bind_error;
    socket_.bind(local_endpoint_, bind_error);
    if (bind_error) {
        throw std::runtime_error(
                "Failed to bind UDP socket on port " + std::to_string(rx_port_) + ": " + bind_error.message());
    }
    try {
        endpoint_ = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(target_ip_), tx_port_);
    } catch (const std::exception &e) {
        throw std::runtime_error("Invalid target IP address " + std::string(target_ip_) + ": " + e.what());
    }
    start_receive();
}


UDP_Port::~UDP_Port() {
    // 关闭 socket
    if (socket_.is_open()) {
        socket_.close();
        std::cout << "Socket closed and resources released." << std::endl;
    }
}

void UDP_Port::stop() {
    std::lock_guard<std::mutex> lock(read_lock);
    if (!is_closed_ && socket_.is_open()) {        // 检查是否已关闭
        socket_.close();
        is_closed_ = true;                         // 更新关闭状态
        std::cout << "Socket manually closed." << std::endl;
    }
}

void UDP_Port::start_receive() {

    if (is_receiving) return;
    is_receiving = true;
    socket_.async_receive_from(
            boost::asio::buffer(buff), remote_endpoint_,
            [this](boost::system::error_code ec, std::size_t bytes_recvd) {
                std::lock_guard<std::mutex> lock(read_lock); // 锁住共享资源

                if (!ec && bytes_recvd > 0) {
                    if (bytes_recvd > buff.size()) {
                        std::cout << "Received data size " << bytes_recvd << " exceeds buffer size " << buff.size()
                                  << std::endl;
                        return;  // 处理超出大小的情况
                    }
                    buff_len = bytes_recvd; // 设置接收的长度
                    buff_ptr = 0;           // 重置指针以填充新数据
                }
                is_receiving = false; // 重置接收状态
                start_receive();
            });
}

int UDP_Port::_read_port(uint8_t &cp) {
    std::lock_guard<std::mutex> lock(read_lock);
    if (buff_ptr < buff_len) {
        cp = buff[buff_ptr];
        buff_ptr++;
        return 1;
    }
    return 0; // 没有数据可读
}

int UDP_Port::read_message(mavlink_message_t &message) {
    uint8_t cp;
    mavlink_status_t status;
    int msgReceived = -1;

    // 从端口读取单字节数据
    int result = _read_port(cp);

    if (result > 0) {
        // 调用mavlink_parse_char来解析Mavlink消息
        msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

        // 检查丢包
        if (msgReceived && debug) {
            unsigned int i;
            uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
            unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);

            if (messageLength > MAVLINK_MAX_PACKET_LEN) {
                std::cout << "FATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE" << std::endl;
            } else {
                for (i = 0; i < messageLength; i++) {
                    unsigned char v = buffer[i];
                    // fprintf(stderr, "%02x ", v); // 输出调试信息
                }
                // fprintf(stderr, "\n");
            }
        }
    } else {
        //std::cout << "ERROR: Could not read, res = " << result << ", errno = " << errno << std::endl;
    }

    return msgReceived;
}

int UDP_Port::write_message(const mavlink_message_t &message) {
    char buf[300];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t *) buf, &message);
    // 检查是否成功转换消息
    if (len > 0) {
        _write_port(buf, len);
        return len;
    } else {
        std::cerr << "Error: Could not convert MAVLink message to buffer." << std::endl;
        return -1;
    }
}

void UDP_Port::_write_port(char *buf, unsigned len) {
    std::lock_guard<std::mutex> lock(write_lock);

    socket_.async_send_to(
            boost::asio::buffer(buf, len), endpoint_,
            [this](boost::system::error_code ec, std::size_t bytes_sent) {
                if (ec) {
                    std::cerr << "Error sending message: " << ec.message() << std::endl;
                } else {
                    std::cout << "Sent message: " << bytes_sent << " bytes" << std::endl;
                }
                // 这里可以添加更多的处理逻辑，如果需要的话
            });
}



