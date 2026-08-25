/****************************************************************************
 *
 *   Copyright (c) 2018 MAVlink Development Team. All rights reserved.
 *   Author: Hannes Diethelm, <hannes.diethelm@gmail.com>
 *           Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file udp_port.h
 *
 * @brief UDP interface definition
 *
 * Functions for opening, closing, reading and writing via UDP ports
 *
 * @author Hannes Diethelm, <hannes.diethelm@gmail.com>
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */

#ifndef UDP_PORT_H_
#define UDP_PORT_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <cstdlib>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <pthread.h> // This uses POSIX Threads
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <iostream>
#include <mutex>
#include <cstring>
#include "generic_port.h"

class UDP_Port : public Generic_Port {

public:

    UDP_Port();

    UDP_Port(boost::asio::io_service &io_service, const char *target_ip_, int rx_port_, int tx_port_,
             bool deubg_ = false);

    virtual ~UDP_Port();

    int read_message(mavlink_message_t &message);

    int write_message(const mavlink_message_t &message);

    void start_receive();

    bool is_running() {
        return is_open;
    }

    void start() {}

    void stop();

private:

    mavlink_status_t lastStatus;

    //void initialize_defaults();
    bool debug;
    const char *target_ip;
    int rx_port;
    int tx_port;
    bool is_open;
    bool is_closed_ = false;
    bool is_receiving = false;

    boost::asio::ip::udp::endpoint target_endpoint;

    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint endpoint_;
    boost::asio::ip::udp::endpoint local_endpoint_;
    boost::asio::ip::udp::endpoint remote_endpoint_;
    boost::array<uint8_t, 2048> buff;  // 接收缓冲区
    std::size_t buff_ptr;
    std::size_t buff_len;
    std::mutex read_lock;
    std::mutex write_lock;

    //int  _read_port(uint8_t &cp);
    void _write_port(char *buf, unsigned len);

    int _read_port(uint8_t &cp);


};


#endif // UDP_PORT_H_


