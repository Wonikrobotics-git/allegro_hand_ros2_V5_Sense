/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Wonik Robotics.
 *  All rights reserved.
 */

/*
 *  @file socket_udp.cpp
 *  @brief UDP-only transport: commands, telemetry, and torque on one socket (port 7000).
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <chrono>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <rclcpp/rclcpp.hpp>

#include "tcpdef.h"
#include "tcpdrv.h"

namespace TCPAPI {

#define LOG_TAG_UDP "allegro_udp_drv"

static int udp_socket_ = -1;
static uint32_t udp_tx_seq_ = 0;
static struct sockaddr_in peer_addr_;

static uint32_t timestampMs()
{
    using namespace std::chrono;
    return static_cast<uint32_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

int udp_open(const char* fw_ip, int peer_port, int local_bind_port)
{
    if (udp_socket_ >= 0) {
        udp_close();
    }

    udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG_UDP),
            "UDP: socket: %s", strerror(errno));
        return -1;
    }

    int reuse = 1;
    setsockopt(udp_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_addr.s_addr = INADDR_ANY;
    bind_addr.sin_port = htons(static_cast<uint16_t>(local_bind_port));

    if (bind(udp_socket_, reinterpret_cast<struct sockaddr*>(&bind_addr),
             sizeof(bind_addr)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG_UDP),
            "UDP: bind port %d: %s", local_bind_port, strerror(errno));
        close(udp_socket_);
        udp_socket_ = -1;
        return -1;
    }

    memset(&peer_addr_, 0, sizeof(peer_addr_));
    peer_addr_.sin_family = AF_INET;
    peer_addr_.sin_port = htons(static_cast<uint16_t>(peer_port));
    if (inet_pton(AF_INET, fw_ip, &peer_addr_.sin_addr) <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG_UDP),
            "UDP: bad firmware IP: %s", fw_ip);
        close(udp_socket_);
        udp_socket_ = -1;
        return -1;
    }

    int flags = fcntl(udp_socket_, F_GETFL, 0);
    if (flags >= 0) {
      fcntl(udp_socket_, F_SETFL, flags | O_NONBLOCK);
    }

    udp_tx_seq_ = 0;
    RCLCPP_INFO(rclcpp::get_logger(LOG_TAG_UDP),
        "UDP: bound local :%d, peer %s:%d", local_bind_port, fw_ip, peer_port);
    return 0;
}

int udp_close()
{
    if (udp_socket_ >= 0) {
        close(udp_socket_);
        udp_socket_ = -1;
        RCLCPP_INFO(rclcpp::get_logger(LOG_TAG_UDP), "UDP: closed");
    }
    return 0;
}

bool udp_is_open()
{
    return udp_socket_ >= 0;
}

int udp_wait_readable(int timeout_ms)
{
    if (udp_socket_ < 0) {
        return -1;
    }
    struct pollfd pfd;
    pfd.fd = udp_socket_;
    pfd.events = POLLIN;
    pfd.revents = 0;
    int pr = ::poll(&pfd, 1, timeout_ms);
    if (pr < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG_UDP),
            "UDP: poll: %s", strerror(errno));
        return -1;
    }
    if (pr == 0) {
        return 0;
    }
    if ((pfd.revents & (POLLERR | POLLNVAL)) != 0) {
        return -1;
    }
    return 1;
}

int udp_send_packet(uint8_t type, const void* payload, uint16_t length,
                    uint32_t* out_seq)
{
    if (udp_socket_ < 0) {
        return -1;
    }

    PacketHeader header;
    header.magic = PROTO_MAGIC;
    header.version = PROTO_VERSION;
    header.type = type;
    header.length = length;
    header.seq = udp_tx_seq_++;
    header.timestamp_ms = timestampMs();

    if (out_seq != nullptr) {
        *out_seq = header.seq;
    }

    uint8_t buf[HEADER_SIZE + 256];
    if (HEADER_SIZE + length > static_cast<int>(sizeof(buf))) {
        return -1;
    }
    memcpy(buf, &header, HEADER_SIZE);
    if (length > 0 && payload != nullptr) {
        memcpy(buf + HEADER_SIZE, payload, length);
    }

    ssize_t n = sendto(udp_socket_, buf, HEADER_SIZE + length, 0,
        reinterpret_cast<struct sockaddr*>(&peer_addr_), sizeof(peer_addr_));
    if (n < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG_UDP),
            "UDP: sendto failed: %s", strerror(errno));
        return -1;
    }
    return 0;
}

int udp_send_torque_cmd(const TorqueCmd* cmd)
{
    return udp_send_packet(TYPE_TORQUE_CMD, cmd, TORQUE_CMD_PAYLOAD, nullptr);
}

int udp_send_hello_req(uint32_t* out_seq)
{
    return udp_send_packet(TYPE_HELLO_REQ, nullptr, 0, out_seq);
}

int udp_send_header_only(uint8_t type, uint32_t* out_seq)
{
    return udp_send_packet(type, nullptr, 0, out_seq);
}

int udp_recv_packet(PacketHeader* header, uint8_t* payload, int max_payload)
{
    if (udp_socket_ < 0) {
        return -2;
    }

    uint8_t buf[2048];
    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);

    ssize_t n = recvfrom(udp_socket_, buf, sizeof(buf), MSG_DONTWAIT,
        reinterpret_cast<struct sockaddr*>(&from), &fromlen);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return -1;
        }
        RCLCPP_ERROR(rclcpp::get_logger(LOG_TAG_UDP),
            "UDP: recvfrom: %s", strerror(errno));
        return -2;
    }
    if (n < HEADER_SIZE) {
        return -1;
    }

    memcpy(header, buf, HEADER_SIZE);
    if (header->magic != PROTO_MAGIC || header->version != PROTO_VERSION) {
        return -1;
    }
    if (header->length > max_payload ||
        n < static_cast<ssize_t>(HEADER_SIZE + header->length)) {
        return -1;
    }
    if (header->length > 0) {
        memcpy(payload, buf + HEADER_SIZE, header->length);
    }
    return 0;
}

}  /* namespace TCPAPI */
