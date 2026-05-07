#include "CommPort.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <vector>

#include "Checksum.h"
#include "Content.hpp"

CommPort::CommPort(const rclcpp::Node& node, const std::string& port_name, uint32_t baudrate) {
    port_.setPort(port_name);
    port_.setBaudrate(baudrate);
    logger_ = node.get_logger();

    // TODO: scan the ttyACM*(d) if failed to open or segment fault
    // catch is needed
    try {
        port_.open();
    } catch (const serial::IOException& ex) {
        RCLCPP_INFO(logger_, "Failed to open serial port by index %s", port_name.c_str());
        exit(-2);
    } catch (const serial::PortNotOpenedException& ex) {
        RCLCPP_INFO(logger_, "Failed to open serial port by index %s", port_name.c_str());
        exit(-2);
    } catch (const serial::SerialException& ex) {
        RCLCPP_INFO(logger_, "Failed to open serial port by index %s", port_name.c_str());
        exit(-2);
    }

    read_stop_flag_ = false;
    write_stop_flag_ = false;
    write_clear_flag_ = true;
    exception_handled_flag_ = true;

    memset(rx_buffer_, 0, sizeof(rx_buffer_));
}

CommPort::~CommPort() { Stop(); }

void CommPort::Read() {
    static uint32_t e_len_cnt = 0;
    static uint32_t e_eof_cnt = 0;
    constexpr uint8_t kSof = 0x3A;
    constexpr uint8_t kEof = 0xAA;
    constexpr size_t kPacketSize = sizeof(RxContent);

    std::vector<uint8_t> stream_buffer;
    stream_buffer.reserve(kPacketSize * 4);

    while (!read_stop_flag_) {
        try {
            uint8_t read_buffer[256]{};
            const size_t bytes_read = port_.read(read_buffer, sizeof(read_buffer));
            if (bytes_read == 0) {
                continue;
            }

            stream_buffer.insert(stream_buffer.end(), read_buffer, read_buffer + bytes_read);

            while (stream_buffer.size() >= kPacketSize) {
                auto sof = std::find(stream_buffer.begin(), stream_buffer.end(), kSof);
                if (sof == stream_buffer.end()) {
                    stream_buffer.clear();
                    break;
                }

                if (sof != stream_buffer.begin()) {
                    e_len_cnt += 1;
                    stream_buffer.erase(stream_buffer.begin(), sof);
                    std::printf("ERROR OF LEN: %i\n", e_len_cnt);
                }

                if (stream_buffer.size() < kPacketSize) {
                    break;
                }

                if (stream_buffer[kPacketSize - 1] != kEof) {
                    e_eof_cnt += 1;
                    stream_buffer.erase(stream_buffer.begin());
                    std::printf("ERROR OF EOF: %i\n", e_eof_cnt);
                    continue;
                }

                std::memcpy(rx_buffer_, stream_buffer.data(), kPacketSize);
                stream_buffer.erase(stream_buffer.begin(), stream_buffer.begin() + kPacketSize);

                RxHandler();
//                write_clear_flag_ = false;
#ifdef USE_DEBUG_SETTINGS
                auto start = std::chrono::high_resolution_clock::now();
#endif
//                TxHandler();
//                Write(tx_buffer_, sizeof(tx_buffer_), true);
#ifdef USE_DEBUG_SETTINGS
                auto elapsed = std::chrono::high_resolution_clock::now() - start;
                printf(
                    "Motion result: %d, Latency: %ld\n", tx_msg_.tx_content_.found,
                    std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
#endif
                //                write_clear_flag_ = true;
            }
        } catch (const serial::SerialException& ex) {
            while (true) {
                if (exception_handled_flag_) {
                    break;
                }
            }
        } catch (const serial::IOException& ex) {
            while (true) {
                if (exception_handled_flag_) {
                    break;
                }
            }
        } catch (const serial::PortNotOpenedException& ex) {
            while (true) {
                if (exception_handled_flag_) {
                    break;
                }
            }
        }
    }
}

void CommPort::Write(const uint8_t* tx_packet, size_t size, bool safe_write) {
    while (!write_clear_flag_);
    if (safe_write) {
        try {
            port_.write(tx_packet, size);
        } catch (const serial::SerialException& ex) {
            SerialFailsafeCallback(true);
        } catch (const serial::IOException& ex) {
            SerialFailsafeCallback(true);
        }
    } else {
        port_.write(tx_packet, size);
    }
}

// sentry only
void CommPort::RunAsync(SERIAL_MODE mode) {
    if (mode == TX_SYNC) {
        RCLCPP_INFO(logger_, "Serial mode: TX_SYNC");
    } else if (mode == TX_RX_ASYNC) {
        RCLCPP_INFO(logger_, "Serial mode: TX_SYNC & RX_ASYNC");
        std::thread serial_read(&CommPort::Read, this);
        serial_read.detach();
        RCLCPP_INFO(logger_, "Async serial read thread started");
    }
}

void CommPort::Start() {
    std::thread t(&CommPort::Read, this);
    t.detach();
}

void CommPort::Stop() {
    write_stop_flag_ = true;
    read_stop_flag_ = true;
#define MS 1000000
    usleep(MS / 2);
#undef MS
    port_.close();
}

void CommPort::SerialFailsafeCallback(bool reopen) {
    exception_handled_flag_ = false;
    std::string target_device = port_.getPort();
    RCLCPP_INFO(logger_, "IO failed at serial device, retrying");
    bool new_device_found = false;

    // Serial exception handling
    while (true) {
#define MS 1000000
        usleep(MS);
#undef MS
        serial_port_info_ = serial::list_ports();
        for (auto& i : serial_port_info_) {
            if (i.description.find("STMicroelectronics") != std::string::npos) {
                target_device = i.port;
                new_device_found = true;
                break;
            }
        }
        if (new_device_found) {
            break;
        }
    }
    if (reopen) {
        port_.close();
    }
    port_.setPort(target_device);
    port_.open();
    RCLCPP_INFO(logger_, "Serial open failsafe succeeded at port");
    exception_handled_flag_ = true;
}

// rx: receive
// tx: transport
void CommPort::RxHandler() {
    // if (Crc8Verify(rx_buffer_, sizeof(ProjectileRx))) {
    //   memcpy(&rx_struct_, rx_buffer_, sizeof(ProjectileRx));
    // }
    // the infantryDL is not using CRC8 for now
    if (sizeof(RxContent) > sizeof(rx_msg_.rx_buffer_)) {
        return;
    }
    rx_msg_.updateFromBuffer(rx_buffer_);
}

RxMsg& CommPort::rx_msg() { return rx_msg_; }

TxMsg& CommPort::tx_msg() { return tx_msg_; }
