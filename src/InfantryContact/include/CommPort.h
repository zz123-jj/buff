#ifndef ROBO_CV_COMMPORT_H
#define ROBO_CV_COMMPORT_H

#include <Checksum.h>
#include <serial/serial.h>
#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <unistd.h>

#include <Content.hpp>
#include <RxMsg.hpp>
#include <TxMsg.hpp>
#include <atomic>
#include <chrono>
#include <thread>

class CommPort {
   private:
    RxMsg rx_msg_{};
    TxMsg tx_msg_{};

    std::atomic<bool> read_stop_flag_{};
    std::atomic<bool> write_stop_flag_{};
    std::atomic<bool> write_clear_flag_{};
    std::atomic<bool> exception_handled_flag_{};

    uint8_t rx_buffer_[sizeof(RxContent)]{};
    serial::Serial port_;

    rclcpp::Logger logger_ = rclcpp::get_logger("CommPort");
    std::vector<serial::PortInfo> serial_port_info_;
    std::string device_desc_;

   public:
    enum SERIAL_MODE { TX_SYNC, TX_RX_ASYNC };

    CommPort(const rclcpp::Node& node, const std::string& port_name = "/dev/ttyACM0",
             uint32_t baudrate = 115200);

    CommPort(const CommPort&) = delete;
    CommPort& operator=(const CommPort&) = delete;

    ~CommPort();

    void RunAsync(SERIAL_MODE mode);

    void Start();

    void Stop();

    void Write(const uint8_t* tx_packet, size_t size, bool safe_write);

    void Read();

    void RxHandler();

    void SerialFailsafeCallback(bool reopen);

    RxMsg& rx_msg();

    TxMsg& tx_msg();
};

#endif  // ROBO_CV_COMMPORT_H
