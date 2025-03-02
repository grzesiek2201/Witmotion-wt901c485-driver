#ifndef WITMOTION_DRIVER
#define WITMOTION_DRIVER


#include <iostream>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <unordered_map>
#include <chrono>
#include <thread>
#include <vector>
#include <iterator>
// #include <algorithm>
// #include <unistd.h> // usleep
#include <sstream>
#include <math.h>


class Driver {
    public:
        Driver();
        ~Driver();
    
        void send_read_command(LibSerial::SerialPort &serial_port, const uint16_t device_addr);
        void read_data(LibSerial::SerialPort &serial_port, const uint16_t ADDR);
        void sendData(LibSerial::SerialPort &serial_port, std::vector<uint8_t> &data);
        void readReg(LibSerial::SerialPort &serial_port, const uint16_t ADDR, const uint16_t regAddr, const uint16_t regCount);
        std::vector<uint8_t> read_register(LibSerial::SerialPort &serial_port, const uint16_t device_addr, const uint16_t reg_addr, const uint16_t reg_count = 1);
        std::unordered_map<std::string, std::string> read_config(LibSerial::SerialPort &serial_port, const uint16_t ADDR, std::string register_name);
        void configure_device(std::string port, uint16_t baudrate, uint16_t bandwidth, uint16_t axis, uint16_t calsw);

    private:
        LibSerial::SerialPort      serial_port;


        std::vector<uint8_t> get_readBytes(const uint8_t devid, const uint16_t regAddr, const uint16_t regCount);
        uint16_t get_crc(const std::vector<uint8_t> &datas, const int dlen);
        std::vector<uint16_t> extract_register_data(const std::vector<uint8_t>& data, int n_registers);
        int16_t getSignInt16(uint16_t num);
        std::unordered_map<std::string, std::string> parse_baudrate_register(const std::vector<uint8_t>& data);
        std::unordered_map<std::string, std::string> parse_bandwidth_register(const std::vector<uint8_t>& data);
        std::unordered_map<std::string, std::string> parse_axis_register(const std::vector<uint8_t>& data);
        std::unordered_map<std::string, std::string> parse_calsw_register(const std::vector<uint8_t>& data);
        std::vector<uint8_t> read_register(LibSerial::SerialPort &serial_port, const uint16_t device_addr, const uint16_t reg_addr, const uint16_t reg_count = 1);
        std::unordered_map<std::string, double> processData(const std::vector<uint8_t>& bytes, const int length);

};


#endif