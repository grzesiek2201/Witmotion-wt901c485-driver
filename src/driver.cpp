#include "driver.hpp"


Driver::Driver() {
}

Driver::~Driver() {
    if (serial_port.IsOpen()) {
        serial_port.Close();
    }
}

void Driver::configure_device(std::string port, uint16_t baudrate, uint16_t bandwidth, uint16_t axis, uint16_t calsw) {
    serial_port.Open(port);
    using namespace LibSerial;
    serial_port.SetBaudRate( BaudRate::BAUD_115200 );
    serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    serial_port.SetParity(Parity::PARITY_NONE);
    serial_port.SetStopBits(StopBits::STOP_BITS_1);
}

void Driver::send_read_command(LibSerial::SerialPort &serial_port, const uint16_t device_addr) {
    while (true) {
        readReg(serial_port, device_addr, 0x34, 13);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void Driver::read_data(LibSerial::SerialPort &serial_port, const uint16_t ADDR) {
    constexpr size_t kTimeoutMs = 1;
    constexpr size_t kHeaderSize = 5;
    constexpr size_t kCrcSize = 2;
    const size_t kMinBufferSize = kHeaderSize + kCrcSize;
    
    std::vector<uint8_t> read_buffer;
    std::vector<uint8_t> buffer;

    while (true) {
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        size_t bytes_available = serial_port.GetNumberOfBytesAvailable();
        if (bytes_available >= 13 * 2 + kHeaderSize) {
            std::cout << "Reading " << bytes_available << " bytes\n";
            serial_port.Read(buffer, bytes_available, kTimeoutMs);

            // Add the byte to read_buffer
            for (auto const &el: buffer) {
                read_buffer.emplace_back(el);
            }
        }

        // If the first byte is not the address, remove it from the buffer
        if (!read_buffer.empty() && read_buffer.front() != ADDR) {
            read_buffer.erase(read_buffer.begin());
            continue;
        }

        // Check if we have enough data to process
        if (read_buffer.size() > kMinBufferSize) {
            // If the second element in the buffer is not 0x03, remove the first element
            if (read_buffer.at(1) != 0x03) {
                read_buffer.erase(read_buffer.begin());
                continue;
            }

            const size_t dataLength = read_buffer.at(2) + kHeaderSize;
            const size_t crcIndex = dataLength - kCrcSize;

            auto buffer_len = read_buffer.size();
            // If enough data has been read
            if (buffer_len >= dataLength) {
                std::vector<uint8_t>tmp_bytes (read_buffer.begin(), read_buffer.begin() + dataLength);
                const uint16_t crc = get_crc(tmp_bytes, dataLength - 2);

                if (crc == (read_buffer.at(crcIndex) << 8) + read_buffer.at(crcIndex + 1)) {
                    auto data = processData(tmp_bytes, dataLength);
                    for (auto const &el: data) {
                        std::cout << el.first << ": " << el.second << " ";
                    }
                    std::cout << std::endl;

                    read_buffer.erase(read_buffer.begin(), read_buffer.begin() + dataLength);
                    
                }
                else {
                    read_buffer.erase(read_buffer.begin());
                }
            }
        }

    }
}

void Driver::sendData(LibSerial::SerialPort &serial_port, std::vector<uint8_t> &data) {
    try { 
        std::stringstream result;
        std::copy(data.begin(), data.end(), std::ostream_iterator<int>(result, " "));
        
        serial_port.Write(data);

    } catch (std::exception& ex) {
        std::cout << ex.what() << std::endl;
    }
}

void Driver::readReg(LibSerial::SerialPort &serial_port, const uint16_t device_addr, const uint16_t reg_addr, const uint16_t reg_count) {
    auto data = get_readBytes(device_addr, reg_addr, reg_count);  // Vector of bytes
    sendData(serial_port, data);
}

std::vector<uint8_t> Driver::read_register(LibSerial::SerialPort &serial_port, const uint16_t device_addr, const uint16_t reg_addr, const uint16_t reg_count) {
    // General function to read data from register. Sends a read command to the device and reads the response. Returns a vector of integers representing the data read.
    std::cout << "Reading address " << std::hex << reg_addr << std::endl;

    // Flush buffers
    serial_port.FlushIOBuffers();

    // Send command to read data
    readReg(serial_port, device_addr, reg_addr, reg_count);

    // Sleep for a bit to ensure the data is available
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Wait until data is available or timeout
    size_t ms_timeout = 10;
    LibSerial::DataBuffer read_data_buffer;
    LibSerial::DataBuffer read_header_buffer;

    try {
        // Read 3 bytes - ID, function code and read length
        serial_port.Read(read_header_buffer, 3, ms_timeout);
        
        uint16_t read_len = static_cast<int>(read_header_buffer.at(2));

        // Read the rest of the data
        serial_port.Read(read_data_buffer, read_len, ms_timeout);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cout << "Read header " << read_header_buffer.size() << " bytes after timeout: " << std::endl;
        for (auto const &el: read_header_buffer) {
            std::cout << std::hex << static_cast<int>(el) << std::flush ;
        }
        std::cout << std::endl;
        std::cout << "Read data " << read_data_buffer.size() << " bytes after timeout: " << std::endl;
        for (auto const &el: read_data_buffer) {
            std::cout << std::hex << static_cast<int>(el) << std::flush ;
        }
        std::cout << std::endl;
    }

    // Merge header and data
    for (auto const &el: read_data_buffer) {
        read_header_buffer.push_back(el);
    }

    return read_header_buffer;
}

std::unordered_map<std::string, std::string> Driver::read_config(LibSerial::SerialPort &serial_port, const uint16_t ADDR, std::string register_name) {
    std::unordered_map<std::string, std::string> config_tmp {};

    auto register_info = CONFIG_REGISTERS.at(register_name);
    char register_addr = register_info.first;
    uint16_t register_count = register_info.second;

    auto register_response = read_register(serial_port, ADDR, register_addr, register_count);

    // Parse
    if (register_name == "BAUDRATE")
        return parse_baudrate_register(register_response);

    else if (register_name == "BANDWIDTH")
        return parse_bandwidth_register(register_response);

    else if (register_name == "AXIS")
        return parse_axis_register(register_response);

    else if (register_name == "CALSW")
        return parse_calsw_register(register_response);
}

std::vector<uint8_t> Driver::get_readBytes(const uint8_t devid, const uint16_t regAddr, const uint16_t regCount) {
    // initialization
    std::vector<uint8_t> tempBytes = {};

    // Device modbus address
    tempBytes.push_back(devid);

    // Read function code
    tempBytes.push_back(0x03);

    // High 8 bits of register address
    tempBytes.push_back(static_cast<uint8_t>(regAddr >> 8));

    // Low 8 bits of register address
    tempBytes.push_back(static_cast<uint8_t>(regAddr & 0xff));

    // High 8 bits of register count
    tempBytes.push_back(static_cast<uint8_t>(regCount >> 8));

    // Low 8 bits of register count
    tempBytes.push_back(static_cast<uint8_t>(regCount & 0xff));

    // Get CRC check
    uint16_t tempCrc = get_crc(tempBytes, tempBytes.size());
    // Crc high 8 bits
    tempBytes.push_back(static_cast<uint8_t>(tempCrc >> 8));

    // Crc low 8 bits
    tempBytes.push_back(static_cast<uint8_t>(tempCrc & 0xff));

    return tempBytes;
}




