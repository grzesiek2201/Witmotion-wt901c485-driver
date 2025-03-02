#include "driver.hpp"


Driver::Driver() {
}

Driver::~Driver() {
    if (serial_port.IsOpen()) {
        std::cout << "Closing the device..." << std::endl;
        serial_port.Close();
    }
}

void Driver::configure_device(std::string port, uint32_t baudrate, uint16_t bandwidth, uint16_t axis, uint16_t calsw, uint polling_ms) {
    serial_port.Open(port);
    using namespace LibSerial;
    serial_port.SetBaudRate( BaudRate::BAUD_115200 );
    serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    serial_port.SetParity(Parity::PARITY_NONE);
    serial_port.SetStopBits(StopBits::STOP_BITS_1);
    this->polling_ms = polling_ms;
}

void Driver::start_reading_data(const uint16_t ADDR, bool blocking) {
    std::thread serial_sender(&Driver::send_read_command, this, ADDR);
    std::thread serial_reader(&Driver::read_data, this, 0x50);

    if (blocking) {
        serial_sender.join();
        serial_reader.join();
    }
    else {
        serial_sender.detach();
        serial_reader.detach();
    }
}

std::unordered_map<std::string, double> Driver::get_sensor_data() {
    return sensor_data;
}

void Driver::send_read_command(const uint16_t device_addr) {
    while (true) {
        readReg(device_addr, 0x34, 13);
        std::this_thread::sleep_for(std::chrono::milliseconds(polling_ms));
    }
}

void Driver::read_data(const uint16_t ADDR) {
    constexpr size_t kTimeoutMs = 1;
    constexpr size_t kHeaderSize = 5;
    constexpr size_t kCrcSize = 2;
    const size_t kMinBufferSize = kHeaderSize + kCrcSize;
    
    std::vector<uint8_t> read_buffer;
    std::vector<uint8_t> buffer;

    while (true) {
        size_t bytes_available = serial_port.GetNumberOfBytesAvailable();
        if (bytes_available >= 13 * 2 + kHeaderSize) {
            // std::cout << "Reading " << bytes_available << " bytes\n";
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
                    // for (auto const &el: data) {
                    //     std::cout << el.first << ": " << el.second << " ";
                    // }
                    // std::cout << std::endl;

                    read_buffer.erase(read_buffer.begin(), read_buffer.begin() + dataLength);
                    
                }
                else {
                    read_buffer.erase(read_buffer.begin());
                }
            }
        }
        // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(polling_ms / 10)));
    }
}

void Driver::sendData(std::vector<uint8_t> &data) {
    try { 
        std::stringstream result;
        std::copy(data.begin(), data.end(), std::ostream_iterator<int>(result, " "));
        
        serial_port.Write(data);

    } catch (std::exception& ex) {
        std::cout << ex.what() << std::endl;
    }
}

void Driver::readReg(const uint16_t device_addr, const uint16_t reg_addr, const uint16_t reg_count) {
    auto data = get_readBytes(device_addr, reg_addr, reg_count);  // Vector of bytes
    sendData(data);
}

std::vector<uint8_t> Driver::read_register(const uint16_t device_addr, const uint16_t reg_addr, const uint16_t reg_count) {
    // General function to read data from register. Sends a read command to the device and reads the response. Returns a vector of integers representing the data read.
    std::cout << "Reading address " << std::hex << reg_addr << std::endl;

    // Flush buffers
    serial_port.FlushIOBuffers();

    // Send command to read data
    readReg(device_addr, reg_addr, reg_count);

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

std::unordered_map<std::string, std::string> Driver::read_config(const uint16_t ADDR, std::string register_name) {
    std::unordered_map<std::string, std::string> config_tmp {};

    auto register_info = CONFIG_REGISTERS.at(register_name);
    char register_addr = register_info.first;
    uint16_t register_count = register_info.second;

    auto register_response = read_register(ADDR, register_addr, register_count);

    // Parse
    if (register_name == "BAUDRATE")
        return parse_baudrate_register(register_response);

    else if (register_name == "BANDWIDTH")
        return parse_bandwidth_register(register_response);

    else if (register_name == "AXIS")
        return parse_axis_register(register_response);

    else if (register_name == "CALSW")
        return parse_calsw_register(register_response);

    return config_tmp;
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

uint16_t Driver::get_crc(const std::vector<uint8_t> &datas, const int dlen) {
    uint16_t tempH = 0xff;  // High CRC byte initialization
    uint16_t tempL = 0xff;  // Low CRC byte initialization

    for (int i{0}; i<dlen; i++) {
        uint16_t tempIndex = (tempH ^ datas[i]) & 0xff;
        tempH = (tempL ^ auchCRCHi[tempIndex]) & 0xff;
        tempL = auchCRCLo[tempIndex];
    }

    return (tempH << 8) | tempL;
}

std::vector<uint16_t> Driver::extract_register_data(const std::vector<uint8_t>& data, int n_registers) {
    if (data.size() < 5) {
        std::cout << "Invalid data length" << std::endl;
        return {};
    }

    int device_id = data[0];  // First byte (device address)
    int function_code = data[1];  // Second byte (function code)
    int byte_count = data[2];  // Third byte (number of data bytes)

    if (byte_count + n_registers > data.size()) {
        std::cout << "Incorrect byte count" << std::endl;
        return {};
    }

    uint16_t reg_value;
    std::vector<uint16_t> registers;

    for (int i = 3; i < 3 + byte_count; i += 2) {
        // reg_value = struct.unpack(">H", data[i:i+2])[0]
        uint16_t reg_value = data[i] << 8 | data[i + 1];
        registers.push_back(reg_value);
    }

    return registers;
}

int16_t Driver::getSignInt16(uint16_t num) {
    if (num >= pow(2, 15)) {
        num -= pow(2, 16);
    }
    return num;
}

std::unordered_map<std::string, std::string> Driver::parse_baudrate_register(const std::vector<uint8_t>& data) {
    auto registers = extract_register_data(data, 1);

    if (registers.empty()) {
        return {};
    }

    // Convert to human readable format
    std::unordered_map<std::string, std::string> config {};

    uint8_t baudrate_value = registers[0] & 0x0F;  // Last 4 bits
    config["BAUDRATE"] = BAUDRATE_VALUES.at(baudrate_value);

    return config;
}

std::unordered_map<std::string, std::string> Driver::parse_bandwidth_register(const std::vector<uint8_t>& data) {
    auto registers = extract_register_data(data, 1);

    if (registers.empty()) {
        return {};
    }

    // Convert to human readable format
    std::unordered_map<std::string, std::string> config {};

    uint8_t bandwidth_value = registers[0] & 0x0F;  // Last 4 bits
    config["BANDWIDTH"] = BANDWIDTH_VALUES.at(bandwidth_value);

    return config;
}

std::unordered_map<std::string, std::string> Driver::parse_axis_register(const std::vector<uint8_t>& data) {
    auto registers = extract_register_data(data, 1);

    if (registers.empty()) {
        return {};
    }

    // Convert to human readable format
    std::unordered_map<std::string, std::string> config {};

    uint8_t axis_value = registers[0] & 0x0F;  // Last 4 bits
    config["AXIS"] = AXIS_VALUES.at(axis_value);

    return config;
}

std::unordered_map<std::string, std::string> Driver::parse_calsw_register(const std::vector<uint8_t>& data) {
    auto registers = extract_register_data(data, 1);

    if (registers.empty()) {
        return {};
    }    

    // Convert to human readable format
    std::unordered_map<std::string, std::string> config {};

    uint8_t calsw_value = registers[0] & 0x0F;  // Last 4 bits
    config["CALSW"] = CALSW_VALUES.at(calsw_value);

    return config;
}

std::unordered_map<std::string, double> Driver::processData(const std::vector<uint8_t>& bytes, const int length) {
    uint16_t addr = bytes.at(0);
    std::unordered_map<std::string, double> data {};

    using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    auto t_now = high_resolution_clock::now();
    /* Getting number of milliseconds as an integer. */
    auto ms_int = duration_cast<milliseconds>(t_now - t_last);
    std::cout << std::dec << ms_int.count() << "ms\n";
    t_last = high_resolution_clock::now();

    if (length - 5 == 26) {
        double accx = static_cast<double>(getSignInt16(bytes.at(3) << 8 | bytes.at(4))) / 32768 * 16;
        double accy = static_cast<double>(getSignInt16(bytes.at(5) << 8 | bytes.at(6))) / 32768 * 16;
        double accz = static_cast<double>(getSignInt16(bytes.at(7) << 8 | bytes.at(8))) / 32768 * 16;

        double asx = static_cast<double>(getSignInt16(bytes.at(9) << 8 | bytes.at(10))) / 32768 * 2000;
        double asy = static_cast<double>(getSignInt16(bytes.at(11) << 8 | bytes.at(12))) / 32768 * 2000;
        double asz = static_cast<double>(getSignInt16(bytes.at(13) << 8 | bytes.at(14))) / 32768 * 2000;

        double hx = static_cast<double>(getSignInt16(bytes.at(15) << 8 | bytes.at(16))) / 13 / 1000;
        double hy = static_cast<double>(getSignInt16(bytes.at(17) << 8 | bytes.at(18))) / 13 / 1000;
        double hz = static_cast<double>(getSignInt16(bytes.at(19) << 8 | bytes.at(20))) / 13 / 1000;

        double angx = static_cast<double>(getSignInt16(bytes.at(21) << 8 | bytes.at(22))) / 32768 * 180;
        double angy = static_cast<double>(getSignInt16(bytes.at(23) << 8 | bytes.at(24))) / 32768 * 180;
        double angz = static_cast<double>(getSignInt16(bytes.at(25) << 8 | bytes.at(26))) / 32768 * 180;

        double temp = static_cast<double>(getSignInt16(bytes.at(27) << 8 | bytes.at(28))) / 100;

        data = {
            {"accx", accx},
            {"accy", accy},
            {"accz", accz},
            {"asx", asx},
            {"asy", asy},
            {"asz", asz},
            {"hx", hx},
            {"hy", hy},
            {"hz", hz},
            {"angx", angx},
            {"angy", angy},
            {"angz", angz},
            {"temp", temp},
        };
    }

    sensor_data = data;

    return data;
}
