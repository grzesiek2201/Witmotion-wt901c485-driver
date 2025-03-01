#include <iostream>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <unordered_map>
#include <chrono>
#include <thread>
#include <vector>
#include <iterator>
#include <algorithm>
#include <unistd.h> // usleep
#include <sstream>
#include <math.h>

// Calculate CRC
const std::vector<uint16_t> auchCRCHi = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40};


const std::vector<uint16_t> auchCRCLo = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40};


// For a given register name, there's a pair of (register address, register length)
const std::unordered_map<std::string, std::pair<uint8_t, uint8_t>> CONFIG_REGISTERS = {
    {"BAUDRATE", std::make_pair(0x04, 1)},
    {"BANDWIDTH", std::make_pair(0x1F, 1)},
    {"AXIS", std::make_pair(0x24, 1)},
    {"CALSW", std::make_pair(0x01, 1)}
};


// 0x04
const std::unordered_map<char, std::string> BAUDRATE_VALUES = {
    {0x01, "4800bps"},
    {0x02, "9600bps"},
    {0x03, "19200bps"},
    {0x04, "38400bps"},
    {0x05, "57600bps"},
    {0x06, "115200bps"},
    {0x07, "230400bps"},
    {0x08, "460800bps"},
    {0x09, "921600bps"}
};

// 0x1F
const std::unordered_map<char, std::string> BANDWIDTH_VALUES = {
    {0x00, "256Hz"},
    {0x01, "188Hz"},
    {0x02, "98Hz"},
    {0x03, "42Hz"},
    {0x04, "20Hz"},
    {0x05, "10Hz"},
    {0x06, "5Hz"}
};

// 0x24
const std::unordered_map<char, std::string> AXIS_VALUES = {
    {0x00, "9-axis"},
    {0x01, "6-axis"}
};

// 0x01
const std::unordered_map<char, std::string> CALSW_VALUES = {
    {0x00, "NORMAL"},
    {0x01, "ACC_CALIB"},
    {0x03, "HEIGHT_RESET"},
    {0x04, "RESET_HEADING_ANGLE"},
    {0x07, "MAG_CALIB_SPHERICAL"},
    {0x08, "SET_ANGLE_REFERENCE"},
    {0x09, "MAG_CALIB_DUAL_PLANE"}
};


uint16_t get_crc(const std::vector<uint8_t> &datas, const int dlen) {
    uint16_t tempH = 0xff;  // High CRC byte initialization
    uint16_t tempL = 0xff;  // Low CRC byte initialization

    for (int i{0}; i<dlen; i++) {
        uint16_t tempIndex = (tempH ^ datas[i]) & 0xff;
        tempH = (tempL ^ auchCRCHi[tempIndex]) & 0xff;
        tempL = auchCRCLo[tempIndex];
    }

    return (tempH << 8) | tempL;
}


template <typename T>
void print_vector(const T& vec) {
    std::copy(vec.cbegin(), vec.cend(), std::ostream_iterator<typename T::value_type>(std::cout, ", "));
    std::cout << std::endl;
}


std::vector<uint8_t> get_readBytes(const uint8_t devid, const uint16_t regAddr, const uint16_t regCount) {
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


void sendData(LibSerial::SerialPort &serial_port, std::vector<uint8_t> &data) {
    try {
        
        std::stringstream result;
        std::copy(data.begin(), data.end(), std::ostream_iterator<int>(result, " "));
        
        serial_port.Write(data);

    } catch (std::exception& ex) {
        std::cout << ex.what() << std::endl;
    }
}


void readReg(LibSerial::SerialPort &serial_port, const uint16_t ADDR, const uint16_t regAddr, const uint16_t regCount) {
    auto data = get_readBytes(ADDR, regAddr, regCount);  // Vector of bytes
    sendData(serial_port, data);
}


std::vector<uint16_t> extract_register_data(const std::vector<uint8_t>& data, int n_registers) {
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


std::unordered_map<std::string, std::string> parse_baudrate_register(const std::vector<uint8_t>& data) {
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


std::unordered_map<std::string, std::string> parse_bandwidth_register(const std::vector<uint8_t>& data) {
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


std::unordered_map<std::string, std::string> parse_axis_register(const std::vector<uint8_t>& data) {
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


std::unordered_map<std::string, std::string> parse_calsw_register(const std::vector<uint8_t>& data) {
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


std::vector<uint8_t> read_register(LibSerial::SerialPort &serial_port, const uint16_t device_addr, const uint16_t reg_addr, const uint16_t reg_count = 1) {
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


std::unordered_map<std::string, std::string> read_config(LibSerial::SerialPort &serial_port, const uint16_t ADDR, std::string register_name) {
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


void send_read_command(LibSerial::SerialPort &serial_port, const uint16_t device_addr) {
    while (true) {
        readReg(serial_port, device_addr, 0x34, 13);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}


int16_t getSignInt16(uint16_t num) {
    if (num >= pow(2, 15)) {
        num -= pow(2, 16);
    }
    return num;
}


using std::chrono::high_resolution_clock;
auto t_last = high_resolution_clock::now();

std::unordered_map<std::string, double> processData(const std::vector<uint8_t>& bytes, const int length) {
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

    return data;
}


std::vector<uint8_t> read_buffer;

void read_data(LibSerial::SerialPort &serial_port, const uint16_t ADDR) {
    constexpr size_t kTimeoutMs = 1;
    constexpr size_t kHeaderSize = 5;
    constexpr size_t kCrcSize = 2;
    const size_t kMinBufferSize = kHeaderSize + kCrcSize;

    std::vector<uint8_t> buffer;

    while (true) {
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


int main() {
    using LibSerial::SerialPort;

    SerialPort      serial_port;

    // Open serial ports
    serial_port.Open( "/dev/ttyUSB0" );

    using namespace LibSerial;
    serial_port.SetBaudRate( BaudRate::BAUD_115200 );
    serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    serial_port.SetParity(Parity::PARITY_NONE);
    serial_port.SetStopBits(StopBits::STOP_BITS_1);

    serial_port.FlushIOBuffers();

    std::unordered_map<std::string, std::string> config {};
    for (auto const &el: CONFIG_REGISTERS) {
        config = read_config(serial_port, 0x50, el.first);
        for (auto const &el: config) {
            std::cout << el.first << ": " << el.second << std::endl;
        }
    }


    std::thread serial_sender(send_read_command, std::ref(serial_port), 0x50);
    std::thread serial_reader(read_data, std::ref(serial_port), 0x50);

    serial_sender.join();
    serial_reader.join();


    serial_port.Close();

    return 0;
}