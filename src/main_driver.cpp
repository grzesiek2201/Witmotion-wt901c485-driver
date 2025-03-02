#include "driver.hpp"
#include <thread>


int main() {
    Driver driver;
    driver.configure_device("/dev/ttyUSB0", 115200, 98, 1, 0, 50);
    
    // Read config
    std::unordered_map<std::string, std::string> config {};
    for (auto const &el: CONFIG_REGISTERS) {
        config = driver.read_config(0x50, el.first);
        for (auto const &el: config) {
            std::cout << el.first << ": " << el.second << std::endl;
        }
    }

    // Start measurements
    driver.start_reading_data(0x50, false);

    std::thread display_data([&driver]() {
        while (true) {
            std::cout << "Sensor data: ";
            auto data = driver.get_sensor_data();
            for (auto const &el: data) {
                std::cout << el.first << ": " << el.second << ", ";
            }
            std::cout << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    });

    display_data.join();

    return 0;
}