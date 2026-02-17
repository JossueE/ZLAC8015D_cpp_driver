
#include <iostream>
#include "zlac8015d_driver.h" 
#include <thread>
#include <chrono>

int main() {
  ZLAC8015D driver("/dev/ttyUSB1", 115200, "velocity");
  if (!driver.connect()) return (std::cerr << "Failed to connect\n", 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  driver.set_rpm(10, 10);

  while (true) {
    auto [l, r] = driver.get_error();
    if (l == "0x0002" && r == "0x0002") {
      driver.reset_alarm();
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      driver.enable_motor();
      std::cout << "Me Conecté...\n";
    } 
    driver.set_rpm(10, 10);
    auto [enc_left, enc_right] = driver.get_encoder_count();
    std::cout << "Encoder counts: Left: " << enc_left << " Right: " << enc_right << "\n";
    auto [temp_left, temp_right] = driver.get_temperature();
    std::cout << "Temperatures: Left: " << temp_left << "°C Right: " << temp_right << "°C\n";
    std::cout << l << "\n" << r << "\n\n";
    driver.get_software_version();

    std::string decoded_left = driver.decode_error(std::stoi(l.substr(2)), "left");
    std::string decoded_right = driver.decode_error(std::stoi(r.substr(2)), "right");
    std::cout << "Decoded left: " << decoded_left << "\n";
    std::cout << "Decoded right: " << decoded_right << "\n\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}


