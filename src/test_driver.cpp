#include <iostream>
#include "zlac8015d_driver.h"
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

static std::atomic_bool running{true};
static void on_sigint(int) { running = false; }

int main() {
  std::signal(SIGINT, on_sigint);

  ZLAC8015D driver("/dev/ttyUSB0", 115200, "velocity");
  if (!driver.connect()) return (std::cerr << "Failed to connect\n", 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  driver.set_speed_resolution(SpeedRes::RPM_0_1);

  while (running) {
    auto [l, r] = driver.get_error();

    if (l == "0x0002" && r == "0x0002") {
      driver.reset_alarm();
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
      driver.enable_motor();
      std::cout << "Me Conecté...\n";
    }

    driver.set_sync_rpm(105, 105); // Prueba: aplica una velocidad constante de 100 RPM a ambos motores
    auto [enc_left, enc_right] = driver.get_encoder_count();
    std::cout << "Encoder counts: Left: " << enc_left << " Right: " << enc_right << "\n";

    auto [temp_left, temp_right] = driver.get_temperature();
    std::cout << "Temperatures: Left: " << temp_left << "°C Right: " << temp_right << "°C\n";

    std::cout << l << "\n" << r << "\n\n";

    driver.get_software_version();
    driver.get_current();
    driver.get_rpm();

    // --- NO CRASHEA si l/r no son hex ---
    std::string decoded_left = l;
    std::string decoded_right = r;

    try {
      int code_left = std::stoi(l, nullptr, 16);   // base 16, acepta "0x...."
      decoded_left = driver.decode_error(code_left, "left");
    } catch (...) {
      // se queda con el mensaje original (ej. "Not connected")
    }

    try {
      int code_right = std::stoi(r, nullptr, 16);
      decoded_right = driver.decode_error(code_right, "right");
    } catch (...) {
      // se queda con el mensaje original
    }

    std::cout << "Decoded left: " << decoded_left << "\n";
    std::cout << "Decoded right: " << decoded_right << "\n\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // Al salir con Ctrl+C: manda 0 para no dejar torque/corriente aplicada
  driver.set_sync_rpm(0, 0);
  std::cout << "Stopped (Ctrl+C).\n";
  return 0;
}
