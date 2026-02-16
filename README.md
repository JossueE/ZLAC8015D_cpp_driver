g++ -I./include src/*.cpp -o ZLAC -lmodbus

./ZLAC

to detect
 - ls /dev/ttyUSB*

To have permitions
 - sudo chmod 666 /dev/ttyUSB0


Ejemplo mínimo para leer PIF:


''' cpp

auto gains_opt = driver.get_control_gains_velocity();

if (gains_opt) {                         // o gains_opt.has_value()
    const VelocityGains& gains = *gains_opt;  // referencia al contenido

    std::cout << "kp_left=" << gains.left.kp
            << " kf_left=" << gains.left.kf
            << " ki_left=" << gains.left.ki  << "\n";  // según tus campos

    std::cout << "kp_right=" << gains.right.kp
            << " kf_right=" << gains.right.kf
            << " ki_right=" << gains.right.ki << "\n";  // según tus

} else {
    std::cout << "No se pudieron leer las ganancias (optional vacío)\n";
}

'''


Ejemplo mínimo para escribir:

#include <iostream>
#include "zlac8015d_driver.hpp"   // tu clase ZLAC8015D
// #include "gains_types.hpp"     // si lo separaste

int main() {
  ZLAC8015D driver("/dev/ttyUSB0", 115200, "absolute_position");  // o como construyas tu driver
  if (!driver.connect()) {
    std::cerr << "Failed to connect\n";
    return 1;
  }

  VelocityGains g;
  g.left.kp  = 100;
  g.left.ki  = 20;
  g.left.kf  = 0;

  g.right.kp = 100;
  g.right.ki = 20;
  g.right.kf = 0;

  int ret = driver.set_control_gains_velocity(g);
  std::cout << "ret=" << ret << "\n";
  return (ret == 0) ? 0 : 1;
}

