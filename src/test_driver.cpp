
#include <iostream>
#include "zlac8015d_driver.h"  // tu clase ZLAC8015D
// #include "gains_types.hpp"     // si lo separaste

int main() {
  ZLAC8015D driver("/dev/ttyUSB0", 115200, "absolute_position");  // o como construyas tu driver
  if (!driver.connect()) {
    std::cerr << "Failed to connect\n";
    return 1;
  }

  PositionGains g;
  g.left.kp  = 200;
  g.left.kf  = 200;

  g.right.kp = 200;
  g.right.kf = 200;

  int ret = driver.set_control_gains_position(g);
  std::cout << "ret=" << ret << "\n";

  auto read_gains = driver.get_control_gains_position();  

  return (ret == 0) ? 0 : 1;
}
