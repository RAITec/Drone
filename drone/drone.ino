#include "Wifi.hpp"
#include "Drone.h"

Wifi w;
Drone d;

void setup() {
  w.WifiSetup("davi", "pipipipopopo","192.168.140.152",5000,115200);
  w.connecting();
  d.MainControlSetup(115200, 18, 19, 17, 5, 32, 26, 25, 33);
  d.MPUconfigSetup();
}

void loop() {
    d.MPUgetSignalsLoop();
    //d.DisplaySerialMpuData();
      
    d.MainControlLoop();
    d.DisplayPlotterMpuData();
    w.getData(d.getData());
    
}
