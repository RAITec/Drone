#include "Wifi.hpp"
#include "Drone.h"

//Wifi *w = new Wifi();
Drone d;

void setup() {
//  w->WifiSetup("davi", "pipipipopopo","192.168.67.61",5000,115200);
//  w->connecting();
  d.MainControlSetup(115200, 14, 27, 25, 26, 33, 32, 17, 5);
  d.MPUconfigSetup();
}

void loop() {
    d.MPUgetSignalsLoop();
    d.DisplaySerialMpuData();
    
//    d.MainControlLoop();
//    d.DisplayPlotterMpuData();
//    w->getData(d.getData());
    
}
