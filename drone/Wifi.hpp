#ifndef WIFI_HPP
#define WIFI_HPP
#include "Arduino.h"
#include "WiFi.h"
#include "WiFiUdp.h"
#include "Data.hpp"

class Wifi{
    private:
    char* ssid;
    char* password;
    char* targetIp;
    uint16_t portaUDP;
    WiFiUDP *canal;


    public:

    void WifiSetup(char* ssid, char* password, char* targetIp, uint16_t portaUDP, int freq);

    void connecting();


    void getData(Data newData);


};


#endif