#include "Arduino.h"
#include "Wifi.hpp"
#include "WiFi.h"
#include "WiFiUdp.h"
#include "Data.hpp"
#include <iostream>

using namespace std;

void Wifi::WifiSetup(char* ssid, char* password, char* targetIp, uint16_t portaUDP, int freq){
    this->ssid = ssid;
    this->password = password;
    this->targetIp = targetIp;
    this->portaUDP = portaUDP;
    Serial.begin(freq);

    WiFi.begin(ssid, password);
    canal = new WiFiUDP();
}

void Wifi::connecting(){
    while(WiFi.status() !=  WL_CONNECTED){
        delay(500);
        Serial.printf("Conectando...\n");
    }
    Serial.println("Conectado!");
}

void Wifi::getData(Data newData){
    Data data = newData;
    
     string mensagem = 
        "Motor1: " + to_string(data.motor1) + 
     " , Motor2: " + to_string(data.motor2) + 
     " , Motor3: " + to_string(data.motor3) + 
     " , Motor4: " + to_string(data.motor4) + 
     " , Angle_Roll: " + to_string(data.roll) +
     " , Angle_Pitch: " + to_string(data.pitch) +
     " , Kalman_Angle_Roll: " + to_string(data.kRoll) +
     " , Kalman_Angle_Pitch: " + to_string(data.kPitch) + 
     " , Tempo: " + to_string(data.tempo) + "\n";
    
//    string mensagem = 
//    "Angle_Roll: " + to_string(data.roll) +
//    " , Angle_Pitch: " + to_string(data.pitch) +
//    " , Tempo: " + to_string(data.tempo) + "\n";

    char* mensagemFinal = new char[mensagem.size() + 2];
  

    strcpy(mensagemFinal, mensagem.c_str());

    canal->beginPacket(targetIp, portaUDP);
    canal->print(mensagemFinal);
    Serial.printf("%s", mensagemFinal);
    canal->endPacket();
    delay(10);
}
