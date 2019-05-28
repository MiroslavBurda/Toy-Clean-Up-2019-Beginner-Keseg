#include <Arduino.h>
#include <Wire.h>
#include "time.hpp"
#include <stdint.h>
#include "stopwatch.hpp"
#include "nvs_flash.h"
#include "BluetoothSerial.h"


#define OTOCNY_MOTOR  rb::MotorId::M1 // motor 
bool L_G_light = false;
int8_t axis[7] = {5,6,7,8,9,10,11};
byte btn[8] = {0,0,0,0,0,0,0,0};
byte btn_last[8] = {0,0,0,0,0,0,0,0};

BluetoothSerial SerialBT;
Stream* serial = nullptr;
timeout send_data { msec(500) }; // timeout zajistuje posilani dat do PC kazdych 500 ms


void setup() {
  Serial.begin(115200);
    if (!SerialBT.begin("firclean")) //Bluetooth device name
    {
        Serial.println("!!! Bluetooth initialization failed!");
        serial = &Serial;
    }
    else
    {
        serial = &SerialBT;
        SerialBT.println("!!! Bluetooth work!");
        Serial.println("!!! Bluetooth work!");
    }
  
  Serial.println("Starting");
  pinMode(L_G, OUTPUT);

}

void read_joystick();

void loop() {
  if (send_data) {
        send_data.ack();
        if (L_G_light) L_G_light = false; else  L_G_light = true;
        digitalWrite(L_G, L_G_light);  // blikani LED indikuje provoz
        Serial.println( millis() );
        SerialBT.println( millis() ); // na tomto pocitaci COM port 5
    }
  read_joystick(); 
}

void read_joystick(){
if (SerialBT.available() > 0)
    {
        uint8_t test = SerialBT.read();
        if (test == 0x80)
            for (uint8_t x = 0; x < 7; x++)
            {
			    while(SerialBT.available() < 1) {
				    // DO NOTHING - WAITING FOR PACKET
				    delay(1);
			    }

                axis[x] = SerialBT.read();
                Serial.print(x);
                Serial.print(": ");
                Serial.print(axis[x], DEC);
                Serial.print(" ");
            }
        else if  ( test == 0x81 )
        {
		    while(SerialBT.available() < 1) {
			    // DO NOTHING - WAITING FOR PACKET
			    delay(1);
		    }
            byte a = SerialBT.read();
		    while(SerialBT.available() < 1) {
			    // DO NOTHING - WAITING FOR PACKET
			    delay(1);
		    }
            btn_last[a] = btn[a]; 
            btn[a] = SerialBT.read();
		    Serial.print(a, DEC); Serial.print(": "); Serial.print(btn[a], DEC); Serial.print("last: "); Serial.print(btn_last[a], DEC);
        SerialBT.print(a, DEC); SerialBT.print(": "); SerialBT.print(btn[a], DEC); SerialBT.print("last: "); SerialBT.print(btn_last[a], DEC);

        }
        Serial.println(" ");
        SerialBT.println(" ");
        
    }
}
