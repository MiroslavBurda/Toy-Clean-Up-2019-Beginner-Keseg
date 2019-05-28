#include <Arduino.h>
#include <Wire.h>
#include "time.hpp"
#include <stdint.h>
#include "stopwatch.hpp"
#include "nvs_flash.h"
#include "BluetoothSerial.h"


#define OTOCNY_MOTOR  rb::MotorId::M1 // motor
#define MIN_M_VAL -32768
#define MAX_M_VAL  32767
#define MIN_PWM    255
bool L_G_light = false;
int8_t axis[7] = {5,6,7,8,9,10,11};
byte btn[8] = {0,0,0,0,0,0,0,0};
byte btn_last[8] = {0,0,0,0,0,0,0,0};
int speed_coef = 50; // nasobeni hodnoty, co leze z joysticku

BluetoothSerial SerialBT;
Stream* serial = nullptr;
timeout send_data { msec(500) }; // timeout zajistuje posilani dat do PC kazdych 500 ms


void setup() {
	
	Serial.begin(115200);
	pinMode(4, OUTPUT);
	digitalWrite(4, HIGH);

	ledcSetup(1,1000,8);
	ledcSetup(2,1000,8);

	ledcAttachPin(18, 1);
	ledcAttachPin(5, 2);

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
	pinMode(L_G, OUTPUT); // asi neni nutne

}

bool read_joystick(); // dole pod loop

void loop() {
	if (send_data) {
		send_data.ack();
		if (L_G_light) L_G_light = false; else  L_G_light = true;
		digitalWrite(L_G, L_G_light);  // blikani LED indikuje provoz
		Serial.println( millis() );
		SerialBT.println( millis() ); // na pocitaci Burda COM port 5
	}
	
	if ( read_joystick() )
	{
		Serial.print("Event");
		float axis_0 = (abs(axis[0]) < 10) ? 0 : axis[0] /128.0;
		axis_0 = axis_0*axis_0*axis_0;
		float axis_1 = (abs(axis[1]) < 10) ? 0 : axis[1] /128.0;
		axis_1 = axis_1*axis_1*axis_1;
		int levy_m = (axis_1- (axis_0 /2 )) * speed_coef;  // hodnota pro levy motor
		int pravy_m = (axis_1+ (axis_0 /2 )) * speed_coef; // hodnota pro pravy motor
		
		// TODO: Petr Bobcik Begin
		// levy_m a pravy_m are -32.768 to 32.767
		int levy_m_duty  = ( levy_m  * 255 ) / ( abs(MIN_M_VAL) );
		int pravy_m_duty = ( pravy_m * 255 ) / ( abs(MIN_M_VAL) );

		if(levy_m_duty == -1)
		levy_m_duty = MIN_PWM;
		if(pravy_m_duty == -1)
		pravy_m_duty = MIN_PWM;

		ledcWrite(1, levy_m_duty);
		ledcWrite(2, pravy_m_duty);
		// TODO: Petr Bobcik End


		printf(" %i %i \n ", levy_m, pravy_m );
		SerialBT.print(levy_m); SerialBT.print(" "); SerialBT.println(pravy_m);
	}
	delay(10);
}

bool read_joystick()
{
	if ( SerialBT.available() == 0 )
	return false;
	
	int test = SerialBT.read();
	if (test == 0x80)
	{
		int axis_count = SerialBT.read();
		for (int x = 0; x < axis_count; x++)
		{
			while(SerialBT.available() < 1)
			{
				// DO NOTHING - WAITING FOR PACKET
				delay(1);
			}

			int8_t tmp = SerialBT.read();
			axis[x] = tmp;
			Serial.print(x);
			Serial.print(": ");
			Serial.print(axis[x], DEC);
			Serial.print(" ");
			SerialBT.print(x);
			SerialBT.print(": ");
			SerialBT.print(axis[x], DEC);
			SerialBT.print(" ");
		}
		return true;
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
		return true;
	}
	return false;
}