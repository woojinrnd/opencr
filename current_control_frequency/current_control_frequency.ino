/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>
#include <math.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)  // When using DynamixelShield
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8);  // DYNAMIXELShield UART RX/TX
#define DXL_SERIAL Serial
#define DEBUG_SERIAL soft_serial
const int DXL_DIR_PIN = 2;      // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE)  // When using DynamixelShield
#define DXL_SERIAL Serial
#define DEBUG_SERIAL SerialUSB
const int DXL_DIR_PIN = 2;  // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO)  // When using DynamixelShield
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL SerialUSB
const int DXL_DIR_PIN = 2;  // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904)  // When using official ROBOTIS board with DXL circuit.
#define DXL_SERIAL Serial3        //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 22;  //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR)  // When using official ROBOTIS board with DXL circuit.
// For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
// Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84;  // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
//OpenRB does not require the DIR control pin.
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;
#else  // Other boards when using DynamixelShield
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 2;  // DYNAMIXEL Shield DIR PIN
#endif


//ID 지정
// const uint8_t DXL_ID[6] = { 7, 3, 1, 5, 8, 2 };
// const uint8_t DXL_ID[4] = { 7, 3, 1, 5 };


// const uint8_t DXL_ID[12] = { 7, 3, 1, 5, 8, 2, 11, 12, 6, 4, 0, 9 };

const uint8_t DXL_ID_A[6] = { 11, 7, 1, 5, 8, 2 };
const uint8_t DXL_ID_B[6] = { 12, 3, 6, 4, 0, 9 };


const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;


//Variable Declaration
// float k = 0;
// float current = 0; // [mA]
// double f = 100;
// double T = (1000/f); //1sec = 1000ms
float ts = 0.001 * 0.001;
float startFlag = 1;
unsigned long tCount = 0;
unsigned long time_previous, time_current;


//SinWave
float sineWave(float time, float freq, float amp, float phase, float bias) {
  return amp * sin(2 * M_PI * freq * time + phase) + bias;
}

void setup() {
  // put your setup code here, to run once:

  // Use UART port of DYNAMIXEL Shield to debug.
  // DEBUG_SERIAL.begin(115200);
  // DEBUG_SERIAL.begin(234000);
  DEBUG_SERIAL.begin(2000000);


  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  // dxl.begin(57600);
  dxl.begin(4500000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information

  // for (int i = 0; i <= 11; i++) {
  //   dxl.ping(DXL_ID[i]);
  // }

  for (int i = 0; i <= 5; i++) {
    dxl.ping(DXL_ID_A[i]);
    dxl.ping(DXL_ID_B[i]);
  }


  // Turn off torque when configuring items in EEPROM area

  for (int i = 0; i <= 5; i++) {
    dxl.torqueOff(DXL_ID_A[i]);
    dxl.setOperatingMode(DXL_ID_A[i], OP_CURRENT);
    dxl.torqueOn(DXL_ID_A[i]);

    dxl.torqueOff(DXL_ID_B[i]);
    dxl.setOperatingMode(DXL_ID_B[i], OP_CURRENT);
    dxl.torqueOn(DXL_ID_B[i]);
  }

  // for (int i = 0; i <= 11; i++) {
  //   dxl.torqueOff(DXL_ID[i]);
  //   dxl.setOperatingMode(DXL_ID[i], OP_CURRENT);
  //   dxl.torqueOn(DXL_ID[i]);
  // }


  // time_previous = millis();  // 시작시간
}

void loop() {
  // put your main code here, to run repeatedly:
  // time_current = millis();  // 현재시간
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value.
  // // Set Goal Current using RAW unit
  // dxl.setGoalCurrent(DXL_ID, 200);
  // delay(1000);
  // // Print present current
  // DEBUG_SERIAL.print("Present Current(raw) : ");
  // DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_ID));
  // delay(1000);

  //// Set Goal Current using mA unit
  tCount = micros();
  float t = (float)(tCount)*ts;
  // float t = (float)(tCount);
  // float t = 1;

  float current = sineWave(t, 1, 1000, 0, 0);

  // DEBUG_SERIAL.print("current : ");
  // DEBUG_SERIAL.println(current);
  // Serial.print("current  ");
  // Serial.println(current);
  // Serial.println(dxl.setGoalCurrent(DXL_ID, current, UNIT_MILLI_AMPERE));

  // dxl.setGoalCurrent(DXL_ID[0], current, UNIT_MILLI_AMPERE);
  // dxl.setGoalCurrent(DXL_ID[1], current, UNIT_MILLI_AMPERE);


  // for (int i = 0; i <= 11; i++) {
  //   dxl.setGoalCurrent(DXL_ID[i], current, UNIT_MILLI_AMPERE);
  // }

  for (int i = 0; i <= 5; i++) {
    dxl.setGoalCurrent(DXL_ID_A[i], current, UNIT_MILLI_AMPERE);
    dxl.setGoalCurrent(DXL_ID_B[i], current, UNIT_MILLI_AMPERE);
    // delay(30);

  }



  // dxl.setGoalCurrent(DXL_ID, 25.8, UNIT_MILLI_AMPERE);
  // delay(1000);
  // float a = dxl.getPresentCurrent(DXL_ID, UNIT_MILLI_AMPERE);
  // Serial.println(a);

  // DEBUG_SERIAL.print("7_current : ");
  // DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_ID[0], UNIT_MILLI_AMPERE));
  // DEBUG_SERIAL.print("3_current : ");
  // DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_ID[1], UNIT_MILLI_AMPERE));
  // DEBUG_SERIAL.print("1_current : ");
  // DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_ID[2], UNIT_MILLI_AMPERE));
  // DEBUG_SERIAL.print("5_current : ");
  // DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_ID[3], UNIT_MILLI_AMPERE));
  // DEBUG_SERIAL.print("8_current : ");
  // DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_ID[4], UNIT_MILLI_AMPERE));
  // DEBUG_SERIAL.print("2_current : ");
  // DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_ID[5], UNIT_MILLI_AMPERE));



  // DEBUG_SERIAL.print("7_position : ");
  // DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID[0]), UNIT_DEGREE);
  // DEBUG_SERIAL.print("3_position : ");
  // DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID[1]), UNIT_DEGREE);
  // DEBUG_SERIAL.print("1_position : ");
  // DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID[2]), UNIT_DEGREE);
  // DEBUG_SERIAL.print("5_position : ");
  // DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID[3]), UNIT_DEGREE);
  // DEBUG_SERIAL.print("5_position : ");
  // DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID[4]));
  // DEBUG_SERIAL.print("2_position : ");
  // DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID[5]));

  // delay(50);

  // delay(50);
  // delay(1000);

  //// Set Goal Current using percentage (-100.0 [%] ~ 100.0[%])
  // dxl.setGoalCurrent(DXL_ID, -10.2, UNIT_PERCENT);
  // delay(1000);
  // DEBUG_SERIAL.print("Present Current(ratio) : ");
  // DEBUG_SERIAL.println(dxl.getPresentCurrent(DXL_ID, UNIT_PERCENT));
  // delay(1000);
}
