Home made Arduino Battery Monitor

This repository contains the scripts I have written for my Sailing boats battery monitor. Visit https://electrifiedboat.com for more information.

Three branches are listed.
 - main contains the working script I use at the helm station. 
 - feature contains the working script I use in the cabin. This has two additional screens featuring analogue volt and amp readouts.
 - dev is the third branch which contains untested developments, currently code improvement and additional canbus input from motor controller.

HARDWARE:

Arduino Uno clone

SH1106 128x64 OLED

MCP2515 TJ1A050 CANBus Transeiver

Pushbutton and 10kOhm resistor (pull down)

BOAT EQUIPMENT:

Orion Jr. CANbus enabled Battery Managment System

Winston Thunderbird 16x 3,3V 200Ah LiFePo4 cells - 48V

Golden Motor BLDC 10kW electric motor

Golden Motor Vector 500 motor controller (Can be sourced with CANbus)

CANBUS data Identifier List

ID 0x03B BYT0+1:INST_VOLT BYT2+3:INST_AMP BYT4+5:ABS_AMP BYT6:SOC *** ABS_AMP from OrionJr errendous ***

ID 0x6B2 BYT0+1:LOW_CELL BYT2+3:HIGH_CELL BYT4:HEALTH BYT5+6:CYCLES

ID 0x0A9 BYT0:RELAY_STATE BYT1:CCL BYT2:DCL BYT3+4:PACK_AH BYT5+6:AVG_AMP

ID 0x0BD BYT0+1:CUSTOM_FLAGS BYT2:HI_TMP BYT3:LO_TMP BYT4:COUNTER BYT5:BMS_STATUS

** ID for VEC500 controller to be added
