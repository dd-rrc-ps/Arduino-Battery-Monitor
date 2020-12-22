// Display CANBus data from battery system BMS to OLED displays controlled by pushbutton
//
// Short press of the single pushbutton to change between pages and long press to turn display off.
//
// First page indicates Battery SOC, Endurance, Low charge, Charger On and Power consumption.
// Second page indicates Pack voltage, Low and High cell voltage, pack Health and Current to and from pack.
// Third page is the diagnosis page with Relay status, Current limit, Fault flags, BMS rolling counter (to see that data is flowing), pack Cycles, pack average Temperature and pack Ampere hours remaining.
//
// WRITTEN BY:
// Tom Storebø 8. april 2019
//
// Change log
// 16/04/19  Added "else if" to many "if"s and changed the text behind the "#define CAN0_INT 9" to reflect the right pin configuration. Also added "&&" limits to gauge wattage indications, health and amp bar, for right positioning.
// 18/04/19  Changed "float" to "int" on rV, lC and hC after adding two decimal points to calculations. Added adjustments to Amp bar at +/- 0-9A. Set bars to demo mode.
// 30/04/19  Changed current negative values to indicate charge. Change button held for longer than 1,5s.
// 01/05/19  Added decimal to amp reading between below 10 and above -10 amp.
// 02/05/19  Changed charge warning symbol to appear at or below 25% if discharge detected; corrected Relay state bit error; Allign AH < 100.0 in text page.
// 03/05/19  Updated the BMS shunt from 50A to 500A. This changed the "rA" 10 times which had to be reflected in the code.
//
// Future revisions to code
//
// Sketch 23412 bytes
//
// HARDWARE:
// Arduino Uno clone
// SH1106 128x64 OLED
// MCP2515 TJ1A050 CANBus Transeiver
// Pushbutton and 10kOhm resistor

#include <U8g2lib.h>
#include <mcp_can.h>
#include <SPI.h>    // SPI library CANBUS
#include <Wire.h>   // I2C library OLED

// OLED library driver
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// CANBUS Shield pins
#define CAN0_INT 9                              // Set INT to pin 9
MCP_CAN CAN0(10);                               // Set CS to pin 10

// Debugging
int debug = 0;

// MCP_CAN DATA
long unsigned int rxId;     // Stores 4 bytes 32 bits
unsigned char len = 0;      // Stores at least 1 byte
unsigned char rxBuf[8];     // Stores 16 bytes, 1 character  = 2 bytes

// CANBUS Data Identifier List
// ID 0x03B BYT0+1:INST_VOLT BYT2+3:AMPERE BYT4+5:SOC
// ID 0x6B2 BYT0+1:LOW_CELL BYT2+3:HIGH_CELL BYT4+5:HEALTH
// ID 0x0A9 BYT0:RELAY_STATE BYT1+2:CCL BYT3+4:DCL BYT5:AVG_TMP BYT6+7:PACK_AH
// ID 0x0BD BYT0+1:CUSTOM_FLAGS BYT2+3:CYCLES BYT4:ROLLING_COUNTER

// Variables
int rV;       // Voltage - multiplied by 10
float rA;     // Current - multiplied by 10 - negative value indicates charge
int sc;       // State of charge - multiplied by 2
int s;        // Faults seen variable
int angle = 0;// Needle angle
int p;        // Watt reading
int m = 10;
int contrast = 80;
u8g2_uint_t xx = 0;

// Button settings
const int buttonPin = 2;  // Pin assigned for button
long millis_held;         // 4 byte variable for storing duration button is held down
unsigned long firstTime;  // 4 byte variable for storing time button is first pushed
byte previous = HIGH;     // Pin state before pushing or releasing button
int hits;                 // Variable for how many times button has bin pushed
int buttonState;          // Variable for button pushed or not


// ------------------------------------------------- void setup() ------------------------------------------

void setup() {

  // Start serial monitor communication
  if (debug == 1) {
    Serial.begin(9600);
  }
  else {
    Serial.begin(9600);
    Serial.println("Debugging Off");
    Serial.end();
  }

   // Initialise MCP2515 running at 8MHz and baudrate 250kb/s
  CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ);

  CAN0.setMode(MCP_NORMAL);
  
  // Configure pin for INT input
  pinMode(CAN0_INT, INPUT);

  // Starts display
  u8g2.begin();

  // Set contrast level
  u8g2.setContrast(contrast);

  // Set standard font
  u8g2.setFont(u8g2_font_chikita_tf);
 
}

// -------------------------------------------------- void off() -------------------------------------------

void off() {
  
  // Puts screen to sleep
  u8g2.setPowerSave(1);
   
  // Complicated and perhaps unecessary to do
  //CAN0.setMode(MCP_SLEEP);
}

// ------------------------------------------------- void gauge() ------------------------------------------

void gauge(uint8_t angle) {

  // Fault messages from CANBus for displaying wrench icon
  int fu;

  // Sort CANBus data buffer
  if(rxId == 0x03B) {
    rV = ((rxBuf[0] << 8) + rxBuf[1]);
    rA = ((rxBuf[2] << 8) + rxBuf[3]);
  }
  if(rxId == 0x0BD) {
    fu = ((rxBuf[0] << 8) + rxBuf[1]);
  }
  
  // Map needle movement
  m = map(p,0,10000,0,90);

  // Needle value calculation
  p = (abs((rA*rV)/100));
  
  // Display dimensions
  int xmax = 128;
  int ymax = 64;
  int xcenter = xmax/2;
  int ycenter = ymax/2+10;
  int arc = ymax/2;

  // Turn screen On after sleep
  u8g2.setPowerSave(0);
  
  // Draw border of the gauge
  u8g2.drawCircle(xcenter, ycenter, arc+6, U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(xcenter, ycenter, arc+4, U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(xcenter, ycenter, arc+6, U8G2_DRAW_UPPER_LEFT);
  u8g2.drawCircle(xcenter, ycenter, arc+4, U8G2_DRAW_UPPER_LEFT);

  // Draw the needle
  float x1 = sin(2*angle*2*3.14/360);
  float y1 = cos(2*angle*2*3.14/360); 
  u8g2.drawLine(xcenter, ycenter, xcenter+arc*x1, ycenter-arc*y1);
  u8g2.drawDisc(xcenter, ycenter, 5, U8G2_DRAW_UPPER_LEFT);
  u8g2.drawDisc(xcenter, ycenter, 5, U8G2_DRAW_UPPER_RIGHT);
 
  // Draw scale labels
  u8g2.drawStr(20, 42, "0");                   
  u8g2.drawStr(24, 18, "25");
  u8g2.drawStr(60, 14, "50");
  u8g2.drawStr(95, 18, "75");
  u8g2.drawStr(105, 42, "100");

  // Draw gauge label
  u8g2.drawStr(45, 32, "% POWER");

  // Draw unit description
  u8g2.drawStr(88, 60, " WATT");
     
  // Draw digital value and align its position
  u8g2.setFont(u8g2_font_profont22_tn);             
  u8g2.setCursor(65,60);
  // Draw leading 0 when values below 10W
  if (p < 10){
    u8g2.print("0");
  }
  // Shift position of values above 99W
  if (p > 99 && p <= 999) {
    u8g2.setCursor(53, 60);
  }
  // Shift position of values above 999W
  else if (p > 999 && p <= 9999) {
    u8g2.setCursor(41, 60);
  }
  // Shift position of values above 9999W
  else if (p > 9999) {
    u8g2.setCursor(29,60);
  }
  u8g2.print(p);
  
  // SOC from CANBus data buffer
  if (rxId == 0x03B) {
    sc = ((rxBuf[4] << 8) + rxBuf[5]);
  }
  
  // Draw lightening bolt when charge current above 5A
  if (rA < -50) {
    u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
    u8g2.drawGlyph(4, 40, 67);
  }
  // Draw sun when charge current at or below 5A
  if (rA < 0 && rA >= -50) {
    u8g2.setFont(u8g2_font_open_iconic_weather_2x_t);
    u8g2.drawGlyph(4, 39, 69);
  }
  
  // Draw warning symbol at and below 25% State of Charge if discharging
  if (sc <= 50 && rA > 0) {   // soc from canbus is multiplied by 2
    u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
    u8g2.drawGlyph(4, 39, 71);
  }

  // Draw wrench icon if BMS faults have not been seen
  if (fu != s) {
    u8g2.setFont(u8g2_font_open_iconic_embedded_2x_t);
    u8g2.drawGlyph(3, 62, 72);
  }
  
  // Clock calculations
  int hD = (sc) / abs(rA/10);
  int mD = ((sc) / abs(rA/10) - hD)*60;
  int hC = (200-sc)/abs(rA/10);
  int mC = ((200-sc)/abs(rA/10) - hC)*60;

  // Draw battery icon
  u8g2.drawFrame( 1, 0, 22, 9);
  u8g2.drawFrame( 23, 2, 2, 5);
  u8g2.drawBox( 3, 2, sc/2*0.18, 5);

  // Draw state of charge
  u8g2.setCursor(4,16);
  u8g2.setFont(u8g2_font_chikita_tf);
  u8g2.print(sc/2); u8g2.print('%');
  
  // Draw clock
    if (rA > 0) {
    char tD[11];
    sprintf(tD, "%02d:%02dhrs", hD, mD);
    if (hD > 99) {
      u8g2.setCursor(86, 5);
    }
    else {
      u8g2.setCursor(88, 5);
    }
    u8g2.print(tD);
  }
  else {
    char tC[11];
    sprintf(tC, "%02d:%02dhrs", hC, mC);
    u8g2.setCursor(88, 5);
    u8g2.print(tC);
  }
}
 
// ---------------------------------------------- void bars() -------------------------------------------------------

void bars() {
  
  // Variables from CANBus
  int lC;       // Low Cell Voltage
  int hC;       // High Cell Voltage
  int h;        // Health

  // Sort CANBus data buffer
  if(rxId == 0x03B) {
    rV = ((rxBuf[0] << 8) + rxBuf[1]);
    rA = ((rxBuf[2] << 8) + rxBuf[3]);
  }
  if(rxId == 0x6B2) {
    lC = ((rxBuf[0] << 8) + rxBuf[1]);
    hC = ((rxBuf[2] << 8) + rxBuf[3]);
    h = ((rxBuf[4] << 8) + rxBuf[5]);
  }

  // Draw pack volt bar
  int pV = ((rV/10.0-43.2)*2.083); // Box length 35/16.8 (volt difference max to min)
  u8g2.setCursor(2, 5);
  u8g2.print(rV/10.0, 1); // One decimal
  u8g2.drawStr(1, 56, "Pack");
  u8g2.drawStr(2, 63, "Volt");
  u8g2.drawFrame(5, 8, 11, 38);
  // Box can't be drawn with negative value
  if (pV > 0) {
    u8g2.drawBox(7, 44-pV, 7, pV);
  }
  
  // Draw Min and Max cell voltage bars
  int lCb = ((lC/1000.00)-2.7)*26.9;
  int hCb = ((hC/1000.00)-2.7)*26.9;
  u8g2.setCursor(28, 5);
  u8g2.print(lC/1000.00, 2); // Two decimals
  u8g2.drawStr(28, 56, "Low");
  u8g2.drawStr(29, 63, "Cell");
  u8g2.drawFrame(31, 8, 11, 38);
  // Box can't be drawn with negative value
  if (lCb > 0) {
    u8g2.drawBox(33, 44-lCb, 7, lCb);
  }
  u8g2.setCursor(54, 5);
  u8g2.print(hC/1000.00, 2);  // Two decimals
  u8g2.drawStr(54, 56, "High");
  u8g2.drawStr(55, 63, "Cell");
  u8g2.drawFrame(57, 8, 11, 38);
  // Box can't be drawn with negative value
  if (hCb > 0) {
    u8g2.drawBox(59, 44-hCb, 7, hCb);
  }
  
  // Draw health bar
  int hBar = h*0.34;
  if (h >= 0 && h <= 9) {
    u8g2.setCursor(88, 5);  // Shift position at and above 0%
  }
  else {                    // Shift position above 9%
    u8g2.setCursor(84, 5);
  }
  u8g2.print(h);
  u8g2.drawStr(76, 56, "Health");
  u8g2.drawStr(86, 63, "%");
  u8g2.drawFrame(84, 8, 11, 38);
  if (h > 0) {
    u8g2.drawBox(86, 44-hBar, 7, hBar);
  }

  // Draw ampere bar
  float aBar = abs(rA)*0.0174;
  if (rA >= 0 && rA <= 90) {             // Shift position at and above 0A
    u8g2.setCursor(111, 5);
  }
  else if (rA > 90 && rA <200) {          // Shift position above 9A
    u8g2.setCursor(113, 5);
  }
  else if (rA > 190 && rA <=999) {        // Shift position above 19A
    u8g2.setCursor(111, 5);
  }  
  else if (rA > 999) {                   // Shift position above 99A
    u8g2.setCursor(110, 5);
  }
  else if (rA < 0 && rA >= -99) {        // Shift position below 0A
    u8g2.setCursor(105, 5);
  }
  else if (rA < -99 && rA > -200) {       // Shift position below -9A
    u8g2.setCursor(107, 5);
  }
  else if (rA < -199 && rA >= -999) {     // Shift position below -19A
    u8g2.setCursor(106, 5);
  }
  else if (rA < -999) {                  // Shift position below -99A
    u8g2.setCursor(104, 5);
  }
  if (rA < 100 && rA > -100) {            // Prints single decimal above -10A and below 10A
    u8g2.print(rA/10, 1);
  }
  else {                                // Prints amp without decimal at and above 10A and at and below -10A
    u8g2.print(rA/10, 0);
  }
  u8g2.drawStr(108, 56, "Amp");
  u8g2.drawFrame(111, 8, 11, 38);
  // No bar displayed below 6A
  if (abs(rA) > 60) {
    u8g2.drawBox(113, 45-aBar, 7, aBar);
  }
}
// ------------------------------------------------- void text() ----------------------------------------------------

void text() {

  // Variables from CANBus
  int fu;       // BMS faults
  byte t;       // Average cell temperature
  int cc;       // Cycles
  float ah;     // Amp hours
  byte ry;      // Relay status
  int ccl;      // Charge current limit
  int dcl;      // Discharge current limit
  byte ct;      // Counter from 0-256 to check BMS or CANBus refreshing

  // Sort CANBus data buffer
  if(rxId == 0x0A9) {
    ry = (rxBuf[0]);
    ccl = ((rxBuf[1] << 8) + rxBuf[2]);
    dcl = ((rxBuf[3] << 8) + rxBuf[4]);
    t = (rxBuf[5]);
    ah = ((rxBuf[6] << 8) + rxBuf[7]);
  }
  if(rxId == 0x0BD) {
    fu = ((rxBuf[0] << 8) + rxBuf[1]);
    cc = ((rxBuf[2] << 8) + rxBuf[3]);
    ct = (rxBuf[4]);
  }

  // Saves fault codes to 's' after reviewing text page
  s = fu;
  
  // Draw horisontal lines
  u8g2.drawHLine(0, 7, 128); u8g2.drawHLine(0, 37, 66); u8g2.drawHLine(0, 46, 128); u8g2.drawHLine(66, 55, 128); u8g2.drawHLine(99, 26, 128); u8g2.drawHLine(99, 37, 128);
  
  // Draw vertical lines
  u8g2.drawVLine(66, 0, 64); u8g2.drawVLine(99, 0, 64);
    
  // Draw relay status
  u8g2.drawStr(0, 5, "Relay Status");
  u8g2.drawStr(0, 16, "Discharge");
  u8g2.setFont(u8g2_font_open_iconic_check_1x_t);
  if ((ry & 0b00000001) == 0b00000001) {
    u8g2.drawGlyph(55, 18, 64);
  }
  else {
    u8g2.drawGlyph(55, 18, 68);
  }
  u8g2.setFont(u8g2_font_chikita_tf);
  u8g2.drawStr(0, 25, "Charge");
  u8g2.setFont(u8g2_font_open_iconic_check_1x_t);
  if ((ry & 0b00000010) == 0b00000010) {
    u8g2.drawGlyph(55, 27, 64);
  }
  else {
    u8g2.drawGlyph(55, 27, 68);
  }
  u8g2.setFont(u8g2_font_chikita_tf);
  u8g2.drawStr(0, 34, "Chg Safety");
  u8g2.setFont(u8g2_font_open_iconic_check_1x_t);
  if ((ry & 0b00000100) == 0b00000100) {
    u8g2.drawGlyph(55, 36, 64);
  }
  else {
    u8g2.drawGlyph(55, 36, 68);
  }

  // Draw current limits
  u8g2.setFont(u8g2_font_chikita_tf);
  u8g2.drawStr(0, 44, "Current Limit");
  u8g2.setCursor(0, 55);
  u8g2.print("DCH        "); u8g2.print(dcl); u8g2.print(" A");
  u8g2.setCursor(0, 64);
  u8g2.print("CHG        "); u8g2.print(ccl); u8g2.print(" A");
  
  // Draw faults and count
  int z = 68;         // x position for flags
  int y = 0;
  u8g2.drawStr(69, 5, "Faults   Count");

  // Flag internal communication fault
  if (((fu & 0x0100) == 0x0100) && y <= 28) {
    u8g2.drawStr(z, 16+y, "intCom");
    y += 7;
  }
  // Flag internal convertions fault
  if (((fu & 0x0200) == 0x0200) && y <= 28) {
    u8g2.drawStr(z, 16+y, "inConv");
    y += 7;
  }
  // Flag weak cell fault
  if (((fu & 0x0400) == 0x0400) && y <= 28) {
    u8g2.drawStr(z, 16+y, "wkCell");
    y += 7;
  }
  // Flag low cell fault
  if (((fu & 0x0800) == 0x0800) && y <= 28) {
    u8g2.drawStr(z, 16+y, "lowCell");
    y += 7;
  }
  // Flag open wire fault
  if (((fu & 0x1000) == 0x1000) && y <= 28) {
    u8g2.drawStr(z, 16+y, "opnWre");
    y += 7;
  }
  // Flag current sense fault
  if (((fu & 0x2000) == 0x2000) && y <= 28) {
    u8g2.drawStr(z, 16+y, "crrSns");
    y += 7;
  }
  // Flag volt sense fault
  if (((fu & 0x4000) == 0x4000) && y <= 28) {
    u8g2.drawStr(z, 16+y, "vltSns");
    y += 7;
  }
  // Flag volt redundancy fault
  if (((fu & 0x8000) == 0x8000) && y <= 28) {
    u8g2.drawStr(z, 16+y, "vltRcy");
    y += 7;
  }
  // Flag weak pack fault
  if (((fu & 0x0001) == 0x0001) && y <= 28) {
    u8g2.drawStr(z, 16+y, "wkPack");
    y += 7;
  }
  // Flag thermistor fault
  if (((fu & 0x0002) == 0x0002) && y <= 28) {
    u8g2.drawStr(z, 16+y, "xThrm");
    y += 7;
  }
  // Flag charge limit enforcement fault
  if (((fu & 0x0004) == 0x0004) && y <= 28) {
    u8g2.drawStr(z, 16+y, "chgEnf");
    y += 7;
  }
  // Flag discharge limit enforcement fault
  if (((fu & 0x0008) == 0x0008) && y <= 28) {
    u8g2.drawStr(z, 16+y, "dchEnf");
    y += 7;
  }
  // Flag charge safety relay fault
  if (((fu & 0x0010) == 0x0010) && y <= 28) {
    u8g2.drawStr(z, 16+y, "chSenf");
    y += 7;
  }
  // Flag internal memory fault
  if (((fu & 0x0020) == 0x0020) && y <= 28) {
    u8g2.drawStr(z, 16+y, "intMem");
    y += 7;
  }
  // Flag internal thermistor fault
  if (((fu & 0x00040) == 0x0040) && y <= 28) {
    u8g2.drawStr(z, 16+y, "intThm");
    y += 7;
  }
  // Flag internal logic fault
  if (((fu & 0x0080) == 0x0080) && y <= 28) {
    u8g2.drawStr(z, 16+y, "intLog");
    y += 7;
  }

  // Draw count
  if (ct < 10) {
    u8g2.setCursor(112, 16);
  }
  else if (ct >= 10) {
    u8g2.setCursor(109, 16);
  }
  else if (ct >= 100) {
    u8g2.setCursor(108, 16);
  }
  else if (ct >= 200) {
    u8g2.setCursor(106, 15);
  }
  u8g2.print(ct);
  u8g2.setFont(u8g2_font_open_iconic_embedded_1x_t);
  u8g2.drawGlyph(101, 25, 79);
  u8g2.setFont(u8g2_font_chikita_tf);
  u8g2.drawStr(110, 24, "256");
  

  // Draw pack cycles
  u8g2.setCursor(101, 34);
  u8g2.print("Cycles");
  if (cc < 10) {
    u8g2.setCursor(112, 44);
  }
  else if (cc >= 10) {
    u8g2.setCursor(109, 44);
  }
  else if (cc >= 100) {
    u8g2.setCursor(108, 44);
  }
  u8g2.print(cc);

  // Draws pack average temp and ampere/hours
  u8g2.drawStr(72, 53, "Temp       Ah");
  if (t < 10) {
    u8g2.setCursor(81, 64);
  }
  else {
    u8g2.setCursor(78, 64);
  }
  u8g2.print(t);
  if (ah < 10000) {
    u8g2.setCursor(106,64);
  }
  else {
  u8g2.setCursor(104,64);
  }
  u8g2.print(ah/100, 1);
}

// ------------------------------------------------- void loop() ------------------------------------------

void loop() {

  // Read MCP2515
  if(!digitalRead(CAN0_INT)) {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
  }
  
  // show needle and dial
  // 135 = zero position, 180 = just before middle, 0 = middle, 45 = max
  xx = m;
  // position correction
  if (xx<45){
    xx=xx+135;
  }
  else {
    xx=xx-45;
  }

  // Check the status of the button
  buttonState = digitalRead(buttonPin);

  // How long is the button held down
  if (buttonState == HIGH && previous == LOW) {
    firstTime = millis();
  }
  if (buttonState == LOW && previous == HIGH) {
    millis_held = millis() - firstTime;
  }

  // Require more than 200ms push to qualify as a button "hit"
  if (millis_held > 200) {
    if (buttonState == LOW && previous == HIGH) {

      // Long push up to 0,5 sec change contrast
      if (millis_held >= 500 && millis_held < 1500) {
        if (contrast == 80) {
          contrast = 160;
        }
        else if (contrast == 160) {
          contrast = 255;
        }
        else {
          contrast = 80;
        }
      }

      // Long push above 1,5 sec turn screen off
      if (millis_held >= 1500) {
        hits = 0;
      }
      // Short button press changes between pages
      if (millis_held < 500) {
        if (hits < 3) {
          hits += 1;  // adds 1 to hits
        }
        else {
          hits = 1;
        }
      }
    }
    // Save button state
    previous = buttonState;
  }

  // Display gauge page
  if (hits == 1) {
    u8g2.firstPage(); 
    do {             
      gauge(xx);
    }
    while( u8g2.nextPage() );
  }

  // Display bars page
  if (hits == 2) {
    u8g2.firstPage();
    do {
      bars();
    }
    while(u8g2.nextPage());
  }

  // Display text page
  if (hits == 3) {
    u8g2.firstPage();
    do {
      text();
    }
    while(u8g2.nextPage());
  }
  // Put to sleep
  if (hits == 0) {
    u8g2.firstPage();
    do {
      off();
    }
    while(u8g2.nextPage());
  }
}
