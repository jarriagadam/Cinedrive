/* 
 *  Project: Interfacing MFRC522 based RFID Module RC522 with Arudino Uno 
 *  Author: Pranay Sawarkar
 *  Website: www.etechpath.com
 *  MFRC522 Library : https://github.com/ljos/MFRC522 
*/

#include <MFRC522.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#define led_pin 5
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, led_pin, NEO_GRB + NEO_KHZ800);
#define SDAPIN 10 
#define RESETPIN 8 
#define Buzzer 3 

//variables to store data
byte FoundTag; 
byte ReadTag; 
byte TagData[MAX_LEN 16]; 
byte TagSerialNumber[5]; 
byte GoodTagSerialNumber[5] = {0x4C, 0x3B, 0xA0, 0x59}; //Good Tag Number (may different in your case)

MFRC522 nfc(SDAPIN, RESETPIN);

void setup() 
{
pinMode(Buzzer, OUTPUT); 
digitalWrite(Buzzer, LOW);
SPI.begin();
Serial.begin(115200);

//NEO Pixel LED strip setup
strip.begin();
strip.show();

// Start searching RFID Module
Serial.println("Searching for RFID Reader");
nfc.begin();
byte version = nfc.getFirmwareVersion(); // store the reader version in variable

// If can not find RFID Module 
if (! version) { 
Serial.print("Failed to search RC522 board, please check the hardware.");
while(1); //Wait until a RFID Module is found
}

// If found, print the information of detected RFID Module in serial monitor.
Serial.print("RC522 Module Found ");
Serial.println();
Serial.print("Firmware version: 0x");
Serial.println(version, HEX);
Serial.println();
}

//NeoPixel LED animation script 
void colorWipe(uint32_t c, uint8_t wait) 
{
  for(uint16_t i=0; i<strip.numPixels(); i++) 
  {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void loop() {

//Searching for RFID Tag indication.
colorWipe(strip.Color(0, 0, 255), 50); // Blue
colorWipe(strip.Color(0, 0, 0), 50); //OFF

//Detecting good Tag
String GoodTag="False";
FoundTag = nfc.requestTag(MF1_REQIDL, TagData);

if (FoundTag == MI_OK) {
delay(200);

ReadTag = nfc.antiCollision(TagData);
memcpy(TagSerialNumber, TagData, 4);

Serial.println("Tag detected.");
Serial.print("Serial Number: ");
// Loop for printing serial number in serial monitor
for (int i = 0; i < 4; i++) {
Serial.print(TagSerialNumber[i], HEX);
Serial.print(", ");
}
Serial.println("");
Serial.println();


// Check the detected tag number is matching with good tag number or not.
for(int i=0; i < 4; i++){
if (GoodTagSerialNumber[i] != TagSerialNumber[i]) 
{
break; // if not equal, then break out of the "for" loop
}
if (i == 3) { // if we made it to 4 loops then the Tag Serial numbers are matching
GoodTag="TRUE";
} 
}
if (GoodTag == "TRUE"){
Serial.println("TAG Matched ... !");
Serial.println();
//Tag matching indication
colorWipe(strip.Color(0, 255, 0), 50); // Green
colorWipe(strip.Color(0, 0, 0), 50); // OFF
//loop for buzzer tone
for (int y = 0; y < 3; y++){
digitalWrite (Buzzer, HIGH) ;
delay (50) ; 
digitalWrite (Buzzer, LOW) ;
delay (50) ;
}
delay(500);
}
else {
Serial.println("TAG does not Matched .....!");
Serial.println();
//Tag not matching indication
colorWipe(strip.Color(255, 0, 0), 50); // RED
colorWipe(strip.Color(0, 0, 0), 50); // OFF
//loop for buzzer tone
for (int y = 0; y < 3; y++){
digitalWrite (Buzzer, HIGH) ;
delay (300) ;
digitalWrite (Buzzer, LOW) ;
delay (400) ;
}
delay(500); 
}
}
}
