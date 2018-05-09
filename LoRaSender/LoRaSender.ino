#include <SPI.h>
#include <LoRa.h>
#include "SSD1306.h"
#include<Arduino.h>
 
//OLED pins to ESP32 GPIOs via this connecthin:
//OLED_SDA -- GPIO4
//OLED_SCL -- GPIO15
//OLED_RST -- GPIO16
 
SSD1306  display(0x3c, 4, 15);
 
// WIFI_LoRa_32 ports
// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO14 -- SX1278's RESET
// GPIO26 -- SX1278's IRQ(Interrupt Request)
 
#define SS      18
#define RST     14
#define DI0     26
#define BAND    868300000
#define SF7     7 //Spreading factor 7 is used
#define BW      125E3
#define CR_DEN  5 //Coding rate denominator, the nominator is fixed and set to 4
#define PR_LEN  8 //Preamble lenght set to 8
#define SW      0x34  //The sync word is 0x34 making this a public network
#define PACKET_SEND_DELAY_MS 10000
int counter = 0;
 
void setup() {
  pinMode(25,OUTPUT); //Send success, LED will bright 1 second
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH);
   
  Serial.begin(115200);
  while (!Serial); //If just the the basic function, must connect to a computer
  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(5,5,"LoRa Sender");
  display.display();
   
  SPI.begin(5,19,27,18);
  LoRa.setPins(SS,RST,DI0);
  Serial.println("LoRa Sender");


  
  LoRa.setPins(SS,RST,DI0);
  LoRa.setSpreadingFactor(SF7);
  LoRa.setSignalBandwidth(BW);
  LoRa.setCodingRate4(CR_DEN);
  LoRa.setPreambleLength(PR_LEN);
  LoRa.setSyncWord(SW);
  LoRa.enableCrc();
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  
  Serial.println("LoRa Initial OK!");
  display.drawString(5,20,"LoRa Initializing OK!");
  display.display();
  delay(2000);
}
void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(3, 5, "Sending packet ");
  display.drawString(50, 30, String(counter));
  display.display();
  // send packet
  LoRa.beginPacket();
  LoRa.print("Hello..");
  LoRa.print(counter);
  LoRa.endPacket();
  counter++;
  digitalWrite(25, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(25, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
  delay(PACKET_SEND_DELAY_MS);
}
