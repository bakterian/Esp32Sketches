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
#define PACKET_SEND_DELAY_MS 50000
#define MOISTURE_SENSOR1_GPIO_NO 34
int counter     = 0;
int halVal      = 0;
int moiValRaw   = 0;
int moiVal      = 0;

int moiCalibAirValue   = 3260;
int moiCalibWaterValue = 1258;


void blinkLED()
{
  digitalWrite(25, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(25, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}

void measureSoilMositure()
{
  //measure soil moisture from GPIO 34 aka. ADC1_6
  //the ADC resolution is 12 bits (reading 3,3 V gives 4095)
  //the ADCs by default have a atteniuation of 11 dB
  //the deafult attenuation is 3.6 (1V input = ADC reading of 3959))
  moiValRaw = analogRead(MOISTURE_SENSOR1_GPIO_NO);
  moiVal = map(moiValRaw,moiCalibWaterValue, moiCalibAirValue, 0, 100);
  //moiVal = moiValRaw;
}

void measureMagneticField()
{
  halVal = hallRead();
}

void sendLoraPacket()
{
  LoRa.beginPacket();
  LoRa.print("Hello #");
  LoRa.print(counter);
  LoRa.print(",Moi: ");
  LoRa.print(moiVal);
  LoRa.print(",Hal: ");
  LoRa.print(halVal);
  LoRa.endPacket();
}

void printOnSerial()
{
  Serial.print("Sending packet: ");
  Serial.println(counter);
  Serial.print("Soil moisture value = ");
  Serial.println(moiVal);
  Serial.print("Hall sensor value = ");
  Serial.println(halVal);
}

void printOnTinyDisplay()
{
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(3,  0, "Send No: ");
  display.drawString(75, 0, String(counter));
  display.drawString(3,  20, "Moi Val: ");
  display.drawString(75, 20, String(moiVal));
  display.drawString(110, 20, "%");
  display.drawString(3, 40, "Hal Val: ");
  display.drawString(75, 40, String(halVal));
  display.display();
}

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


void loop() 
{
  measureMagneticField();
  
  measureSoilMositure();

  printOnSerial();

  printOnTinyDisplay();
  
  //sendLoraPacket();

  blinkLED();
  
  counter++;
  
  //delay(PACKET_SEND_DELAY_MS);
  delay(500);
}
