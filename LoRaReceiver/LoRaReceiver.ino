#include <SPI.h>
#include <LoRa.h>
#include "SSD1306.h"
#include<Arduino.h>

//OLED pins to ESP32 GPIOs via this connecthin:
#define OLED_SDA  4
#define OLED_SCL  15
#define OLED_RST  16
#define OLED_I2C_ADDR 0x3C

// WIFI_LoRa_32 ports
#define SCK     5
#define MISO    19
#define MOSI    27
#define CS      18

#define RST     14
#define DI0     26

#define BAND    868300000
#define SF7     7 //Spreading factor 7 is used
#define BW      125E3
#define CR_DEN  5 //Coding rate denominator, the nominator is fixed and set to 4
#define PR_LEN  8 //Preamble lenght set to 8
#define SW      0x34  //The sync word is 0x34 making this a public network
#define MAX_PAYLOAD_SIZE 0x40
#define PACKET_SEND_DELAY_MS 10000

#define DEBUG 1
#define LORA_CALLBACK_MODE 1

unsigned int _counter = 1;
volatile int _rxPkgSize = 0;
char _rxPayload[MAX_PAYLOAD_SIZE];

SSD1306  display(OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

void BlinkLED()
{
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(500);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(500);                       // wait for a second
}

void InitLoRaPins()
{
  pinMode(CS, OUTPUT);
  pinMode(RST, OUTPUT);
  pinMode(DI0, INPUT);                // This pin is interrupt

  LoRa.setPins(CS,RST,DI0);
}
void InitLoRaPayloadBuffer()
{
  memset(_rxPayload,0x00, MAX_PAYLOAD_SIZE * sizeof(char));
}

void FlushLoraPayload()
{
  int i = 0;
  while (LoRa.available()) 
  {
    _rxPayload[i++] = LoRa.read();
  }
  _rxPkgSize = i;
}

void InitOled()
{
  pinMode(LED_BUILTIN,OUTPUT); //Send success, LED will bright 1 second
  pinMode(OLED_RST,OUTPUT);
  digitalWrite(OLED_RST, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(OLED_RST, HIGH);
  
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(5,5,"LoRa Reader");
  display.display();
}

void DisplayReadCounter(int packetSize, int counterVal)
{
  auto packetRssi = LoRa.packetRssi();
  auto packetSNR = LoRa.packetSnr();
  
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
      
  display.drawString(0, 0, "Packet No:");
  display.drawString(80, 0, String(counterVal)); 
  
  display.drawString(0, 16, "RSSI:" );
  display.drawString(50, 16, String(packetRssi));
  display.drawString(80, 16, ",L:");
  display.drawString(100, 16, String(packetSize));

  display.drawString(0, 32, "SNR:" );
  display.drawString(50, 32, String(packetSNR) );
  display.drawString(0, 48, String(_rxPayload));
  display.display();

  #ifdef DEBUG
  Serial.print("\nReceived packet nr: ");
  Serial.println(counterVal, DEC);
  
  Serial.print("RSSI: ");
  Serial.print(packetRssi);

  Serial.print(", SNR: ");
  Serial.print(packetSNR);

  Serial.print(", Length: ");
  Serial.println(packetSize);

  // read packet
  for(int i = 0; i < packetSize; ++i) 
  {
    Serial.print(" 0x");
    Serial.print((char)_rxPayload[i],HEX);
  }
  Serial.print("\nASCII representation:");
  Serial.print(String(_rxPayload));
  #endif
}

void onLoraPkgReceive(int packetSize)
{
   FlushLoraPayload();
}

void setup() 
{
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("LoRa Receiver");

  InitLoRaPins();
  InitLoRaPayloadBuffer();
  
  InitOled();

  SPI.begin(SCK,MISO,MOSI,CS);
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

  #if LORA_CALLBACK_MODE >= 1
  // register the receive callback
  //attachInterrupt(digitalPinToInterrupt(DI0),IntHandler,RISING);
  LoRa.onReceive(onLoraPkgReceive);

  // put the radio into receive mode
  LoRa.receive();
#endif
}

void loop() 
{
  #if LORA_CALLBACK_MODE == 0
  // try to parse packet
  if(LoRa.parsePacket())
  {
    FlushLoraPayload();
  }
  #endif
  
  if (_rxPkgSize)
  {
    // received a packet
    DisplayReadCounter(_rxPkgSize, _counter++);
    BlinkLED();
    _rxPkgSize = 0;
  }

}
