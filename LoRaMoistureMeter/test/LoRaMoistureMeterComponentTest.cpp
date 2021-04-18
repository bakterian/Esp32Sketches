// Copyright 2014 http://switchdevice.com
// This example code is in the public domain.

#include "Arduino.h"
#include "Serial.h"

#include "../LoRaMoistureMeter.ino"

using ::testing::Return;
TEST(loop, pushed) 
{
  
  ArduinoMock*  arduinoMock = arduinoMockInstance();
  StringMock*   stringMock = StringMockInstance();
  SerialMock*   serialMock = serialMockInstance();
  LoRaMock*     loRaMock = loRaMockInstance();
  FreeRtosMock* freertosMock = freeRtosMockInstance();
  SSD1306WireMock* ssd1306WireMock = SSD1306WireMockInstance();

  /*
  EXPECT_CALL(*arduinoMock, digitalRead(2))
    .WillOnce(Return(1));
  EXPECT_CALL(*serialMock, println(1, 10));
  EXPECT_CALL(*arduinoMock, delay(1));
  */
  //loop();

  blinkLED();

  releaseSSD1306WireMock();
  releaseFreeRtosMock();
  releaseLoRaMock();
  releaseSerialMock();
  releaseStringMockInstance();
  releaseArduinoMock();
}
