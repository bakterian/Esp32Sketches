#ifndef _MOSTURE_METER_APP_CONFIG_H_
#define _MOSTURE_METER_APP_CONFIG_H_

#include <freertos/task.h>
#include "esp_task.h"
#include <freertos/event_groups.h>

//OLED pins to ESP32 GPIOs:
//OLED_SDA -- GPIO4
//OLED_SCL -- GPIO15
//OLED_RST -- GPIO16
#define OLED_SDA_PIN 4
#define OLED_SCL_PIN 15
#define OLED_RST_PIN 16

// WIFI_LoRa_32 ports and Config
// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO14 -- SX1278's RESET
// GPIO26 -- SX1278's IRQ(Interrupt Request)
#define LORA_ENABLED 0
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

// BUTTON DEBOUNCE CONFIG
#define DEBOUNCE_TIME_MS 250 // using extream value 250 to be sure that debouncing phase is over
#define DEBOUNCE_TIME_100US (DEBOUNCE_TIME_MS * 10)
#define DEBOUNCE_TIME_10US (DEBOUNCE_TIME_MS * 100)
#define DEBOUNCE_TIME_US (DEBOUNCE_TIME_MS * 1000)
#define LONGPRESS_THRESHOLD_MS 500
#define DISPLAY_RERESH 1000

// GPIOs CONFIG
#define LED_PIN 25
#define BUTTON_1_PIN 13
#define BUTTON_2_PIN 12


// FREE RTOS CONFIG
namespace FreeRtosConfig
{
  const UBaseType_t ButtonTaskPrio        = (ESP_TASK_PRIO_MIN + 4);
  const uint32_t    ButtonTaskStackDepth  = 4092U;
  const uint32_t    ButtonPollingPeriodMs = 3000U;
  const uint8_t     ProCoreID             = 0;
  const uint8_t     AppCoreID             = 1;
  const EventBits_t Button1LowEvent       = 1;
  const EventBits_t Button2LowEvent       = 2;
  const UBaseType_t MaxButtonTrigAmount   = 10;        
} // namespace FreeRtosAppConfig

enum class ButtonPressType
{
  NormalPress = 0,
  LongPress = 1
};

struct ButtonTrigger
{
  uint8_t           Identifier;
  ButtonPressType   PressType;
  uint32_t          Timestamp;
};

struct ButtonTaskParamters
{
  uint8_t Identifier;
  uint8_t GpioNo;
  EventBits_t EventId;
  const char* TaskInformation;
};


static EventGroupHandle_t  g_TaskIntEventGroupHandle = nullptr;
static QueueHandle_t       g_ButtonTriggersQueue = nullptr;
static const ButtonTaskParamters g_Button1Params{1, BUTTON_1_PIN,::FreeRtosConfig::Button1LowEvent, "Callibration button was pressed"};
static const ButtonTaskParamters g_Button2Params{2, BUTTON_2_PIN,::FreeRtosConfig::Button2LowEvent, "Measurment buttton  was pressed"}; 

#endif // _MOSTURE_METER_APP_CONFIG_H_
