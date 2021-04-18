#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include "SSD1306Wire.h"
#include <Arduino.h>
#include "AppConfig.h"

 
SSD1306Wire  display(0x3c, 4, 15);

// ##### Moisture Measurment variables
int counter     = 0;
int halVal      = 0;
int moiValRaw   = 0;
int moiVal      = 0;
int moiCalibAirValue   = 3260;
int moiCalibWaterValue = 1258;

// ########### MOISURE MEAS LOGIC ##############
void measureSoilMositure()
{
  //measure soil moisture from GPIO 34 aka. ADC1_6
  //the ADC resolution is 12 bits (reading 3,3 V gives 4095)
  //the ADCs by default have a atteniuation of 11 dB
  //the deafult attenuation is 3.6 (1V input = ADC reading of 3959))
  moiValRaw = analogRead(MOISTURE_SENSOR1_GPIO_NO);
  moiVal = map(moiValRaw,moiCalibWaterValue, moiCalibAirValue, 0, 100);
}
void measureMagneticField()
{
  halVal = hallRead();
}
// ############################################


// ################ LORA Logic ################
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
void configureLora()
{
  #if LORA_ENABLED
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
  #endif
}
// ########################################


// ############ UTILITIES #################
void blinkLED()
{
  digitalWrite(LED_PIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                  // wait for a second
  digitalWrite(LED_PIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                  // wait for a second
}
void configureGPIOs()
{
  pinMode(LED_PIN,OUTPUT);
  pinMode(OLED_RST_PIN,OUTPUT);
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
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
void oledDispalyReset()
{
  digitalWrite(OLED_RST_PIN, LOW);
  delay(50); 
  digitalWrite(OLED_RST_PIN, HIGH);
}
void oledConfigure()
{
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(5,5,"LoRa Sender");
  display.display();
}
void serialConfigure()
{
  Serial.begin(115200);
  while (!Serial);
}
void displayInitSuccessMsg()
{
  Serial.println("Moisture Meter Init OK!");
  display.drawString(5,20,"LoRa Initializing OK!");
  display.display();
  delay(2000);
}
// Thanks to the IRAM_ATTR attribute function is put to RAM (faster access)
void IRAM_ATTR button1InterruptHanlder()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  auto result = xEventGroupSetBitsFromISR( g_TaskIntEventGroupHandle, ::FreeRtosConfig::Button1LowEvent, &xHigherPriorityTaskWoken );

  /* pdFALSE to be reutned if the command could not be written to the timer command queue as it was aleady full. 
     This is not expected and acceptable at all! */
  assert( result == pdTRUE );

 if (xHigherPriorityTaskWoken) 
 {
    portYIELD_FROM_ISR();
 }
}
void IRAM_ATTR button2InterruptHanlder()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  auto result = xEventGroupSetBitsFromISR( g_TaskIntEventGroupHandle, ::FreeRtosConfig::Button2LowEvent, &xHigherPriorityTaskWoken );

  /* pdFALSE to be reutned if the command could not be written to the timer command queue as it was aleady full. 
     This is not expected and acceptable at all! */
  assert( result == pdTRUE );

 if (xHigherPriorityTaskWoken) 
 {
    portYIELD_FROM_ISR();
 }
}
void configureButtonInterrupts()
{
  const auto interruptNoBut1 = digitalPinToInterrupt(BUTTON_1_PIN);
  attachInterrupt(interruptNoBut1, button1InterruptHanlder, FALLING);
  
  const auto interruptNoBut2 = digitalPinToInterrupt(BUTTON_2_PIN);
  attachInterrupt(interruptNoBut2, button2InterruptHanlder, FALLING);
}
void displayButtonStats(uint32_t butttonId)
{
   Serial.print("Button #");
   Serial.print(butttonId);
   Serial.println(" is pressed.");
     
   display.clear();
   display.setFont(ArialMT_Plain_16);
   display.drawString(3,  0, "Ints: ");
   display.drawString(75, 0,  String(1));
   display.drawString(3,  20, "PRES");
   display.drawString(3,  40, "T: ");
   display.drawString(75, 40, String(millis()));
   display.display();
}
void displayButtonTriggerInfos(const ButtonTrigger& buttonTrigger)
{
  const char* butonPressInfo[2]{"Normal Press", "Long Press"};
  Serial.println("Button press identified:");
  Serial.print("Button No: ");
  Serial.println(buttonTrigger.Identifier);
  Serial.print("Button Type:  ");
  Serial.println(butonPressInfo[(uint32_t)buttonTrigger.PressType]);
  Serial.print("Timestamp: ");
  Serial.println(buttonTrigger.Timestamp);   
}
void buttonProcessingTask(void *pvParameters)
{
  const ButtonTaskParamters* taskParameters = reinterpret_cast<ButtonTaskParamters*>(pvParameters);
  const BaseType_t xClearOnExit    = pdFALSE;
  const BaseType_t xWaitForAllBits = pdTRUE;
  const TickType_t xTicksToWait    = portMAX_DELAY;
  EventBits_t      retEventBits    = 0;
  bool  initialButtonRebounce      = true;
  uint32_t buttonPress1msTicks     = 0U;

  Serial.print("Task no ");
  Serial.print(taskParameters->Identifier);
  Serial.println(" started");
  
  for(;;)
  {
    // after this line the task will likely enter the block state for a longer while:
    retEventBits = xEventGroupWaitBits(g_TaskIntEventGroupHandle, taskParameters->EventId, xClearOnExit, xWaitForAllBits, xTicksToWait);
  
    // The following if-check becomes more usefull if xTicksToWait is not set to portMAX_DELAY
    // as unblocking of this task may happen due to a timeout.
    assert((retEventBits & taskParameters->EventId) != 0 );

    if(initialButtonRebounce)
    {//initial button-press processing
       //interrupt detach is an option here: detachInterrupt(PIN_BUTTON)
       //do a naive debouncing (in testing longer bouncing than 3 ms was not observed)
       delay(10);

       // TODO: display short summary only
       //Serial.println(taskParameters->TaskInformation);
       //displayButtonStats(taskParameters->Identifier);   
      
       initialButtonRebounce = false;
    }

    // Read cyclicly every 1ms:
    const auto buttonState = digitalRead(taskParameters->GpioNo); //TODO this should come from task context
    
    //Button remains pressed
    if(buttonState == LOW)
    {
      ++buttonPress1msTicks;
    }
    // Button is depressed
    else
    {
      auto buttonPressType = (buttonPress1msTicks >= LONGPRESS_THRESHOLD_MS) ? ButtonPressType::LongPress : ButtonPressType::NormalPress;
      const char* buttonPressSumaryStr = (buttonPressType == ButtonPressType::LongPress) ? 
                                          "Button processing ended. Was a LONG-BUTTON-PRESS." : 
                                          "Button processing ended. Was a NORMAL-BUTTON-PRESS.";
      // TODO: display short summary only
      //Serial.println(buttonPressSumaryStr);
      //Serial.println(buttonPress1msTicks);
      delay(20); // some debounce protection for the button depress. Otherwise the depressing can be recognized as a button press.  
      
      //TODO: interrupt attach again possible here
      buttonPress1msTicks = 0;  
      
      initialButtonRebounce = true;

      const ButtonTrigger buttonTrigger{taskParameters->Identifier, buttonPressType, millis()};

      // xTicksToWait set to 0 as this function call should not put the task in blocking state
      // in case the client is not keeping up with freeing up the queue
      auto sendResult = xQueueSendToBack(g_ButtonTriggersQueue,(void *const) &buttonTrigger, 0);

      if( sendResult == errQUEUE_FULL )
      {
        Serial.println("The button triggers Queue is full");
        Serial.println("Why isnt't the client reading the items and making space?");
      }

      // Button press indtification is concluded the task can now clear the event and handle new ones.
      xEventGroupClearBits(g_TaskIntEventGroupHandle, taskParameters->EventId);
    }

    // Once the button event is set -> the ButtonTask to be blocked every 1 ms (essentialy every tick).
    delay(1);   
  }
}

void configureRtos()
{
    /* Event Grups used as a way of notihng the button task that a GPIO interrupt was captured */
    g_TaskIntEventGroupHandle = xEventGroupCreate();
    /* In case of the unlikely event that unsufficient heap memory is available (null returned) call abort() via assert. */
    assert(g_TaskIntEventGroupHandle != nullptr);
    
    /* Queue used by the button tasks to store identified button press actions, to be read back by main processing thread. */
    g_ButtonTriggersQueue    = xQueueCreate( ::FreeRtosConfig::MaxButtonTrigAmount, sizeof(ButtonTrigger));
    /* In case of the unlikely event that unsufficient heap memory is available (null returned) call abort() via assert. */
    assert(g_ButtonTriggersQueue != nullptr);

    
    /* The Task scheduler was already started. 
     * Two button handling task used with the same priority and the same core => therefore round robin should be possible.
     * TODO: add a Task for gatering statistics.
    */
    xTaskCreatePinnedToCore(buttonProcessingTask, "buttonProcessingTask1", ::FreeRtosConfig::ButtonTaskStackDepth, (void* const) &g_Button1Params, ::FreeRtosConfig::ButtonTaskPrio, NULL, FreeRtosConfig::AppCoreID);
    xTaskCreatePinnedToCore(buttonProcessingTask, "buttonProcessingTask2", ::FreeRtosConfig::ButtonTaskStackDepth, (void* const) &g_Button2Params, ::FreeRtosConfig::ButtonTaskPrio, NULL, FreeRtosConfig::AppCoreID);
}
// ########################################

void setup() 
{
  configureGPIOs();
  
  oledDispalyReset();

  serialConfigure();

  oledConfigure();

  configureLora();

  configureRtos();
  
  configureButtonInterrupts();

  displayInitSuccessMsg();
}

void loop() 
{
  //delay(DISPLAY_RERESH);
   
  //Serial.println("\n[LOOP]");

  ButtonTrigger buttonTrigger;
  // xTicksToWait -> set to 0 as we don't want to block the loop task.
  auto receiveStatus = xQueueReceive(g_ButtonTriggersQueue,&buttonTrigger, portMAX_DELAY);
  
  Serial.println("\n[LOOP]");
  
  if(receiveStatus == pdPASS)
  {
     displayButtonTriggerInfos(buttonTrigger);
  }

  /*
  measureMagneticField();
  
  measureSoilMositure();

  printOnSerial();

  printOnTinyDisplay();
  
  blinkLED();
  
  counter++;

  #if LORA_ENABLED
  sendLoraPacket();
  delay(PACKET_SEND_DELAY_MS);
  #endif
  
  delay(500);
  */
}
