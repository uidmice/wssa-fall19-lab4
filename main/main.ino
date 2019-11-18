#include <SPI.h>
#include "FXOS8700CQ.h"
#include <FreeRTOS_ARM.h>



// Chip Select Pin for SPI and Interrupt Pin
#define EXT_SPI_SS 4
#define INT_PIN 51

FXOS8700CQ sensor;
SemaphoreHandle_t sem;

void setup() {
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);

  // Initialize SerialUSB 
  SerialUSB.begin(9600);
  while(!SerialUSB);
  
  // Initialize SPI
  SPI.begin();

  // Initialize sensor
  sensor = FXOS8700CQ();
  sensor.init();

  attachInterrupt(digitalPinToInterrupt(INT_PIN), ISR_Handler, CHANGE);

  sem = xSemaphoreCreateBinary();  
  portBASE_TYPE s = xTaskCreate(colllectData, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//  portBASE_TYPE s2 = xTaskCreate(processData, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  
  // check for creation errors
  if ( sem==NULL ||  s != pdPASS ) {
     SerialUSB.println(F("Creation problem"));
     while(1);
  }
  // start scheduler
  vTaskStartScheduler();
  SerialUSB.println("Insufficient RAM");
  while(1);


}
void colllectData()​​{
  float vX, vY, vZ;
  while (1){
    xSemaphoreTake( sem, portMAX_DELAY );
    SerialUSB.println("Collect!");
    sensor.readMagData();
    vX = sensor.magData.x*0.1;
    vY = sensor.magData.y*0.1;
    vZ = sensor.magData.z*0.1;
    processData();
  }
  
}


void processData()​​{
  static int count = 0;
  count ++;
  SerialUSB.println(count);
}

void ISR_Handler(){
  SerialUSB.println("Interrupt!");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR( sem, &xHigherPriorityTaskWoken );
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  
}



void loop() {
  //
}
