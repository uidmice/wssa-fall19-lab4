#include "FXOS8700CQ.h"
#include <FreeRTOS_ARM.h>

static void collectData(void* arg);
static void processData(void* arg);
static void ISR_Handler(void );


FXOS8700CQ sensor;
SemaphoreHandle_t sem1, sem2;
int16_t thres = 120; //set interrupt threshold to be 12uT

void setup() {
  // Initialize SPI
  SPI.begin();

  // Initialize SerialUSB
  SerialUSB.begin(9600);
  while (!SerialUSB);

  // Initialize sensor
  sensor = FXOS8700CQ();
  sensor.init();
  sensor.enMagInt();
  sensor.disDrdyInt();
  sensor.setThreshold(thres);

  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);

  sem1 = xSemaphoreCreateBinary();
  sem2 = xSemaphoreCreateBinary();
  portBASE_TYPE s1 = xTaskCreate(collectData, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);
  portBASE_TYPE s2 = xTaskCreate(processData, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  attachInterrupt(digitalPinToInterrupt(INT_PIN), ISR_Handler, CHANGE);
  sensor.readIntReg();


  // check for creation errors
  if ( sem1 == NULL || sem2 == NULL ||  s1 != pdPASS || s2 != pdPASS ) {
    SerialUSB.println(F("Creation problem"));
    while (1);
  }
  // start scheduler
  vTaskStartScheduler();
  SerialUSB.println("Insufficient RAM");
  while (1);


}

static void collectData(void* arg){
  float vX, vY, vZ, mag;
  while (1) {
    xSemaphoreTake(sem1, portMAX_DELAY );
    SerialUSB.println("Interrupt due to magnetometer vector magnitude!");

    for(;;){
      sensor.readMagData();
      vX = (sensor.magData.x-sensor.refMagData.x)* 0.1;
      vY = (sensor.magData.y-sensor.refMagData.y)* 0.1;
      vZ = (sensor.magData.z-sensor.refMagData.z)* 0.1;
      mag = sqrt(vX*vX+vY*vY+vZ*vZ);
      if (mag<thres * 0.1) 
        break;
      SerialUSB.print("\nMagnetometer:\n dX = ");
      SerialUSB.println(vX, 4);
      SerialUSB.print(" dY = ");
      SerialUSB.println(vY, 4);
      SerialUSB.print(" dZ = ");
      SerialUSB.println(vZ, 4);
      delay(10); 
      
    }
    xSemaphoreGive(sem2);
  }
}


static void processData(void* arg){
  static int count = 0;
  while (1) {
    xSemaphoreTake (sem2, portMAX_DELAY);
    count ++;
    SerialUSB.println(count);
    sensor.readIntReg();
  }

}

static void ISR_Handler(void ) {
    xSemaphoreGiveFromISR( sem1, NULL);
}

void loop() {
  //
}
