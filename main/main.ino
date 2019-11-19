#include "FXOS8700CQ.h"
#include <FreeRTOS_ARM.h>

static void collectData(void* arg);
static void processData(void* arg);
static void ISR_Handler(void );


FXOS8700CQ sensor;
SemaphoreHandle_t sem1, sem2;

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
  sensor.setThreshold(120);

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

    sensor.readMagData();
    vX = (sensor.magData.x-sensor.refMagData.x)* 0.1;
    vY = (sensor.magData.y-sensor.refMagData.y)* 0.1;
    vZ = (sensor.magData.z-sensor.refMagData.z)* 0.1;
    mag = sqrt(vX*vX+vY*vY+vZ*vZ);
    SerialUSB.println(mag,1);
    SerialUSB.println(vY,1);
    SerialUSB.println(vZ,1);
    xSemaphoreGive(sem2);
  }
}


static void processData(void* arg){
  static int count = 0;
  while (1) {
    xSemaphoreTake (sem2, portMAX_DELAY);
    count ++;
    SerialUSB.println(count);
  }

}

static void ISR_Handler(void ) {
  byte intReg = sensor.readIntReg();
  if (intReg & 0x02){
    xSemaphoreGiveFromISR( sem1, NULL);
  }
}

void loop() {
  //
}
