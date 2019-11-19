#include "FXOS8700CQ.h"
#include <math.h>

//------------------------------------------------------------------------------
// FXOS8700CQ(): Initialize configuration parameters
//------------------------------------------------------------------------------
FXOS8700CQ::FXOS8700CQ() {
    magODR = MODR_100HZ; // Magnetometer data/sampling rate
    magOSR = MOSR_5;     // Choose magnetometer oversample rate
    magData = {0, 0, 0};    
    refMagData = {0, 0, 0}; 
    whoAmIData = FXOS8700CQ_WHOAMI_VAL;    
    _minX=0x7fff;
    _minY=0x7fff; 
    _minZ=0x7fff;
    _maxX=0x8000;
    _maxY=0x8000;
    _maxZ=0x8000;

    pinMode(EXT_SPI_SS, OUTPUT);        // Select the GPIO Pin 51 as SPI Chip Select
    digitalWrite(EXT_SPI_SS, HIGH);     // Set Pin to high (Active Low)
    pinMode(INT_PIN, INPUT);
}

//------------------------------------------------------------------------------
// writeReg(): Writes to a register
//------------------------------------------------------------------------------
void FXOS8700CQ::writeReg(uint8_t reg, uint8_t data) {
  spi_write_cmd(reg, data);
}

//------------------------------------------------------------------------------
// readReg(): Reads from a register
//------------------------------------------------------------------------------
uint8_t FXOS8700CQ::readReg(uint8_t reg) {
  return (spi_read_cmd(reg));
}

//------------------------------------------------------------------------------
// readMagData(): Read the magnometer X, Y and Z axisdata
//------------------------------------------------------------------------------
void FXOS8700CQ::readMagData() {
  uint8_t XU = readReg(FXOS8700CQ_M_OUT_X_MSB);
  uint8_t XL = readReg(FXOS8700CQ_M_OUT_X_LSB);
  uint8_t YU = readReg(FXOS8700CQ_M_OUT_Y_MSB);
  uint8_t YL = readReg(FXOS8700CQ_M_OUT_Y_LSB);
  uint8_t ZU = readReg(FXOS8700CQ_M_OUT_Z_MSB);
  uint8_t ZL = readReg(FXOS8700CQ_M_OUT_Z_LSB);
  magData.x = (int16_t) (XU<<8)+XL;
  magData.y = (int16_t) (YU<<8)+YL;
  magData.z = (int16_t) (ZU<<8)+ZL;

}
//------------------------------------------------------------------------------
// standby(): Put the FXOS8700CQ into standby mode for writing to registers
//------------------------------------------------------------------------------
void FXOS8700CQ::standby() {
  uint8_t old = readReg(FXOS8700CQ_CTRL_REG1);
  uint8_t data = old & 0b11111110; //clear the active bit
  writeReg(FXOS8700CQ_CTRL_REG1, data);

}

//------------------------------------------------------------------------------
// active(): Put the FXOS8700CQ into active mode to output data
//------------------------------------------------------------------------------
void FXOS8700CQ::active() {
  uint8_t old = readReg(FXOS8700CQ_CTRL_REG1);
  uint8_t data = old | 0b00000001; //set the active bit
  writeReg(FXOS8700CQ_CTRL_REG1, data);
}

//------------------------------------------------------------------------------
// init(): Initialize the magnetometer
//         This function will put the magnetometer in standby mode, modify the 
//         registers that put the device in mag-only mode, set the correct data
//         rate (ODR) and oversampling rate (OSR) for the magnetometer and put
//         it back in active mode
//------------------------------------------------------------------------------
void FXOS8700CQ::init() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  //write to CTRL_REG1 reg, set to 100Hz and enter active mode
  byte data = (magODR << 3 ) | 0x01;
  writeReg(FXOS8700CQ_CTRL_REG1, data);

  //write to M_CTRL_REG1 reg, set to Mag-only mode and OSR = 5
  data = (magOSR << 2 ) | 0x01 ;
  writeReg(FXOS8700CQ_M_CTRL_REG1, data);
  writeReg(FXOS8700CQ_M_VECM_CFG, 0x38);
  checkWhoAmI();

  calibrate();
  
}

//------------------------------------------------------------------------------
// checkWhoAmI(): Check the whoAmI register
//------------------------------------------------------------------------------
void FXOS8700CQ::checkWhoAmI(void) {
  byte data = readReg(FXOS8700CQ_WHO_AM_I);
  if (data!=whoAmIData)
    SerialUSB.println("ERROR: whoAmIData mismatch!!");
  else 
    debug_println("ID identified successfully." );
}

//*****************************************************************************

//------------------------------------------------------------------------------
// enMagInt(): Enable magnetic vector interrupt on INT1
//------------------------------------------------------------------------------
void FXOS8700CQ::enMagInt(void) {
  writeReg(FXOS8700CQ_M_VECM_CFG, 0x7B);
}

//*****************************************************************************

//------------------------------------------------------------------------------
// disMagInt(): Disable magnetic vector interrupt on INT1
//------------------------------------------------------------------------------
void FXOS8700CQ::disMagInt(void) {
  writeReg(FXOS8700CQ_M_VECM_CFG, 0x38);
}

//*****************************************************************************

//*****************************************************************************

//------------------------------------------------------------------------------
// enDrdyInt(): Enable DRDY interrupt on INT1
//------------------------------------------------------------------------------
void FXOS8700CQ::enDrdyInt(void) {
  writeReg(FXOS8700CQ_CTRL_REG4, 0x01);
  writeReg(FXOS8700CQ_CTRL_REG5, 0x01);
}

//*****************************************************************************

//------------------------------------------------------------------------------
// disDrdyInt(): Disable DRDY interrupt on INT1
//------------------------------------------------------------------------------
void FXOS8700CQ::disDrdyInt(void) {
  writeReg(FXOS8700CQ_CTRL_REG4, 0x00);
  writeReg(FXOS8700CQ_CTRL_REG5, 0x00);
}

//*****************************************************************************

//------------------------------------------------------------------------------
// calibrate(): set offset and threshold values
//------------------------------------------------------------------------------
void FXOS8700CQ::calibrate() {
  writeReg(FXOS8700CQ_M_CTRL_REG2, 0x01);
  byte i;
  for (i = 0; i < 30; i ++){
    readMagData() ;
    _minX = _minX < magData.x ? _minX : magData.x;
    _maxX = _maxX > magData.x ? _maxX : magData.x;
    _minY = _minY < magData.y ? _minY : magData.y;
    _maxY = _maxY > magData.y ? _maxY : magData.y;
    _minZ = _minZ < magData.z ? _minZ : magData.z;
    _maxZ = _maxZ > magData.z ? _maxZ : magData.z;

    delay(10);
  }

  refMagData.x = (_minX + _maxX) >> 1;
  refMagData.y = (_minY + _maxY) >> 1;
  refMagData.z = (_minZ + _maxZ) >> 1;

  SerialUSB.print("refX = ");
  SerialUSB.print(refMagData.x);
  SerialUSB.print(", refY = ");
  SerialUSB.print(refMagData.y);
  SerialUSB.print(", refZ = ");
  SerialUSB.println(refMagData.z);
  writeReg(FXOS8700CQ_M_VECM_INITX_MSB, (refMagData.x & 0xFF00)>>8 );
  writeReg(FXOS8700CQ_M_VECM_INITX_LSB, (refMagData.x & 0x00FF));
  writeReg(FXOS8700CQ_M_VECM_INITY_MSB, (refMagData.y & 0xFF00)>>8 );
  writeReg(FXOS8700CQ_M_VECM_INITY_LSB, (refMagData.y & 0x00FF));
  writeReg(FXOS8700CQ_M_VECM_INITZ_MSB, (refMagData.z & 0xFF00)>>8 );
  writeReg(FXOS8700CQ_M_VECM_INITZ_LSB, (refMagData.z & 0x00FF));
  
}

//*****************************************************************************

//------------------------------------------------------------------------------
// readIntReg(): Read interrupt register
//------------------------------------------------------------------------------
byte FXOS8700CQ::readIntReg(void) {
  byte data = readReg(FXOS8700CQ_M_INT_SRC);
}

//*****************************************************************************

//------------------------------------------------------------------------------
// setThreshold(): Set threshold for interrupt
//------------------------------------------------------------------------------
void FXOS8700CQ::setThreshold(int16_t thr) {
    writeReg(FXOS8700CQ_M_VECM_THS_MSB, (thr & 0xFF00)>>8 );
    writeReg(FXOS8700CQ_M_VECM_THS_LSB, (thr & 0x00FF) );


}

//*****************************************************************************
