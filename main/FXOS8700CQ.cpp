#include "FXOS8700CQ.h"
#include <math.h>

//------------------------------------------------------------------------------
// FXOS8700CQ(): Initialize configuration parameters
//------------------------------------------------------------------------------
FXOS8700CQ::FXOS8700CQ() {
    magODR = MODR_100HZ; // Magnetometer data/sampling rate
    magOSR = MOSR_5;     // Choose magnetometer oversample rate
    magData = {0, 0, 0};      
    whoAmIData = FXOS8700CQ_WHOAMI_VAL;     

    pinMode(CS_PIN, OUTPUT);        // Select the GPIO Pin 51 as SPI Chip Select
    digitalWrite(CS_PIN, HIGH);     // Set Pin to high (Active Low)
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
  debug_print("MagX read: ");
  debug_prints(magData.x, DEC);
  debug_prints("(0x");
  debug_prints(magData.x, HEX);
  debug_printlns(")");
  debug_prints("MagY read: ");
  debug_prints(magData.y, DEC);
  debug_prints("(0x");
  debug_prints(magData.y, HEX);
  debug_printlns(")");
  debug_prints("MagZ read: ");
  debug_prints(magData.z, DEC);
  debug_prints("(0x");
  debug_prints(magData.z, HEX);
  debug_printlns(")");

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

  checkWhoAmI();
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
