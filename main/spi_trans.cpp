#include "spi_trans.h"

void spi_write_cmd(uint8_t address, uint8_t tx_data) {
  byte c1 = address | WRITE;
  byte c2 = address; 
  digitalWrite(EXT_SPI_SS, LOW);
  SPI.transfer(c1);
  SPI.transfer(c2);
  SPI.transfer(tx_data);
  digitalWrite(EXT_SPI_SS, HIGH);
    
}

//------------------------------------------------------------------------------
// spi_read_cmd(): Read from a SPI device. Return the data read from register
//------------------------------------------------------------------------------
uint8_t spi_read_cmd(uint8_t address) {
  uint8_t data = 0;           // incoming byte from the SPI

  uint8_t c1 = address & READ;
  uint8_t c2 = address; 
  digitalWrite(EXT_SPI_SS, LOW);
  SPI.transfer(c1);
  SPI.transfer(c2);
  data = SPI.transfer(0x00);
  digitalWrite(EXT_SPI_SS, HIGH);

  return (data);

}

//*****************************************************************************
