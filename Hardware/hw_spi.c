#include "hw_spi.h"

void spi_write_bus(unsigned char dat) {
  unsigned char recv_data = 0;

  // 发送数据
  DL_SPI_transmitData8(SPI_LCD_INST, dat);
  // 等待SPI总线空闲
  while (DL_SPI_isBusy(SPI_LCD_INST))
    ;

  // //接收数据
  // recv_data = DL_SPI_receiveData8(SPI_LCD_INST);
  // //等待SPI总线空闲
  // while(DL_SPI_isBusy(SPI_LCD_INST));
}