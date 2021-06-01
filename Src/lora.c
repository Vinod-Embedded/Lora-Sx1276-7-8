/* This Driver is suitable for SX1276/7/8 Lora module
 * Author: Vinod Kumar from Vinod Embedded
 * Goto: vinodembedded.wordpress.com for detailed explanation of the 
 * lora driver
 */
 
#include "lora.h"
#include <string.h>

uint8_t packetIndex;

uint8_t lora_read_reg(lora_t * module, uint8_t addr) {
	uint8_t txByte = addr & 0x7f;
	uint8_t rxByte = 0x00;
	HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(module->pin->spi, &txByte, 1, 1000);
	while (HAL_SPI_GetState(module->pin->spi) != HAL_SPI_STATE_READY);
	
	HAL_SPI_Receive(module->pin->spi,&rxByte, 1, 1000);
	while(HAL_SPI_GetState(module->pin->spi) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_SET);
	return rxByte;
}

void lora_write_reg(lora_t * module, uint8_t addr, uint8_t cmd){
	uint8_t add = addr | 0x80;
  HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_RESET);
  HAL_SPI_Transmit(module->pin->spi, &add, 1, 1000);
	while (HAL_SPI_GetState(module->pin->spi) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(module->pin->spi, &cmd, 1, 1000);
	while (HAL_SPI_GetState(module->pin->spi) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(module->pin->nss.port, module->pin->nss.pin,GPIO_PIN_SET);
}

uint8_t lora_init(lora_t * module){
	uint8_t ret;
	HAL_GPIO_WritePin(module->pin->reset.port, module->pin->reset.pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(module->pin->reset.port, module->pin->reset.pin, GPIO_PIN_SET);
  HAL_Delay(10);
	
	ret = lora_read_reg(module, REG_VERSION);
	if(ret != 0x12){
		return 1;
	}
	lora_write_reg(module, REG_OP_MODE, (MODE_LONG_RANGE_MODE | MODE_SLEEP));
	//lora_write_reg(module, REG_FRF_MSB, 0x6C);
	//lora_write_reg(module, REG_FRF_MID, 0x40);
	//lora_write_reg(module, REG_FRF_LSB, 0x00);
	lora_set_frequency(module, FREQUENCY[module->frequency]);
	lora_write_reg(module, REG_FIFO_TX_BASE_ADDR, 0);
	lora_write_reg(module, REG_FIFO_RX_BASE_ADDR, 0);
	ret = lora_read_reg(module, REG_LNA);
	lora_write_reg(module, REG_LNA, ret | 0x03);
	lora_write_reg(module, REG_MODEM_CONFIG_3, 0x04);
	lora_write_reg(module, REG_PA_CONFIG, 0x8f);
	lora_write_reg(module, REG_OP_MODE, (MODE_LONG_RANGE_MODE | MODE_STDBY));
	return 0;
}

int lora_prasePacket(lora_t * module){
	int packetLength = 0, irqFlags; //,ret;
	irqFlags = lora_read_reg(module, REG_IRQ_FLAGS);
	//ret = lora_read_reg(module, REG_MODEM_CONFIG_1);
	lora_write_reg(module, REG_MODEM_CONFIG_1, 0x72);
	
	lora_write_reg(module, REG_IRQ_FLAGS, irqFlags);

	if((irqFlags & IRQ_RX_DONE_MASK) && ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)) {
		packetLength = lora_read_reg(module, REG_RX_NB_BYTES);
		lora_write_reg(module, REG_FIFO_ADDR_PTR, lora_read_reg(module, REG_FIFO_RX_CURRENT_ADDR));
		lora_write_reg(module, REG_OP_MODE, 0x81);
		packetIndex = 0;
	}
	else if((lora_read_reg(module, REG_OP_MODE)) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)){
		lora_write_reg(module, REG_FIFO_ADDR_PTR, 0);
		lora_write_reg(module, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
	}
	if((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK)== IRQ_PAYLOAD_CRC_ERROR_MASK){
		return -1;
	}
	return packetLength;
}

uint8_t lora_available(lora_t * module){
	return (lora_read_reg(module, REG_RX_NB_BYTES) - packetIndex);
}

uint8_t lora_read(lora_t * module){
	if(!lora_available(module))
		return 0;
	packetIndex++;
	return lora_read_reg(module, REG_FIFO);
}

uint8_t lora_begin_packet(lora_t * module){
	//int ret;
	if ((lora_read_reg(module, REG_OP_MODE) & MODE_TX) == MODE_TX) {
    return 1;
  }
	lora_write_reg(module, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
	lora_write_reg(module, REG_MODEM_CONFIG_1, 0x72);
	lora_write_reg(module, REG_FIFO_ADDR_PTR, 0);
  lora_write_reg(module, REG_PAYLOAD_LENGTH, 0);
	return 0;
}

void lora_tx(lora_t * module, uint8_t * buf, uint8_t size){
	int currentLength = lora_read_reg(module, REG_PAYLOAD_LENGTH);
  if ((currentLength + size > MAX_PKT_LENGTH)){
    size = MAX_PKT_LENGTH - currentLength;
  }

  for (int i = 0; i < size; i++) {
    lora_write_reg(module, REG_FIFO, buf[i]);
  }
  lora_write_reg(module, REG_PAYLOAD_LENGTH, currentLength + size);
}

uint8_t lora_end_packet(lora_t * module){
	uint8_t timeout = 100;
	lora_write_reg(module, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
  while((lora_read_reg(module,REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
		if(--timeout==0){
			HAL_Delay(1);
			return 1;
		}
  }
  lora_write_reg(module, REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	return 0;
}

void lora_set_frequency(lora_t * module, uint64_t freq){
	uint64_t frf = ((uint64_t)freq << 19) / 32000000;
  lora_write_reg(module, REG_FRF_MSB, (uint8_t)(frf >> 16));
  lora_write_reg(module,REG_FRF_MID, (uint8_t)(frf >> 8));
  lora_write_reg(module,REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/*   End of File
 * Author  : Vinod Kumar
 * Website : vinodembedded.wordpress.com
 */
