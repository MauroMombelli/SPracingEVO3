#include "spi.h"

void SPI_Config(void) {

	// Enable SPI1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Enable the SPI peripheral */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_TypeDef * SPI1_GPIO = GPIOB;

	uint16_t SPI1_SCK_PIN = 1 << 3;
	uint16_t SPI1_MISO_PIN = 1 << 4;
	uint16_t SPI1_MOSI_PIN = 1 << 5;
	uint16_t SPI1_NSS_PIN = 1 << 9;

	uint16_t SPI1_SCK_PIN_SOURCE = GPIO_PinSource3;
	uint16_t SPI1_MISO_PIN_SOURCE = GPIO_PinSource4;
	uint16_t SPI1_MOSI_PIN_SOURCE = GPIO_PinSource5;
	uint16_t SPI1_NSS_PIN_SOURCE = GPIO_PinSource9;

	/* Set pins */
	GPIO_PinAFConfig(SPI1_GPIO, SPI1_SCK_PIN_SOURCE, GPIO_AF_5);
	GPIO_PinAFConfig(SPI1_GPIO, SPI1_MISO_PIN_SOURCE, GPIO_AF_5);
	GPIO_PinAFConfig(SPI1_GPIO, SPI1_MOSI_PIN_SOURCE, GPIO_AF_5);
	GPIO_PinAFConfig(SPI1_GPIO, SPI1_NSS_PIN_SOURCE, GPIO_AF_5);

	// Init pins
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

	// General-purpose pin config
	GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN | SPI1_MISO_PIN | SPI1_MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI1_NSS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(SPI1_GPIO, &GPIO_InitStructure);

	// DEInit I2S hardware
	SPI_I2S_DeInit(SPI1);

	SPI_InitTypeDef spi;
	spi.SPI_Mode = SPI_Mode_Master;
	spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi.SPI_DataSize = SPI_DataSize_8b;
	spi.SPI_NSS = SPI_NSS_Soft;
	spi.SPI_FirstBit = SPI_FirstBit_MSB;
	spi.SPI_CRCPolynomial = 7;
	spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;

	spi.SPI_CPOL = SPI_CPOL_High;
	spi.SPI_CPHA = SPI_CPHA_2Edge;

	// Configure for 8-bit reads.
	SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);

	SPI_Init(SPI1, &spi);
	SPI_Cmd(SPI1, ENABLE);

	// Drive NSS high to disable connected SPI device.
	GPIO_SetBits(SPI1_GPIO, SPI1_NSS_PIN);
}

uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t data){
    uint16_t spiTimeout = 1000;

    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET) {
        if ((spiTimeout--) == 0)
            break;
    }

    SPI_SendData8(instance, data);

    spiTimeout = 1000;
    while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET){
        if ((spiTimeout--) == 0)
            break;
    }

    return ((uint8_t)SPI_ReceiveData8(instance));
}

/**
 * Return true if the bus is currently in the middle of a transmission.
 */
bool spiIsBusBusy(SPI_TypeDef *instance){
    return SPI_GetTransmissionFIFOStatus(instance) != SPI_TransmissionFIFOStatus_Empty || SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_BSY) == SET;
}

void spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len){
    uint16_t spiTimeout = 1000;

    uint8_t b;
    instance->DR;
    while (len--) {
        b = in ? *(in++) : 0xFF;
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_TXE) == RESET) {
            if ((spiTimeout--) == 0)
                break;
        }
        SPI_SendData8(instance, b);
        spiTimeout = 1000;
        while (SPI_I2S_GetFlagStatus(instance, SPI_I2S_FLAG_RXNE) == RESET) {
            if ((spiTimeout--) == 0)
                break;
        }
        b = SPI_ReceiveData8(instance);

        if (out)
            *(out++) = b;
    }
}


void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor){
#define BR_CLEAR_MASK 0xFFC7

    uint16_t tempRegister;

    SPI_Cmd(instance, DISABLE);

    tempRegister = instance->CR1;

    switch (divisor) {
        case 2:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_2;
            break;

        case 4:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_4;
            break;

        case 8:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_8;
            break;

        case 16:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_16;
            break;

        case 32:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_32;
            break;

        case 64:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_64;
            break;

        case 128:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_128;
            break;

        case 256:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_256;
            break;
    }

    instance->CR1 = tempRegister;

    SPI_Cmd(instance, ENABLE);
}
