#include "stm32f30x.h"
#include "stdbool.h"
void SPI_Config(void);

uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t *data);
bool spiIsBusBusy(SPI_TypeDef *instance);
void spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len);
void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor);
