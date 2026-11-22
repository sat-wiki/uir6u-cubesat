#ifndef TEST_SPI_H
#define TEST_SPI_H
#include <stdint.h>
#include <stddef.h>
typedef int SPI_HandleTypeDef;
extern SPI_HandleTypeDef hspi4;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define GPIOE 0
#define GPIO_PIN_4 16
#define NAND_CS_Pin GPIO_PIN_4
#define NAND_CS_GPIO_Port GPIOE
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
void HAL_GPIO_WritePin(int port, int pin, int state);
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi, uint8_t *data, unsigned size, unsigned timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *spi, uint8_t *data, unsigned size, unsigned timeout);
#endif
