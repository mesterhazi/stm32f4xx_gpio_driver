/*
 * D_gpio.h
 *
 *  Created on: Sep 14, 2019
 *      Author: adam
 */

#ifndef D_GPIO_H_
#define D_GPIO_H_

//#include "ret_values.h"
#include "stm32f407xx.h"

// General ENABLED / DISABLED macro def
#define D_ENABLED					((uint8_t)0x1)
#define D_DISABLED					((uint8_t)0x0)

// GPIO states
#define D_GPIO_STATE_SET					((uint8_t) 0x1)
#define D_GPIO_STATE_RESET					((uint8_t) 0x0)

// GPIO Pin switch:
#define D_GPIO_Pin0					((uint16_t)0x0001U) 	// GPIO PIN 0 selected
#define D_GPIO_Pin1					((uint16_t)0x0002U) 	// GPIO PIN 1 selected
#define D_GPIO_Pin2					((uint16_t)0x0004U) 	// GPIO PIN 2 selected
#define D_GPIO_Pin3					((uint16_t)0x0008U) 	// GPIO PIN 3 selected
#define D_GPIO_Pin4					((uint16_t)0x0010U) 	// GPIO PIN 4 selected
#define D_GPIO_Pin5					((uint16_t)0x0020U) 	// GPIO PIN 5 selected
#define D_GPIO_Pin6					((uint16_t)0x0040U) 	// GPIO PIN 6 selected
#define D_GPIO_Pin7					((uint16_t)0x0080U) 	// GPIO PIN 7 selected
#define D_GPIO_Pin8					((uint16_t)0x0100U) 	// GPIO PIN 8 selected
#define D_GPIO_Pin9					((uint16_t)0x0200U) 	// GPIO PIN 9 selected
#define D_GPIO_Pin10				((uint16_t)0x0400U) 	// GPIO PIN 10 selected
#define D_GPIO_Pin11				((uint16_t)0x0800U) 	// GPIO PIN 11 selected
#define D_GPIO_Pin12				((uint16_t)0x1000U) 	// GPIO PIN 12 selected
#define D_GPIO_Pin13				((uint16_t)0x2000U) 	// GPIO PIN 13 selected
#define D_GPIO_Pin14				((uint16_t)0x4000U) 	// GPIO PIN 14 selected
#define D_GPIO_Pin15				((uint16_t)0x8000U) 	// GPIO PIN 15 selected
#define D_GPIO_PinALL				((uint16_t)0xFFFFU) 	// ALL GPIO PINs selected
#define D_GPIO_Pin_MASK				0x0000FFFFU 		// mask for checking Pin validity



// Possible GPIO modes
#define D_GPIO_MODE_IN 				0x00000000U
#define D_GPIO_MODE_OUT				0x00000001U
#define D_GPIO_MODE_AF				0x00000002U
#define D_GPIO_MODE_ANALOG			0x00000003U
#define D_GPIO_MODE_IT_FE			0x00010000U
#define D_GPIO_MODE_IT_RE			0x00010100U
#define D_GPIO_MODE_IT_FRE			0x00011000U
#define D_GPIO_MODE_IT_MASK			0x00010000U


// Possible GPIO Output modes
#define D_GPIO_OTYPE_PushPull		0x00000000U
#define D_GPIO_OTYPE_OpenDrain		0x00000001U

// Possible Output Speed settings
#define D_GPIO_OSPEED_LOW			0x00000000U
#define D_GPIO_OSPEED_MEDIUM		0x00000001U
#define D_GPIO_OSPEED_HIGH			0x00000002U
#define D_GPIO_OSPEED_VERYHIGH		0x00000003U

// Possible GPIO Output settings
#define D_GPIO_PUPD_NONE			0x00000000U
#define D_GPIO_PUPD_UP				0x00000001U
#define D_GPIO_PUPD_DOWN			0x00000002U

// Possible Alternate Function configs
#define D_GPIO_AF_0					0x00000000U
#define D_GPIO_AF_1					0x00000001U
#define D_GPIO_AF_2					0x00000002U
#define D_GPIO_AF_3					0x00000003U
#define D_GPIO_AF_4					0x00000004U
#define D_GPIO_AF_5					0x00000005U
#define D_GPIO_AF_6					0x00000006U
#define D_GPIO_AF_7					0x00000007U
#define D_GPIO_AF_8					0x00000008U
#define D_GPIO_AF_9					0x00000009U
#define D_GPIO_AF_10				0x00000010U
#define D_GPIO_AF_11				0x00000011U
#define D_GPIO_AF_12				0x00000012U
#define D_GPIO_AF_13				0x00000013U
#define D_GPIO_AF_14				0x00000014U
#define D_GPIO_AF_15				0x00000015U

// Utility functions for argument checking
#define D_IS_GPIO_MODE(MODE) 	(((MODE) == D_GPIO_MODE_IN) ||\
								((MODE) == D_GPIO_MODE_OUT) ||\
								((MODE) == D_GPIO_MODE_AF)  ||\
								((MODE) == D_GPIO_MODE_ANALOG) ||\
								((MODE) == D_GPIO_MODE_IT_RE) ||\
								((MODE) == D_GPIO_MODE_IT_FE) ||\
								((MODE) == D_GPIO_MODE_IT_FRE))

#define D_GPIO_GET_INDEX(__GPIOx__)    (uint8_t)(((__GPIOx__) == (GPIOA))? 0U :\
                                               ((__GPIOx__) == (GPIOB))? 1U :\
                                               ((__GPIOx__) == (GPIOC))? 2U :\
                                               ((__GPIOx__) == (GPIOD))? 3U :\
                                               ((__GPIOx__) == (GPIOE))? 4U :\
                                               ((__GPIOx__) == (GPIOF))? 5U :\
                                               ((__GPIOx__) == (GPIOG))? 6U :\
                                               ((__GPIOx__) == (GPIOH))? 7U : 8U)


#define D_IS_GPIO_IT_MODE(MODE) ((MODE) & D_GPIO_MODE_IT_MASK)

#define D_IS_GPIO_OTYPE(TYPE)	(((TYPE) == D_GPIO_OTYPE_PushPull) ||\
								((TYPE)  == D_GPIO_OTYPE_OpenDrain))

#define D_IS_GPIO_AF(MODE)		((MODE) > 0x00000000U && (MODE) < 0x00000016U)
#define D_IS_GPIO_PIN(PIN)		(((PIN) & D_GPIO_Pin_MASK) && ((PIN) && (~D_GPIO_Pin_MASK)))
#define D_IS_GPIO_SPEED(SPEED)	(((SPEED) == D_GPIO_OSPEED_LOW) ||\
								((SPEED) == D_GPIO_OSPEED_MEDIUM) ||\
								((SPEED) == D_GPIO_OSPEED_HIGH) ||\
								((SPEED) == D_GPIO_OSPEED_VERYHIGH))
#define D_IS_GPIO_PuPd(VAL)		(((VAL) == D_GPIO_PUPD_DOWN) ||\
								((VAL) == D_GPIO_PUPD_UP) ||\
								((VAL) == D_GPIO_PUPD_NONE))

// NVIC related macros
#define PIN_TO_NVIC_OFFSET				6U
#define NVIC_9_5_POS					23U
#define NVIC_15_10_POS					40U
#define NVIC_15_10_OFFSET				8U
#define D_GPIO_GET_NVIC_FROM_PIN(pin)	((pin) < 5 ? (pin) + PIN_TO_NVIC_OFFSET : ((pin) < 10 ? NVIC_9_5_POS : NVIC_15_10_OFFSET))

typedef struct{
	uint32_t Pin; 				// PIN number mask on lower 2 bytes
	uint32_t Mode; 				// MODE
	uint32_t OutputType;		// OTYPE
	uint32_t OutSpeed;			// OSPEED
	uint32_t PuPd;				// PUPDR
	uint32_t AlternateFunc;		// AFR
}D_GPIO_InitTypeDef;

typedef enum{
	D_GPIO_PIN_RESET = 0,
	D_GPIO_PIN_SET
}D_GPIO_PinState;


void D_Init_Gpio(GPIO_TypeDef* GpioPort, D_GPIO_InitTypeDef* GpioInit);
void D_GPIO_SetPin(GPIO_TypeDef* GpioPort, uint16_t Pin);
void D_GPIO_ResetPin(GPIO_TypeDef* GpioPort, uint16_t Pin);
void D_GPIO_TogglePin(GPIO_TypeDef* GpioPort, uint16_t Pin);
void D_GPIO_ClockEn(GPIO_TypeDef* GpioPort, uint8_t Enabled);
D_GPIO_PinState D_GPIO_ReadPin(GPIO_TypeDef* GpioPort, uint16_t Pin);
uint16_t D_GPIO_ReadPort(GPIO_TypeDef* GpioPort);

// IRQ
void D_GPIO_IRQInit(uint16_t Pin, uint8_t Priority);
void D_GPIO_IRQDeinit(uint16_t Pin);
void GPIO_IRQHandler(uint16_t Pin);


#endif /* D_GPIO_H_ */
