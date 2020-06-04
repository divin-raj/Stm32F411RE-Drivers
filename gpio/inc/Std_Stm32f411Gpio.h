/**************************************************************************************
 *                                   C O P Y R I G H T                                *
 **************************************************************************************
 *										      *
 *										      *
 *										      *
 **************************************************************************************/
/*!************************************************************************************
 * @file Stm32f411_Gpio.h                                                             *
 *                                                                                    *
 * @ingroup GPIO Driver                                                               *
 *                                                                                    *
 * @brief This is Gpio header file. It contains function prototype,                   *
 *  struct handlers and Macro configuration parameters for the Gpio module.           *
 *                                                                                    *
 * @Author Divin Raj                                                                  *
 **************************************************************************************/
#ifndef INC_STD_STM32F411GPIO_H_
#define INC_STD_STM32F411GPIO_H_

/**************************************************************************************
 * System Includes                                                                    *
 **************************************************************************************/
#include "stm32f411re.h"

/**************************************************************************************/

#define GPIOPORTCODE_CONVETOR(x)    ((x == GPIOA) ? 0 :\
				     (x == GPIOB) ? 1 :\
				     (x == GPIOC) ? 2 :\
				     (x == GPIOD) ? 3 :\
				     (x == GPIOE) ? 4 :\
				     (x == GPIOH) ? 7 : 0)

/*! @GPIO Configuration parameter */
typedef struct
    {
	/*! @Gpio Pin Number */
	uint8_t GpioPin;

	/*! @Gpio Mode */
	uint8_t GpioMode;

	/*! @Gpio OutputType */
	uint8_t GpioOttype;

	/*! @Gpio Output Speed */
	uint8_t GpioOspeed;

	/*! @Gpio pullUp PullDdown Selection */
	uint8_t GpioPupdr;

    } GpioConfig_t;

/*! @GPIO Handler structure */
typedef struct
    {
	/*! @GPIO register definition */
	GPIO_RegDef_t *Gpiox;

	/*! @GPIO Configuration Parameters */
	GpioConfig_t GpioConfig;

    } GpioHandler_t;

/*! @GPIO Mode Macro */
#define INPUT_MODE   0u
#define OUTPUT_MODE  1u
#define ALTFUN_MODE  2u
#define ANALOG_MODE  3u
#define RT_MODE      4u
#define FT_MODE      5u
#define RTFT_MODE    6u

/*! @GPIO Speed Macro */
#define LO_SPEED     0u
#define MI_SPEED     1u
#define HI_SPEED     2u
#define FA_SPEED     3u

/*! @GPIO Type Macro */
#define PUSH_PULL    0u
#define OPEN_DRAIN   1u

/*! @GPIO Pull up Pull down Macro */
#define NO_UPDOWN    0u
#define PULLUP       1u
#define PULLDOWN     2u
#define RESERVED     3u

/**************************************************************************************
 * @fn  	      - GPIO_Init                                                     *
 *                                                                                    *
 * @brief             - This function Initialize the GPIO Functionality               *
 *                                                                                    *
 * @param[in]         - Base address of Gpio Handler                                  *
 *                                                                                    *
 * @param[in]         -                                                               *
 *                                                                                    *
 * @param[in]         -                                                               *
 *                                                                                    *
 * @return            - none                                                          *
 *                                                                                    *
 * @Note              - none                                                          *
 **************************************************************************************/
void GpioInit(GpioHandler_t *Handler);

/**************************************************************************************
 * @fn  	      - GpioClkInitialization                                         *
 *                                                                                    *
 * @brief             - This function Initialize GPIO clock                           *
 *                                                                                    *
 * @param[in]         - GPIOx  -  Base address of Gpio Handler                        *
 *                                                                                    *
 * @param[in]         - EnorDi -  Enable or Disable                                   *
 *                                                                                    *
 * @param[in]         -                                                               *
 *                                                                                    *
 * @return            - none                                                          *
 *                                                                                    *
 * @Note              - none                                                          *
 **************************************************************************************/
void GpioClkInitialization(GPIO_RegDef_t *GPIOx, uint8_t EnorDi);

/**************************************************************************************
 * @fn  	      - Gpio_ReadBit                                                  *
 *                                                                                    *
 * @brief             - This function reading the GPIO pin                            *
 *                                                                                    *
 * @param[in]         - GPIOx     - Base address of Gpio Handler                      *
 *                                                                                    *
 * @param[in]         - PinNumber - Pin Number                                        *
 *                                                                                    *
 * @param[in]         - Value     - Value to writing                                  *
 *                                                                                    *
 * @return            - none                                                          *
 *                                                                                    *
 * @Note              - none                                                          *
 **************************************************************************************/
uint8_t Gpio_ReadBit(GPIO_RegDef_t *GPIOx, uint8_t PinNumber);

/**************************************************************************************
 * @fn  	      - Gpio_SetBit                                                   *
 *                                                                                    *
 * @brief             - This function set the GPIO pin                                *
 *                                                                                    *
 * @param[in]         - Base address of Gpio Handler                                  *
 *                                                                                    *
 * @param[in]         - pinNumber - Pin number to set or reset                                                            *
 *                                                                                    *
 * @param[in]         - Value - Value to write                                                            *
 *                                                                                    *
 * @return            - none                                                          *
 *                                                                                    *
 * @Note              - none                                                          *
 **************************************************************************************/
void Gpio_SetBit(GPIO_RegDef_t *GPIOx,uint8_t pinNumber, uint8_t Value);

/**************************************************************************************
 * @fn  	      - Gpio_ReadPort                                                  *
 *                                                                                    *
 * @brief             - This function reading the GPIO pin                            *
 *                                                                                    *
 * @param[in]         - Base address of Gpio Handler                                  *
 *                                                                                    *
 * @param[in]         -                                                               *
 *                                                                                    *
 * @param[in]         -                                                               *
 *                                                                                    *
 * @return            -  none                                                         *
 *                                                                                    *
 * @Note              -  none                                                         *
 **************************************************************************************/
uint8_t Gpio_ReadPort(GPIO_RegDef_t *GPIOx);

/**************************************************************************************
 * @fn  	      - Gpio_ToggleBit                                                   *
 *                                                                                    *
 * @brief             - This function toggle the GPIO pin                                *
 *                                                                                    *
 * @param[in]         - Base address of Gpio Handler                                  *
 *                                                                                    *
 * @param[in]         -                                                               *
 *                                                                                    *
 * @param[in]         -                                                               *
 *                                                                                    *
 * @return            -  none                                                         *
 *                                                                                    *
 * @Note              -  none                                                         *
 **************************************************************************************/
void Gpio_ToggleBit(GPIO_RegDef_t *GPIOx,uint8_t pinNumber);

/**************************************************************************************
 * @fn  	      - Gpio_SetBit                                                   *
 *                                                                                    *
 * @brief             - This function set the GPIO pin                                *
 *                                                                                    *
 * @param[in]         - Base address of Gpio Handler                                  *
 *                                                                                    *
 * @param[in]         - PinNumber - Specific Pin where to write                       *
 *                                                                                    *
 * @param[in]         - Value - status to write                                       *
 *                                                                                    *
 * @return            - none                                                          *
 *                                                                                    *
 * @Note              - none                                                          *
 **************************************************************************************/
void Gpio_WriteToPort(GPIO_RegDef_t *GPIOx, uint8_t Value);

/**************************************************************************************
 * @fn  	      - Gpio_IRQInterrupteConfigure                                                 *
 *                                                                                    *
 * @brief             - This function used for interrupt configuration                           *
 *                                                                                    *
 * @param[in]         - InterruptNumber - Interupte Number                                   *
 *                                                                                    *
 * @param[in]         - EnorDi -  Enable or Disable                                                           *
 *                                                                                    *
 * @param[in]         -                                                               *
 *                                                                                    *
 * @return            -  Value - Gpio port status                                     *
 *                                                                                    *
 * @Note              -  none                                                         *
 **************************************************************************************/
void Gpio_IRQInterrupteConfigure(uint8_t InterruptNumber, uint8_t EnorDi);

/**************************************************************************************
 * @fn  	      - Gpio_IRQPriorityConfigure                                                 *
 *                                                                                    *
 * @brief             - This function reading the GPIO pin                            *
 *                                                                                    *
 * @param[in]         - InterruptNumber - Interrupt Number                                  *
 *                                                                                    *
 * @param[in]         - Priority - Priority value                                                               *
 *                                                                                    *
 * @param[in]         -                                                               *
 *                                                                                    *
 * @return            -  Value - Gpio port status                                     *
 *                                                                                    *
 * @Note              -  none                                                         *
 **************************************************************************************/
void Gpio_IRQPriorityConfigure(uint8_t InterruptNumber, uint8_t Priority);

/**************************************************************************************
 * @fn  	      - Gpio_IRQPriorityConfigure                                                 *
 *                                                                                    *
 * @brief             - This function reading the GPIO pin                            *
 *                                                                                    *
 * @param[in]         - InterruptNumber - Interrupt Number                                  *
 *                                                                                    *
 * @param[in]         - Priority - Priority value                                                               *
 *                                                                                    *
 * @param[in]         -                                                               *
 *                                                                                    *
 * @return            -  Value - Gpio port status                                     *
 *                                                                                    *
 * @Note              -  none                                                         *
 **************************************************************************************/
void GPIO_IRQHandler(uint8_t PinNumber);


#endif /* INC_STD_STM32F411GPIO_H_ */

/***********************************************************************************
 *                                 END OF FILE                                     *
 ***********************************************************************************/
