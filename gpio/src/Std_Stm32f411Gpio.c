/**************************************************************************************
 *                                   C O P Y R I G H T                                *
 **************************************************************************************
 *										      *
 *										      *
 *										      *
 **************************************************************************************/
/*! ***********************************************************************************
 * @file Stm32f411_Gpio.c                                                             *
 *                                                                                    *
 * @ingroup GPIO Driver                                                               *
 *                                                                                    *
 * @brief This is Gpio header file. It contains function Defnition                    *
 *                                                                                    *
 * @Author Divin Raj  								      *
 *                                                                                    *
 * @E-Mail divinraj1994@gmail.com                                                     *
 **************************************************************************************/
#include "stm32f411re.h"
#include "Std_Stm32f411Gpio.h"

/**************************************************************************************
 * @fn  	      - GpioClkInitialization                                         *
 *                                                                                    *
 * @brief             - This function Initialize GPIO clock                           *
 *                                                                                    *
 * @param[in]         - GPIOx -  Base address of Gpio Handler                                  *
 *                                                                                    *
 * @param[in]         - EnorDi - Enable or Disable                                                             *
 *                                                                                    *
 * @param[in]         -                                                               *
 *                                                                                    *
 * @return            -  none                                                         *
 *                                                                                    *
 * @Note              -  none                                                         *
 **************************************************************************************/
void GpioClkInitialization(GPIO_RegDef_t *GPIOx, uint8_t EnorDi)
    {
    /* GPIO PERIPHERAL CLOCK ENABLE */
    if (ENABLE == EnorDi)
	{
	if (GPIOA == GPIOx)
	    {
	    GPIOA_CLKEN();
	    }
	else if (GPIOB == GPIOx)
	    {
	    GPIOB_CLKEN();
	    }
	else if (GPIOC == GPIOx)
	    {
	    GPIOC_CLKEN();
	    }
	else if (GPIOD == GPIOx)
	    {
	    GPIOD_CLKEN();
	    }
	else if (GPIOE == GPIOx)
	    {
	    GPIOE_CLKEN();
	    }
	else if (GPIOH == GPIOx)
	    {
	    GPIOH_CLKEN();
	    }
	}

    /* GPIO PERIPHERAL CLOCK DISABLE */
    else
	{
	if (GPIOA == GPIOx)
	    {
	    GPIOA_CLKDI()
	    ;
	    }
	else if (GPIOB == GPIOx)
	    {
	    GPIOB_CLKDI()
	    ;
	    }
	else if (GPIOC == GPIOx)
	    {
	    GPIOC_CLKDI()
	    ;
	    }
	else if (GPIOD == GPIOx)
	    {
	    GPIOD_CLKDI()
	    ;
	    }
	else if (GPIOE == GPIOx)
	    {
	    GPIOE_CLKDI()
	    ;
	    }
	else if (GPIOH == GPIOx)
	    {
	    GPIOH_CLKDI()
	    ;
	    }
	}

    }

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
 * @return            -  none                                                         *
 *                                                                                    *
 * @Note              -  none                                                         *
 **************************************************************************************/
void GpioInit(GpioHandler_t *GpioHandler)
    {
    /* Auto variable */
    uint16_t temp16 = 0x0;
    uint32_t temp32 = 0x0;
    uint8_t pos = 0x0;
    uint8_t pas = 0x0;
    uint8_t portCode = 0x0;

    /* Enter if Normal, IN PUT or OUT PUT Mode */
    if (GpioHandler->GpioConfig.GpioMode < ALTFUN_MODE)
	{

	/* Setting Input and OutPut Mode */
	temp32 |= (GpioHandler->GpioConfig.GpioMode
		<< MUL_2 * GpioHandler->GpioConfig.GpioPin);
	GpioHandler->Gpiox->MODER &= ~(GpioHandler->GpioConfig.GpioMode
		<< MUL_2 * GpioHandler->GpioConfig.GpioPin);
	GpioHandler->Gpiox->MODER |= temp32;

	}
    else
	{

	/* Enabling SYSCFG Peripheral Clock */
	SYSCFG_CLKEN();

	/* Raising edge interrupt mode */
	if (GpioHandler->GpioConfig.GpioMode == RT_MODE)
	    {

	    EXTI->EXTI_RTSR |= (SET << GpioHandler->GpioConfig.GpioPin);
	    EXTI->EXTI_FTSR &= ~(SET << GpioHandler->GpioConfig.GpioPin);

	    }

	/* Falling edge interrpt mode */
	else if (GpioHandler->GpioConfig.GpioMode == FT_MODE)
	    {

	    EXTI->EXTI_FTSR |= (SET << GpioHandler->GpioConfig.GpioPin);
	    EXTI->EXTI_RTSR &= ~(SET << GpioHandler->GpioConfig.GpioPin);

	    }

	/* Settings Falling as well as Raising edge Interrupt mode */
	else if (GpioHandler->GpioConfig.GpioMode == RTFT_MODE)
	    {

	    EXTI->EXTI_RTSR |= (SET << GpioHandler->GpioConfig.GpioPin);
	    EXTI->EXTI_FTSR |= (SET << GpioHandler->GpioConfig.GpioPin);

	    }

	pos = (GpioHandler->GpioConfig.GpioPin / DIV_4);
	pas = (GpioHandler->GpioConfig.GpioPin % DIV_4);
	portCode = GPIOPORTCODE_CONVETOR(GpioHandler->Gpiox);

	SYSCFG->SYSCFG_EXTICR[pos] &= ~(NIBBLE_MSK << (MUL_4 * pas));
	SYSCFG->SYSCFG_EXTICR[pos] |= (portCode << (MUL_4 * pas));

	EXTI->EXTI_IMR |= (SET << GpioHandler->GpioConfig.GpioPin);

	}

    /* Setting output type  */
    temp16 |= (GpioHandler->GpioConfig.GpioOttype
	    << GpioHandler->GpioConfig.GpioPin);
    GpioHandler->Gpiox->OTYPER &= ~(GpioHandler->GpioConfig.GpioOttype
	    << GpioHandler->GpioConfig.GpioPin);
    GpioHandler->Gpiox->OTYPER = temp16;

    /* Setting output speed */
    temp32 = 0x00;
    temp32 |= (GpioHandler->GpioConfig.GpioOspeed
	    << MUL_2 * GpioHandler->GpioConfig.GpioOspeed);
    GpioHandler->Gpiox->OSPEEDR = ~(GpioHandler->GpioConfig.GpioOspeed
	    << GpioHandler->GpioConfig.GpioOspeed);
    GpioHandler->Gpiox->OSPEEDR = temp32;

    /* Setting pull up or pull down */
    temp32 = 0x00;
    temp32 |= (GpioHandler->GpioConfig.GpioPupdr
	    << GpioHandler->GpioConfig.GpioPin);
    GpioHandler->Gpiox->PUPDR &= ~(GpioHandler->GpioConfig.GpioPupdr
	    << GpioHandler->GpioConfig.GpioPin);
    GpioHandler->Gpiox->PUPDR |= temp32;

    /* Settings Analog function mode */
    if (GpioHandler->GpioConfig.GpioMode == ANALOG_MODE)
	{

	/* Selecting Alternate function Lower or High */
	pos = ((GpioHandler->GpioConfig.GpioPin) / DIV_8);

	/* Finding Shift Number */
	pas = ((GpioHandler->GpioConfig.GpioPin) % REM_8);

	GpioHandler->Gpiox->AFR[pos] &= ~(NIBBLE_MSK << MUL_4 * pas);
	GpioHandler->Gpiox->AFR[pos] |= (GpioHandler->GpioConfig.GpioMode
		<< MUL_4 * pas);

	}

    }

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
void Gpio_SetBit(GPIO_RegDef_t *GPIOx, uint8_t pinNumber, uint8_t Value)
    {

    /* Writting to Gpio Pin */
    if (Value == SET)
	{
	GPIOx->ODR |= Value << pinNumber;
	}
    else
	{
	GPIOx->ODR &= ~(Value << pinNumber);
	}
    }

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
void Gpio_WriteToPort(GPIO_RegDef_t *GPIOx, uint8_t Value)
    {

    /* Writting to Gpio Pin */

	GPIOx->ODR |= Value;

    }

/**************************************************************************************
 * @fn  	      - Gpio_ToggleBit                                                *
 *                                                                                    *
 * @brief             - This function toggle the GPIO pin                             *
 *                                                                                    *
 * @param[in]         - Base address of Gpio Handler                                  *
 *                                                                                    *
 * @param[in]         - PinNumber - Specific Pin where to read                        *
 *                                                                                    *
 * @param[in]         -                                                               *
 *                                                                                    *
 * @return            -  none                                                         *
 *                                                                                    *
 * @Note              -  none                                                         *
 **************************************************************************************/
void Gpio_ToggleBit(GPIO_RegDef_t *GPIOx, uint8_t pinNumber)
    {

    /* Toggling Gpio Pin */

    GPIOx->ODR ^= ENABLE << pinNumber;

    }

/**************************************************************************************
 * @fn  	      - Gpio_ReadBit                                                  *
 *                                                                                    *
 * @brief             - This function reading the GPIO pin                            *
 *                                                                                    *
 * @param[in]         - Base address of Gpio Handler                                  *
 *                                                                                    *
 * @param[in]         - PinNumber - Specific Pin where to read                        *
 *                                                                                    *
 * @param[in]         -                                                               *
 *                                                                                    *
 * @return            -  Value - Pin status                                           *
 *                                                                                    *
 * @Note              -  none                                                         *
 **************************************************************************************/
uint8_t Gpio_ReadBit(GPIO_RegDef_t *GPIOx, uint8_t PinNumber)
    {

    /* Reading pins */
    uint8_t Value = 0x0;
    Value = (GPIOx->IDR >> PinNumber) & 00000001;
    return Value;

    }

/**************************************************************************************
 * @fn  	      - Gpio_ReadPort                                                 *
 *                                                                                    *
 * @brief             - This function reading the GPIO pin                            *
 *                                                                                    *
 * @param[in]         - Base address of Gpio Handler                                  *
 *                                                                                    *
 * @param[in]         -                                                               *
 *                                                                                    *
 * @param[in]         -                                                               *
 *                                                                                    *
 * @return            -  Value - Gpio port status                                     *
 *                                                                                    *
 * @Note              -  none                                                         *
 **************************************************************************************/
uint8_t Gpio_ReadPort(GPIO_RegDef_t *GPIOx)
    {

    /* Reading ports */
    uint8_t Value = 0x0;
    Value = GPIOx->IDR;
    return Value;

    }

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
void Gpio_IRQInterrupteConfigure(uint8_t InterruptNumber, uint8_t EnorDi)
    {
    if (ENABLE == EnorDi)
	{
	/* Interupte Enabling */
	if (InterruptNumber <= NVIC_0_31)
	    {
	    *NVIC_ISER0 = (SET << InterruptNumber);
	    }
	else if (NVIC_0_31 < InterruptNumber && InterruptNumber < NVIV_32_63)
	    {
	    *NVIC_ISER1 |= (SET << (InterruptNumber % REM32));
	    }
	else if (NVIV_32_63 < InterruptNumber && InterruptNumber <= NVIC_64_80)
	    {
	    *NVIC_ISER2 |= (SET << InterruptNumber % REM64);
	    }
	}
    else
	{

	/* Interupte Disabling */
	if (InterruptNumber <= NVIC_0_31)
	    {
	    *NVIC_ICER0 |= (SET << InterruptNumber);
	    }
	else if (NVIC_0_31 < InterruptNumber && InterruptNumber < NVIV_32_63)
	    {
	    *NVIC_ICER1 |= (SET << (InterruptNumber % REM32));
	    }
	else if (NVIV_32_63 < InterruptNumber && InterruptNumber <= NVIC_64_80)
	    {
	    *NVIC_ICER2 |= (SET << InterruptNumber % REM64);
	    }
	}
    }

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
void Gpio_IRQPriorityConfigure(uint8_t InterruptNumber, uint8_t Priority)
    {

    /* Configuring Interrupt Priority */
    uint8_t pas = InterruptNumber % 4;
    uint8_t pos = InterruptNumber / 4;
    uint8_t shphase = (pas * 8) + (8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_IPR0 + pos) |= (Priority << shphase);

    }

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
void GPIO_IRQHandler(uint8_t PinNumber)
    {

    /*Interrupt Handler GPIO */
    if (EXTI->EXTI_PR & (SET << PinNumber))
	{
	EXTI->EXTI_PR |= SET << PinNumber;
	}

    }


/***********************************************************************************
 *                                 END OF FILE                                     *
 ***********************************************************************************/
