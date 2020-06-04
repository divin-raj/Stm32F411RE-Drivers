/*
 * stm32f411re.h
 *
 *  Created on: 25-Apr-2020
 *      Author: dvn
 */

#ifndef STM32F411RE_H_
#define STM32F411RE_H_
/***********************Header files included******************************/
#include <stdint.h>

/**************************************************************************/

/*! @User defined types */
typedef uint8_t StdReturn_Type;
#define __vo    volatile


#define    NVIC_ISER0     ((__vo uint32_t *)0xE000E100)
#define    NVIC_ISER1     ((__vo uint32_t *)0xE000E104)
#define    NVIC_ISER2     ((__vo uint32_t *)0xE000E108)
#define    NVIC_ISER3     ((__vo uint32_t *)0xE000E10c)

#define    NVIC_ICER0     ((__vo uint32_t *)0XE000E180)
#define    NVIC_ICER1     ((__vo uint32_t *)0XE000E184)
#define    NVIC_ICER2     ((__vo uint32_t *)0XE000E188)
#define    NVIC_ICER3     ((__vo uint32_t *)0XE000E183)

#define    NVIC_IPR0      ((uint32_t *)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED   4u

#define NVIC_0_31              31u
#define NVIV_32_63             63u
#define NVIC_64_80             80u

/*! @User defined return types */
#define E_OK            0u
#define E_NOT_OK        1u
#define MUL_2           2u
#define MUL_4           4u
#define MUL_8		8u

#define DIV_2           2u
#define DIV_4           4u
#define DIV_8           8u

#define REM_2		2u
#define REM_4		4u
#define REM_8		8u
#define REM16           16u
#define REM32           32u
#define REM64           64u

#define NIBBLE_MSK          0xFu
#define BYTE_MSK            0xFFu
#define HLFWORD_MSK         0xFFFFu
#define WORD_MSK            0xFFFFFFFFu
/*! ************************************************************************
 * @PERIPHERAL REGISTERS
 **************************************************************************/
#define AHB1_BASEADDR   (uint32_t *)0x40020000
#define APB2_BASEADDR   (uint32_t *)0x40010000
#define APB1_BASEADDR   (uint32_t *)0x40000000

/*! @GPIO Peripheral Base address */
#define GPIOA_BASEADDR   ((uint32_t)AHB1_BASEADDR+0x0000)
#define GPIOB_BASEADDR   ((uint32_t)AHB1_BASEADDR+0x0400)
#define GPIOC_BASEADDR   ((uint32_t)AHB1_BASEADDR+0x0800)
#define GPIOD_BASEADDR   ((uint32_t)AHB1_BASEADDR+0x0C00)
#define GPIOE_BASEADDR   ((uint32_t)AHB1_BASEADDR+0x1000)
#define GPIOH_BASEADDR   ((uint32_t)AHB1_BASEADDR+0x1C00)
#define RCC_BASEADDR     ((uint32_t)AHB1_BASEADDR+0x3800)
#define EXTI_BASEADDR    ((uint32_t)APB2_BASEADDR+0x3C00)
#define SYSCFG_BASEADDR  ((uint32_t)APB2_BASEADDR+0x3800)

/*! @Gpio Pin Number Definition */
#define GPIO_PIN_NUM_0          0u
#define GPIO_PIN_NUM_1          1u
#define GPIO_PIN_NUM_2          2u
#define GPIO_PIN_NUM_3          3u
#define GPIO_PIN_NUM_4          4u
#define GPIO_PIN_NUM_5          5u
#define GPIO_PIN_NUM_6          6u
#define GPIO_PIN_NUM_7          7u
#define GPIO_PIN_NUM_8          8u
#define GPIO_PIN_NUM_9          9u
#define GPIO_PIN_NUM_10         10u
#define GPIO_PIN_NUM_11         11u
#define GPIO_PIN_NUM_12         12u
#define GPIO_PIN_NUM_13         13u
#define GPIO_PIN_NUM_14         14u
#define GPIO_PIN_NUM_15         15u

#define SET    1u
#define RST    0u

#define ENABLE  SET
#define DISABLE RST

/*! @GPIO Register maps */
typedef struct
    {
	/*! ********************************************************
	 *  @Description    :GPIO port mode register*
	 *  @Address offset :0x00
	 *  @Reset Value    :0xA800 0000 for port A
	 *                   0x0000 0280 for port B
	 *                   0x0000 0000 for other ports
	 ***********************************************************/
	__vo uint32_t MODER;

	/*! ********************************************************
	 *  @Description    :GPIO port output type register
	 *  @Address offset :0x04
	 *  @Reset Value    :0xA800 0000 for port
	 ***********************************************************/
	__vo uint32_t OTYPER;

	/*! ********************************************************
	 *  @Description    :GPIO port output speed register
	 *  @Address offset :0x08
	 *  @Reset Value    :0x0C000000 for port A
	 *                   0x000000C0 for port B
	 *                   0x00000000 for other ports
	 ***********************************************************/
	__vo uint32_t OSPEEDR;

	/*! ********************************************************
	 *  @Description    :GPIO port pull-up/pull-down register
	 *  @Address offset :0x0C
	 *  @Reset Value    :0x64000000 for port A
	 *                   0x00000100 for port B
	 *                   0x00000000 for other ports
	 ***********************************************************/
	__vo uint32_t PUPDR;

	/*! ********************************************************
	 *  @Description    :GPIO port input data register
	 *  @Address offset :0x10
	 *  @Reset Value    :0x0000 xxxx (x Undefined)
	 ***********************************************************/
	__vo uint32_t IDR;

	/*! ********************************************************
	 *  @Description    :GPIO port output data register
	 *  @Address offset :0x14
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t ODR;
	/*! ********************************************************
	 *  @Description    :GPIO port bit set/reset register
	 *  @Address offset :0x18
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t BSRR;

	/*! ********************************************************
	 *  @Description    :GPIO port configuration lock register
	 *  @Address offset :0x1c
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t LCKR;

	/*! ********************************************************
	 *  @Description    :GPIO alternate function register
	 *  @Address offset :0x20
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t AFR[2];
    } GPIO_RegDef_t;

/*! @RCC Register maps */
typedef struct
    {
	/*! ********************************************************
	 *  @Description    :RCC clock control register (RCC_CR)
	 *  @Address offset :0x00
	 *  @Reset Value    :0x0000 XX81 where X is undefined
	 ***********************************************************/
	__vo uint32_t CR;

	/*! ********************************************************
	 *  @Description    :RCC PLL configuration register (RCC_PLLCFGR)
	 *  @Address offset :0x04
	 *  @Reset Value    :0x24003010
	 ***********************************************************/
	__vo uint32_t PLLCFGR;

	/*! ********************************************************
	 *  @Description    :RCC clock configuration register (RCC_CFGR)
	 *  @Address offset :0x08
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t CFGR;

	/*! ********************************************************
	 *  @Description    :RCC clock interrupt register (RCC_CIR)
	 *  @Address offset :0x0C
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t CIR;

	/*! ********************************************************
	 *  @Description    :RCC AHB1 peripheral reset register (RCC_AHB1RSTR)
	 *  @Address offset :0x10
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t AHB1RSTR;

	/*! ********************************************************
	 *  @Description    :RCC AHB2 peripheral reset register (RCC_AHB2RSTR)
	 *  @Address offset :0x14
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t AHB2RSTR;

	/*! ********************************************************
	 *  @Description    : none
	 *  @Address offset : none
	 *  @Reset Value    : none
	 ***********************************************************/
	__vo uint32_t RESERVED[2];

	/*! ********************************************************
	 *  @Description    :RCC APB1 peripheral reset register for (RCC_APB1RSTR)
	 *  @Address offset :0x20
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t APB1RSTR;

	/*! ********************************************************
	 *  @Description    :RCC APB2 peripheral reset register (RCC_APB2RSTR)
	 *  @Address offset :0x24
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t APB2RSTR;

	/*! ********************************************************
	 *  @Description    :none
	 *  @Address offset :none
	 *  @Reset Value    :none
	 ***********************************************************/
	__vo uint32_t RESERVED2[2];

	/*! ********************************************************
	 *  @Description    :RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)
	 *  @Address offset :0x30
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t AHB1ENR;

	/*! ********************************************************
	 *  @Description    :RCC AHB2 peripheral clock enable register (RCC_AHB2ENR)
	 *  @Address offset :0x34
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t AHB2ENR;

	/*! ********************************************************
	 *  @Description    :none
	 *  @Address offset :none
	 *  @Reset Value    :none
	 ***********************************************************/
	__vo uint32_t RESERVED4[2];

	/*! ********************************************************
	 *  @Description    :RCC APB1 peripheral clock enable register (RCC_APB1ENR)
	 *  @Address offset :0x40
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t APB1ENR;

	/*! ********************************************************
	 *  @Description    :RCC APB2 peripheral clock enable register (RCC_APB2ENR)
	 *  @Address offset :0x44
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t APB2ENR;

	/*! ********************************************************
	 *  @Description    :none
	 *  @Address offset :none
	 *  @Reset Value    :none
	 ***********************************************************/
	__vo uint32_t RESERVED6[2];

	/*! ********************************************************
	 *  @Description    :RCC AHB1 peripheral clock enable in low power mode register(RCC_AHB1LPENR)
	 *  @Address offset :0x50
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t AHB1LPENR;

	/*! ********************************************************
	 *  @Description    :RCC AHB2 peripheral clock enable in low power mode register
	 *                  (RCC_AHB2LPENR)
	 *  @Address offset :0x54
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t AHB2LPENR;

	/*! ********************************************************
	 *  @Description    :none
	 *  @Address offset :none
	 *  @Reset Value    :none
	 ***********************************************************/
	__vo uint32_t RESERVED8[2];

	/*! ********************************************************
	 *  @Description    :RCC APB1 peripheral clock enable in low power mode register
	 *                   (RCC_APB1LPENR)
	 *  @Address offset :0x60
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t APB1LPENR;

	/*! ********************************************************
	 *  @Description    :RCC APB2 peripheral clock enabled in low power mode register
	 *                   (RCC_APB2LPENR)
	 *  @Address offset :0x64
	 *  @Reset Value    :0x00077930
	 ***********************************************************/
	__vo uint32_t APB2LPENR;

	/*! ********************************************************
	 *  @Description    :none
	 *  @Address offset :none
	 *  @Reset Value    :none
	 ***********************************************************/
	__vo uint32_t RESERVED10[2];

	/*! ********************************************************
	 *  @Description    :RCC Backup domain control register (RCC_BDCR)
	 *  @Address offset :0x70
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t BDCR;

	/*! ********************************************************
	 *  @Description    :RCC clock control & status register (RCC_CSR)
	 *  @Address offset :0x74
	 *  @Reset Value    :0x0E000000
	 ***********************************************************/
	__vo uint32_t CSR;

	/*! ********************************************************
	 *  @Description    :none
	 *  @Address offset :none
	 *  @Reset Value    :none
	 ***********************************************************/
	__vo uint32_t RESERVED12[2];

	/*! ********************************************************
	 *  @Description    :RCC spread spectrum clock generation register (RCC_SSCGR)
	 *  @Address offset :0x80
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t SSCGR;

	/*! ********************************************************
	 *  @Description    :RCC PLLI2S configuration register (RCC_PLLI2SCFGR)
	 *  @Address offset :0x84
	 *  @Reset Value    :0x24003000
	 ***********************************************************/
	__vo uint32_t PLLI2SCFGR;

	/*! ********************************************************
	 *  @Description    :RCC Dedicated Clocks Configuration Register (RCC_DCKCFGR)
	 *  @Address offset :0x8C
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t DCKCFGR;

    } RCC_RegDef_t;

/*! @EXTI Register Map */
typedef struct
    {
	/*! ********************************************************
	 *  @Description    :Interrupt mask register (EXTI_IMR)
	 *  @Address offset :0x00
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t EXTI_IMR;

	/*! ********************************************************
	 *  @Description    :Event mask register (EXTI_EMR)
	 *  @Address offset :0x04
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t EXTI_EMR;

	/*! ********************************************************
	 *  @Description    :Rising trigger selection register (EXTI_RTSR)
	 *  @Address offset :0x08
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t EXTI_RTSR;

	/*! ********************************************************
	 *  @Description    :Falling trigger selection register (EXTI_FTSR)
	 *  @Address offset :0x0C
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t EXTI_FTSR;

	/*! ********************************************************
	 *  @Description    :Software interrupt event register (EXTI_SWIER)
	 *  @Address offset :0x10
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t EXTI_SWIER;

	/*! ********************************************************
	 *  @Description    :Pending register (EXTI_PR)
	 *  @Address offset :0x14
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t EXTI_PR;

    } EXTI_RegDef_t;

/* @SYSCFG Register Maps */
typedef struct
    {
	/*! ********************************************************
	 *  @Description    :SYSCFG memory remap register (SYSCFG_MEMRMP)
	 *  @Address offset :0x00
	 *  @Reset Value    :0x0000 000X (X is the memory mode selected by the BOOT pins)
	 ***********************************************************/
	__vo uint32_t SYSCFG_MEMRMP;

	/*! ********************************************************
	 *  @Description    :SYSCFG peripheral mode configuration register (SYSCFG_PMC)
	 *  @Address offset :0x04
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t SYSCFG_PMC;

	/*! ********************************************************
	 *  @Description    :SYSCFG external interrupt configuration register 1
	 *                  (SYSCFG_EXTICR1)
	 *  @Address offset :0x08
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t SYSCFG_EXTICR[4];

	/*! ********************************************************
	 *  @Description    :SYSCFG external interrupt configuration register 2
	 *                   (SYSCFG_EXTICR2)
	 *  @Address offset :0x0C
	 *  @Reset Value    :0x00000000
	 ***********************************************************/
	__vo uint32_t SYSCFG_CMPCR;

    } SYSCFG_RegDef_t;

/*! @GPIOx Port definition  */
#define  GPIOA  ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define  GPIOB  ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define  GPIOC  ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define  GPIOD  ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define  GPIOE  ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define  GPIOH  ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define  RCC    ((RCC_RegDef_t *)RCC_BASEADDR)
#define  EXTI   ((EXTI_RegDef_t *) EXTI_BASEADDR)
#define  SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

/*! @GPIO PERIPHERAL CLOCK ENABLE */
#define GPIOA_CLKEN() (RCC->AHB1ENR |= (SET << GPIO_PIN_NUM_0))
#define GPIOB_CLKEN() (RCC->AHB1ENR |= (SET << GPIO_PIN_NUM_1))
#define GPIOC_CLKEN() (RCC->AHB1ENR |= (SET << GPIO_PIN_NUM_2))
#define GPIOD_CLKEN() (RCC->AHB1ENR |= (SET << GPIO_PIN_NUM_3))
#define GPIOE_CLKEN() (RCC->AHB1ENR |= (SET << GPIO_PIN_NUM_4))
#define GPIOH_CLKEN() (RCC->AHB1ENR |= (SET << GPIO_PIN_NUM_7))

/*! @SYSCFG CLOCK ENABLE */
#define SYSCFG_CLKEN() (RCC->APB2ENR |= (SET << GPIO_PIN_NUM_14))

/*! @GPIO PERIPHERAL CLOCK DISABLE */
#define GPIOA_CLKDI() do {(RCC->AHB1RSTR |= (SET << GPIO_PIN_NUM_0)); (RCC->AHB1RSTR &= ~(SET << GPIO_PIN_NUM_0));}while(0)
#define GPIOB_CLKDI() do {(RCC->AHB1RSTR |= (SET << GPIO_PIN_NUM_1)); (RCC->AHB1RSTR &= ~(SET << GPIO_PIN_NUM_1));}while(0)
#define GPIOC_CLKDI() do {(RCC->AHB1RSTR |= (SET << GPIO_PIN_NUM_2)); (RCC->AHB1RSTR &= ~(SET << GPIO_PIN_NUM_2));}while(0)
#define GPIOD_CLKDI() do {(RCC->AHB1RSTR |= (SET << GPIO_PIN_NUM_3)); (RCC->AHB1RSTR &= ~(SET << GPIO_PIN_NUM_3));}while(0)
#define GPIOE_CLKDI() do {(RCC->AHB1RSTR |= (SET << GPIO_PIN_NUM_4)); (RCC->AHB1RSTR &= ~(SET << GPIO_PIN_NUM_4));}while(0)
#define GPIOH_CLKDI() do {(RCC->AHB1RSTR |= (SET << GPIO_PIN_NUM_7)); (RCC->AHB1RSTR &= ~(SET << GPIO_PIN_NUM_7));}while(0)

/*! @SYSCFG CLOCK ENABLE */
#define SYSCFG_CLKDI() do {(RCC->APB2RSTR | = (SET << GPIO_PIN_NUM_14)); (RCC->APB2RSTR & = ~(SET << GPIO_PIN_NUM_14));}while(0)


#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

#define NVIC_IRQ_PRI0           0
#define NVIC_IRQ_PRI15          15
#endif /* STM32F411_H_ */

/*************************************************END of the File ********************************************************/
