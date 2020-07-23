/*
 * stm32f446xx.h
 *
 *  Created on: Jul 8, 2020
 *      Author:
 */

#include <stdint.h>
#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_
#define __vo   volatile
/**************************************************
    Base addresses of FLASH and SRAM memories
****************************************************/

#define FLASH_BASEADDR                 0x08000000U
#define SRAM1_BASEADDR                 0x20000000U //112kb
#define SRAM2_BASEADDR                 0x20001C00U //16kb
#define ROM                            0x1FFF0000U
#define SRAM                           SRAM1_BASEADDR


/*
 * AHBx and APBx Peripherals Base Addresses
 */

#define  PERIPH_BASEADDR             0x40000000U
#define  APB1PERIPH_BASEADDR         PERIPH_BASEADDR
#define  APB2PERIPH_BASEADDR         0x40010000U
#define  AHB1PERIPH_BASEADDR         0x40020000U
#define  AHB2PERIPH_BASEADDR         0x50000000U

/*
 * Base address of peripherals which hanging on AHB1 bus
 */

#define GPIOA_BASEADDR             (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR             (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR             (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR             (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR             (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR             (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR             (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR             (AHB1PERIPH_BASEADDR + 0x1C00)

#define RCC_BASEADDR               (AHB1PERIPH_BASEADDR + 0x3800)
/*
 * Base address of peripherals which hanging on APB1 bus
 */

#define I2C1_BASEADDR               (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR               (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR               (APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR               (APB1PERIPH_BASEADDR + 0X3800)
#define SPI3_BASEADDR               (APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR             (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR             (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR              (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR              (APB1PERIPH_BASEADDR + 0x5000)

/*
 * Base address of peripherals which hanging on APB2 bus
 */

#define EXTI_BASEADDR               (APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR               (APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR             (APB2PERIPH_BASEADDR + 0x3800)

#define USART1_BASEADDR             (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR             (APB2PERIPH_BASEADDR + 0x1400)


/**************************PERIPHERAL REGISTER DEFINITION STRUCTERS************************
 *
 *
 * Note: this structers exclusive for the MCU STM32F446RE
 * Number of register for SPI peripheral of STM32F4 family of MCUs might be different
 * compared to number of registers of SPI peripheral STM32Lx or STM32F0x family of MCUs
 * please check your device Reference Manual
 *
 *
 *
 * */

typedef struct
{
	__vo uint32_t MODER;                    /* GPIO port mode register,                            address offset 0x00   */
	__vo int32_t OTYPER;                    /* GPIO port output type register,                     address offset 0x04   */
	__vo int32_t OSPEEDER;                  /* GPIO port output speed register,                    address offset 0x08   */
	__vo uint32_t PUPDR;                    /* GPIO port pull-up/pull-down register,               address offset 0x0C   */
	__vo uint32_t IDR;                      /* GPIO port input data register,                      address offset 0x10   */
	__vo uint32_t ODR;                      /* GPIO port output data register,                     address offset 0x14   */
	__vo uint32_t BSRR;                     /* GPIO port bit set/reset register,                   address offset 0x18   */
	__vo uint32_t LCKR;                     /* GPIO port configuration lock register,              address offset 0x1C   */
	__vo uint32_t AFR[2];                   /* For AFR[0] GPIO alternate function low register,    address offset 0x20   */
	                                        /* For AFR[1] GPIO alternate function high register    address offset 0x24   */

}GPIO_RegDef_t;



/*
 * Peripheral definitions (peripheral base adresses type casted to xxx_RegDef_t)
 *
 */

 #define  GPIOA         ((GPIO_RegDef_t*)GPIOA_BASEADDR)
 #define  GPIOB         ((GPIO_RegDef_t*)GPIOB_BASEADDR)
 #define  GPIOC         ((GPIO_RegDef_t*)GPIOC_BASEADDR)
 #define  GPIOD         ((GPIO_RegDef_t*)GPIOD_BASEADDR)
 #define  GPIOE         ((GPIO_RegDef_t*)GPIOE_BASEADDR)
 #define  GPIOF         ((GPIO_RegDef_t*)GPIOF_BASEADDR)
 #define  GPIOG         ((GPIO_RegDef_t*)GPIOG_BASEADDR)
 #define  GPIOH         ((GPIO_RegDef_t*)GPIOH_BASEADDR)


#define   RCC           ((RCC_RegDef_t*)RCC_BASEADDR )

typedef struct
{

	__vo uint32_t CR;                /* RCC clock control register                     Address offset: 0x00   */
	__vo uint32_t PLLCFGR;           /* RCC PLL configuration register                 Address offset: 0x04   */
	__vo uint32_t CFGR;              /* RCC clock configuration register               Address offset: 0x08   */
	__vo uint32_t CIR;               /* RCC clock interrupt register                   Address offset: 0x0C   */
	__vo uint32_t AHB1RSTR;          /* RCC AHB1 peripheral reset register             Address offset: 0x10   */
	__vo uint32_t AHB2RSTR;          /* RCC AHB2 peripheral reset register             Address offset: 0x14   */
	__vo uint32_t AHB3RSTR;          /* RCC AHB3 peripheral reset register             Address offset: 0x18   */
	 uint32_t    RESERVED0;         /*     Reserved        0x1C                                              */
	__vo uint32_t APB1RSTR;          /*  RCC APB1 peripheral reset register             Address offset: 0x20  */
	__vo uint32_t APB2RSTR;          /* RCC APB2 peripheral reset register             Address offset: 0x24   */
     uint32_t     RESERVED1[2];      /*     Reserved        0x28 - 0x2C                                       */
	__vo uint32_t AHB1ENR;           /* RCC AHB1 peripheral clock enable register      Address offset: 0x30   */
	__vo uint32_t AHB2ENR;           /* RCC AHB2 peripheral clock enable register      Address offset: 0x34   */
	__vo uint32_t AHB3ENR;           /* RCC AHB3 peripheral clock enable register      Address offset: 0x38   */
	 uint32_t     RESERVED2;         /*     Reserved      0x3C                                                */
	__vo uint32_t APB1ENR;           /* RCC APB1 peripheral clock enable register      Address offset: 0x40   */
	__vo uint32_t APB2ENR;           /* RCC APB2 peripheral clock enable register      Address offset: 0x44   */
	__vo uint32_t RESERVED3[2];      /*     Reserved      0x48 - 0x4C                                                          */
	__vo uint32_t AHB1LPENR;         /* RCC AHB1 peripheral clock enable in low power mode register     Address offset: 0x50   */
	__vo uint32_t AHB2LPENR;         /* RCC AHB2 peripheral clock enable in low power mode register     Address offset: 0x54   */
	__vo uint32_t AHB3LPENR;         /* RCC AHB3 peripheral clock enable in low power mode register     Address offset: 0x58   */
	 uint32_t     RESERVED4;         /*     Reserved      0x5C                                                                 */
	__vo uint32_t APB1LPENR;         /* RCC_APB1 peripheral clock enable in low power mode register     Address offset: 0x60   */
	__vo uint32_t APB2LPENR;         /* RCC_APB2 peripheral clock enable in low power mode register     Address offset: 0x64   */
	__vo uint32_t RESERVED5[2];      /*     Reserved      0x68 - 0x6C                                                          */
	__vo uint32_t BDCR;              /* RCC Backup domain control register                              Address offset: 0x70   */
	__vo uint32_t CSR;               /* RCC clock control & status register                             Address offset: 0x74   */
	__vo uint32_t RESERVED6[2];      /*     Reserved        0x78 - 0x7C                                                        */
	__vo uint32_t SSCGR;             /* RCC spread spectrum clock generation register                   Address offset: 0x80   */
	__vo uint32_t PLLI2SCFGR;        /* RCC PLLI2S configuration register                               Address offset: 0x84   */
	__vo uint32_t PLLSAICFGR;        /* RCC PLL configuration register                                  Address offset: 0x88   */
	__vo uint32_t DCKCFGR;           /* RCC Dedicated Clock Configuration Register                      Address offset: 0x8C   */
	__vo uint32_t CKGATENR;          /* RCC clocks gated enable register                                Address offset: 0x90   */
	__vo uint32_t DCKCFGR2;          /* RCC dedicated clocks configuration register 2                   Address offset: 0x94   */


}RCC_RegDef_t;



/******* Clock Enable Macros For GPIOx Peripherals *******/
#define GPIOA_PCLK_EN()         ( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()         ( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()         ( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()         ( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()         ( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN()         ( RCC->AHB1ENR |= ( 1 << 5 ) )
#define GPIOG_PCLK_EN()         ( RCC->AHB1ENR |= ( 1 << 6 ) )
#define GPIOH_PCLK_EN()         ( RCC->AHB1ENR |= ( 1 << 7 ) )


/******* Clock Enable Macros For I2Cx Peripherals *******/

#define I2C1_PCLK_EN()          (RCC->APB1ENR |= ( 1 <<21 ) )
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= ( 1 <<22 ) )
#define I2C3_PCLK_EN()          (RCC->APB1ENR |= ( 1 <<23 ) )


/******* Clock Enable Macros For SPIx Peripherals *******/

#define SPI1_PCLK_EN()          (RCC->APB2ENR |= ( 1 <<12 ) )
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= ( 1 <<14 ) )
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= ( 1 <<15 ) )


/******* Clock Enable Macros For USARTx Peripherals *******/

#define USART1_PCLK_EN()          (RCC->APB2ENR |= ( 1 <<4 ) )
#define USART2_PCLK_EN()          (RCC->APB1ENR |= ( 1 <<17 ) )
#define USART3_PCLK_EN()          (RCC->APB1ENR |= ( 1 <<18 ) )
#define UART4_PCLK_EN()           (RCC->APB1ENR |= ( 1 <<19 ) )
#define UART5_PCLK_EN()           (RCC->APB1ENR |= ( 1 <<20 ) )
#define USART6_PCLK_EN()          (RCC->APB2ENR |= ( 1 <<5 ) )


/******* Clock Enable Macros For SYSCFG Peripherals *******/

#define SYSCFG_PCLK_EN ()          (RCC->APB2ENR |= ( 1 <<14 ) )


/******* Clock Disable Macros For GPIOx Peripherals *******/

#define GPIOA_PCLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 7 ) )


/******* Clock Disable Macros For I2Cx Peripherals *******/

#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~( 1 <<21 ) )
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~( 1 <<22 ) )
#define I2C3_PCLK_DI()          (RCC->APB1ENR &= ~( 1 <<23 ) )


/******* Clock Disable Macros For SPIx Peripherals *******/

#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~( 1 <<12 ) )
#define SPI2_PCLK_DI()          (RCC->APB1ENR &= ~( 1 <<14 ) )
#define SPI3_PCLK_DI()          (RCC->APB1ENR &= ~( 1 <<15 ) )


/******* Clock Disable Macros For USARTx Peripherals *******/

#define USART1_PCLK_DI()          (RCC->APB2ENR &= ~( 1 <<4 ) )
#define USART2_PCLK_DI()          (RCC->APB1ENR &= ~( 1 <<17 ) )
#define USART3_PCLK_DI()          (RCC->APB1ENR &= ~( 1 <<18 ) )
#define UART4_PCLK_DI()           (RCC->APB1ENR &= ~( 1 <<19 ) )
#define UART5_PCLK_DI()           (RCC->APB1ENR &= ~( 1 <<20 ) )
#define USART6_PCLK_DI()          (RCC->APB2ENR &= ~( 1 <<5 ) )


/******* Clock Disable Macros For SYSCFG Peripherals *******/

#define SYSCFG_PCLK_DI()          (RCC->APB2ENR &= ~( 1 <<14 ) )



/******* Macros for resetting GPIO peripherals *************/

#define GPIOA_REG_RESET()       do{(RCC->AHB1RSTR |= ( 1 << 0 ) ); (RCC->AHB1RSTR &= ~( 1 << 0 ) ); }while(0)
#define GPIOB_REG_RESET()       do{(RCC->AHB1RSTR |= ( 1 << 1 ) ); (RCC->AHB1RSTR &= ~( 1 << 1 ) ); }while(0)
#define GPIOC_REG_RESET()       do{(RCC->AHB1RSTR |= ( 1 << 2 ) ); (RCC->AHB1RSTR &= ~( 1 << 2 ) ); }while(0)
#define GPIOD_REG_RESET()       do{(RCC->AHB1RSTR |= ( 1 << 3 ) ); (RCC->AHB1RSTR &= ~( 1 << 3 ) ); }while(0)
#define GPIOE_REG_RESET()       do{(RCC->AHB1RSTR |= ( 1 << 4 ) ); (RCC->AHB1RSTR &= ~( 1 << 4 ) ); }while(0)
#define GPIOF_REG_RESET()       do{(RCC->AHB1RSTR |= ( 1 << 5 ) ); (RCC->AHB1RSTR &= ~( 1 << 5 ) ); }while(0)
#define GPIOG_REG_RESET()       do{(RCC->AHB1RSTR |= ( 1 << 6 ) ); (RCC->AHB1RSTR &= ~( 1 << 6 ) ); }while(0)
#define GPIOH_REG_RESET()       do{(RCC->AHB1RSTR |= ( 1 << 7 ) ); (RCC->AHB1RSTR &= ~( 1 << 7 ) ); }while(0)








/***************** some generic macros *******************/

#define ENABLE             1
#define DISABLE            0
#define SET                ENABLE
#define RESET              DISABLE
#define GPIO_PIN_SET       SET
#define GPIO_PIN_RESET     RESET



#include "stm32f4xx_gpio_driver.h"


#endif /* INC_STM32F446XX_H_ */
