/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: 11 Jul 2020
 *      Author: templ
 */

#include "stm32f4xx_gpio_driver.h"


/*****************Peripheral Clock Setup*********************/

/****************************************************************
*@fn                 -GPIO_PeriClockControl
*
*@brief              -This function enables or disables periheral clock for the given GPIO port
*
*@Param[in]          -base address of GPIO  Peripheral
*@Param[in]          -ENABLE or DISABLE macros
*@Param[in]          -
*
*@Return             -
*
*@Notes              -
*
*
*/
void GPIO_PeriClockControl (GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		  {
		     GPIOA_PCLK_EN();
		  }else if(pGPIOx == GPIOB)
			 {
				 GPIOB_PCLK_EN();
			 }else if(pGPIOx == GPIOC)
			   {
				     GPIOC_PCLK_EN();
			   }else if(pGPIOx == GPIOD)
			     {
				        GPIOD_PCLK_EN();
			     }else if(pGPIOx == GPIOE)
			         {
				          GPIOE_PCLK_EN();
			         }else if(pGPIOx == GPIOF)
			            {
				             GPIOF_PCLK_EN();
			            }else if(pGPIOx == GPIOG)
			               {
				                GPIOG_PCLK_EN();
			               }else if(pGPIOx == GPIOH)
			                   {
				                   GPIOH_PCLK_EN();
			                   }
      }else
       {
    		if(pGPIOx == GPIOA)
    			  {
    			     GPIOA_PCLK_DI();
    			  }else if(pGPIOx == GPIOB)
    				 {
    					 GPIOB_PCLK_DI();
    				 }else if(pGPIOx == GPIOC)
    				   {
    					     GPIOC_PCLK_DI();
    				   }else if(pGPIOx == GPIOD)
    				     {
    					        GPIOD_PCLK_DI();
    				     }else if(pGPIOx == GPIOE)
    				         {
    					          GPIOE_PCLK_DI();
    				         }else if(pGPIOx == GPIOF)
    				            {
    					             GPIOF_PCLK_DI();
    				            }else if(pGPIOx == GPIOG)
    				               {
    					                GPIOG_PCLK_DI();
    				               }else if(pGPIOx == GPIOH)
    				                   {
    					                   GPIOH_PCLK_DI();
    				                   }
       }


}

/****************Init and DeInit************/

/****************************************************************
*@fn                 -GPIO_Init
*
*@brief              -this functions configure the given given port
*
*@Param[in]          -base address of GPIO  Handle Structure
*
*@Return             -
*
*@Notes              -
*
*/

void GPIO_Init (GPIO_Handle_t *pGPIOHandle)
{
	  uint32_t temp = 0 ; /*temp register */
	  /* 1. configure the mod of GPIO pin */

	  if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG )
	  {
            // non interupt mode
            temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
            pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); //clearing
		    pGPIOHandle->pGPIOx->MODER |= temp;

	  }else
	  {
		  /* this part code later (interupt mode) */

	  }



       temp = 0;

	   //2.configure the speed
       temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
       pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); //clearing
       pGPIOHandle->pGPIOx->OSPEEDER |= temp;

       temp = 0;
	   //3. confgure the pupd settings
       temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
       pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); //clearing
       pGPIOHandle->pGPIOx->PUPDR |= temp;

       temp = 0;
	   //4.configre  output type
       temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
       pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); //clearing
       pGPIOHandle->pGPIOx->OTYPER |= temp;

       temp = 0;
	  // 5. configure alternate functonality (if required)
       if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ALTFN )
       {
    	   //configure alternate fucntion registers
    	   uint8_t temp1, temp2;

    	   temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
    	   temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

    	   pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2) );



       }


}

/****************************************************************
*@fn                 -GPIO_DeInit
*
*@brief              -This function deinitialize the given GPIO port
*
*@Param[in]          -base address of Peripheral definiation structer
*
*@Return             -
*
*@Notes              -
*
*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	 {
	    GPIOA_REG_RESET();
	  }else if(pGPIOx == GPIOB)
	   {
		  GPIOA_REG_RESET();
	   }else if(pGPIOx == GPIOC)
	    {
		   GPIOA_REG_RESET();
	    }else if(pGPIOx == GPIOD)
	     {
	    	GPIOA_REG_RESET();
	     }else if(pGPIOx == GPIOE)
	      {
	    	 GPIOA_REG_RESET();
	      }else if(pGPIOx == GPIOF)
	       {
	    	  GPIOA_REG_RESET();
	       }else if(pGPIOx == GPIOG)
	        {
	    	   GPIOA_REG_RESET();
	        }else if(pGPIOx == GPIOH)
	        {
	        	GPIOA_REG_RESET();
	        }



}

/****************Data Read and Write ************/

uint8_t  GPIO_ReadFromInputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001 ) ;

	return value;

}







uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx)
{

	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;

}






void GPIO_WriteToOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

	 if(Value == GPIO_PIN_SET)
	 {
		 //write 1 to the output data register at the bit field corresponding to the pin number
		 pGPIOx->ODR |= ( 1 << PinNumber);

	 }else
	  {
		 //write 0
		 pGPIOx->ODR &= ~( 1 << PinNumber);
	   }



}


void GPIO_WriteToOutputPort (GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

	pGPIOx->ODR = Value;

}

void GPIO_ToggleOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

		pGPIOx->ODR = pGPIOx->ODR ^ (1 <<PinNumber);
}

/***************IRQ Configuration and ISR Handling **************/
void GPIO_IRQConfig (uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnorDi);
void GPIO_IRQHandling (uint8_t PinNumber);



