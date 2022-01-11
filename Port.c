/*
 * Port.c
 *
 *  Created on: Jan 5, 2022
 *      Author: Mohamed
 */
#include "Port.h"
#include "Port_Regs.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif

STATIC const Port_ConfigChannel * Port_PortChannels = NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Initialize the Port module.
************************************************************************************/
void Port_Init(const Port_ConfigType * ConfigPtr)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* check if the input configuration pointer is not a NULL_PTR */
	if (NULL_PTR == ConfigPtr)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_Init_SID,
		     PORT_E_PARAM_CONFIG);
	}
	else
#endif
	{
		/*
		 * Set the module state to initialized and point to the PB configuration structure using a global pointer.
		 * This global pointer is global to be used by other functions to read the PB configuration structures
		 */
		Port_Status       = PORT_INITIALIZED;
		Port_PortChannels = ConfigPtr->Channels; /* address of the first Channels structure --> Channels[0] */
	}

	 volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
	 volatile uint32 delay = 0;
	 volatile uint8 i= 0 ;
	 for (i = 0 ; i < PORT_CONFIGURED_CHANNLES ;i ++)
	 {
	    switch(Port_PortChannels[i].port_num)
	    {
	        case  PortConfig_PORTA_NUM:
	        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
			 break;
		    case  PortConfig_PORTB_NUM:
			PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
			 break;
	    	case  PortConfig_PORTC_NUM:
			PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
			 break;
	        case  PortConfig_PORTD_NUM:
			PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
			 break;
	        case  PortConfig_PORTE_NUM:
	        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
			 break;
	        case  PortConfig_PORTF_NUM:
	        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
			 break;
	    }
	    /* Enable clock for PORT and allow time for clock to start*/
	        SYSCTL_REGCGC2_REG |= (1<<Port_PortChannels[i].port_num);
	        delay = SYSCTL_REGCGC2_REG;

	        if( ((Port_PortChannels[i].port_num == PortConfig_PORTD_NUM) && (Port_PortChannels[i].pin_num == PortConfig_PIN7_NUM)) || ((Port_PortChannels[i].port_num == PortConfig_PORTF_NUM) && (Port_PortChannels[i].pin_num == PortConfig_PIN0_NUM)) ) /* PD7 or PF0 */
	           {
	               *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = Magic_Number;                     /* Unlock the GPIOCR register */
	               SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PortChannels[i].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
	           }
	           else if( (Port_PortChannels[i].port_num == PortConfig_PORTC_NUM) && (Port_PortChannels[i].pin_num <= PortConfig_PIN3_NUM) ) /* PC0 to PC3 */
	           {
	               /* Do Nothing ...  this is the JTAG pins */
	               return;
	           }
	           else
	           {
	               /* Do Nothing ... No need to unlock the commit register for this pin */
	           }

	        if ( Port_PortChannels[i].pin_mode == PortConfig_Mode_ADC) /*ADC Mode Select */
	        {
	        	SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[i].pin_num);/*SET Analog */
	        	CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[i].pin_num);/* Clear Digital */
	        }
	        else
	        {
	        	CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[i].pin_num);/*Clear Analog */
	            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[i].pin_num);   /* Set Digital */
	        }

	        if ( Port_PortChannels[i].pin_mode == PortConfig_Mode_GPIO )
	        {
	        	CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[i].pin_num); /* Clear Alternate Function To Select GPIO*/
	        }
	        else
	        {
	        	SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[i].pin_num); /* Set Alternate Function To Use Other Function Of Pin */
	        }

	        switch ( Port_PortChannels[i].pin_mode)
	        {
	        case PortConfig_Mode_GPIO:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[i].pin_num * 4));
	        	break;
	        case PortConfig_Mode_ADC:
	        	/* Do Nothing */
	        	break;
	        case PortConfig_Mode_UART:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_UART << (Port_PortChannels[i].pin_num * 4));
	        	break;
	        case PortConfig_Mode_SSI:
	        	if ( (Port_PortChannels[i].port_num == PortConfig_PORTD_NUM ) && ( Port_PortChannels[i].pin_num <= PortConfig_PIN3_NUM))
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_UART << (Port_PortChannels[i].pin_num * 4));
	        	}
	        	else
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_SSI << (Port_PortChannels[i].pin_num * 4));
	        	}
	        	break;
	        case PortConfig_Mode_I2C:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_I2C << (Port_PortChannels[i].pin_num * 4));
	        	break;
	        case PortConfig_Mode_M0PWM:
	        	if(((Port_PortChannels[i].port_num == PortConfig_PORTD_NUM ) && ( Port_PortChannels[i].pin_num <= PortConfig_PIN6_NUM)) ||
	        		((Port_PortChannels[i].port_num == PortConfig_PORTF_NUM ) && ( Port_PortChannels[i].pin_num <= PortConfig_PIN2_NUM)))
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_M0FAULT << (Port_PortChannels[i].pin_num * 4));
	        	}
	        	else
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_M0PWM << (Port_PortChannels[i].pin_num * 4));
	        	}
	        	break;
	        case PortConfig_Mode_M1PWM:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_M1PWM << (Port_PortChannels[i].pin_num * 4));
	        	break;
	        case PortConfig_Mode_IDX_PHASE:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConfig_Mode_IDX_PHASE << (Port_PortChannels[i].pin_num * 4));
	        	break;
	        case PortConfig_Mode_TIMER:
	        	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConfig_Mode_TIMER << (Port_PortChannels[i].pin_num * 4));
	        	break;
	        case PortConfig_Mode_CAN:
	        	if (((Port_PortChannels[i].port_num == PortConfig_PORTF_NUM ) && ( Port_PortChannels[i].pin_num <= PortConfig_PIN0_NUM)) ||
	        			((Port_PortChannels[i].port_num == PortConfig_PORTF_NUM ) && ( Port_PortChannels[i].pin_num <= PortConfig_PIN3_NUM)))
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConfig_Mode_I2C << (Port_PortChannels[i].pin_num * 4));
	        	}
	        	else if (((Port_PortChannels[i].port_num == PortConfig_PORTF_NUM ) && ( Port_PortChannels[i].pin_num <= PortConfig_PIN0_NUM)) ||
	        			((Port_PortChannels[i].port_num == PortConfig_PORTD_NUM ) && ( Port_PortChannels[i].pin_num <= PortConfig_PIN7_NUM)))
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConfig_Mode_NMI << (Port_PortChannels[i].pin_num * 4));
	        	}
	        	else if (((Port_PortChannels[i].port_num == PortConfig_PORTF_NUM ) && ( Port_PortChannels[i].pin_num <= PortConfig_PIN4_NUM)) ||
	        			((Port_PortChannels[i].port_num == PortConfig_PORTD_NUM ) && ( Port_PortChannels[i].pin_num <= PortConfig_PIN3_NUM))  ||
						((Port_PortChannels[i].port_num == PortConfig_PORTD_NUM ) && ( Port_PortChannels[i].pin_num <= PortConfig_PIN2_NUM))  ||
						((Port_PortChannels[i].port_num == PortConfig_PORTC_NUM ) && ( Port_PortChannels[i].pin_num <= PortConfig_PIN6_NUM))  ||
						((Port_PortChannels[i].port_num == PortConfig_PORTC_NUM ) && ( Port_PortChannels[i].pin_num <= PortConfig_PIN7_NUM)))

	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConfig_Mode_USB << (Port_PortChannels[i].pin_num * 4));
	        	}
	        	else
	        	{
	        		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConfig_Mode_CAN << (Port_PortChannels[i].pin_num * 4));
	        	}
	        	break;
	        }

	        if(Port_PortChannels[i].direction == PORT_PIN_OUT)
	            {
	        	SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[i].pin_num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

	                if(Port_PortChannels[i].initial_value == STD_HIGH)
	                {
	                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PortChannels[i].pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
	                }
	                else
	                {
	                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PortChannels[i].pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
	                }
	            }
	            else if(Port_PortChannels[i].direction == PORT_PIN_IN)
	            {
	                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[i].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

	                if(Port_PortChannels[i].resistor == PULL_UP)
	                {
	                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PortChannels[i].pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
	                }
	                else if(Port_PortChannels[i].resistor == PULL_DOWN)
	                {
	                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PortChannels[i].pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
	                }
	                else
	                {
	                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PortChannels[i].pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
	                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PortChannels[i].pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
	                }
	            }
	            else
	            {
	                /* Do Nothing */
	            }

	 }
}
/************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): Pin,Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Set The Direction Of Pin.
************************************************************************************/
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction )
{
	boolean error = FALSE ;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check If Pin Is Correct */
	if (Pin >= PORT_CONFIGURED_CHANNLES)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,
		     PORT_E_PARAM_PIN);
		error = TRUE ;
	}
	else
	{
		/* DO Nothing */
	}
	if (Port_Status == PORT_NOT_INITIALIZED)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,
				     PORT_E_UNINIT);
		error = TRUE ;
	}
	else
	{
		/* Do Nothing */
	}
#endif
	volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
	volatile uint32 delay = 0;
	if (error == FALSE)
	{
		switch(Port_PortChannels[Pin].port_num)
		{
		case  PortConfig_PORTA_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		break;
		case  PortConfig_PORTB_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		break;
		case  PortConfig_PORTC_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		break;
		case  PortConfig_PORTD_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		break;
		case  PortConfig_PORTE_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		break;
		case  PortConfig_PORTF_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		break;
		}
		/* Enable clock for PORT and allow time for clock to start*/
		SYSCTL_REGCGC2_REG |= (1<<Port_PortChannels[Pin].port_num);
	    delay = SYSCTL_REGCGC2_REG;

	    if( ((Port_PortChannels[Pin].port_num == PortConfig_PORTD_NUM) && (Port_PortChannels[Pin].pin_num == PortConfig_PIN7_NUM)) || ((Port_PortChannels[Pin].port_num == PortConfig_PORTF_NUM) && (Port_PortChannels[Pin].pin_num == PortConfig_PIN0_NUM)) ) /* PD7 or PF0 */
	    {
	     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = Magic_Number;                     /* Unlock the GPIOCR register */
	     SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
	     }
	     else if( (Port_PortChannels[Pin].port_num == PortConfig_PORTC_NUM) && (Port_PortChannels[Pin].pin_num <= PortConfig_PIN3_NUM) ) /* PC0 to PC3 */
	     {
	     /* Do Nothing ...  this is the JTAG pins */
	     return;
	     }
	     else
	     {
	     /* Do Nothing ... No need to unlock the commit register for this pin */
	     }

		if (Direction == PORT_PIN_OUT)
		{
			SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[Pin].pin_num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
            #if (PORT_DEV_ERROR_DETECT == STD_ON)
	             if (BIT_IS_CLEAR(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[Pin].pin_num))
	             {
	            	 Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,
	            	 		     PORT_E_DIRECTION_UNCHANGEABLE);
	            	 		error = TRUE ;
	             }
	             else
	             {
	            	 /* Do Nothing */
	             }
            #endif

	          if(Port_PortChannels[Pin].initial_value == STD_HIGH)
	          {
	            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PortChannels[Pin].pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
	           }
	          else
	           {
	             CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PortChannels[Pin].pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
	            }
		   }
		else if (Direction == PORT_PIN_IN)
		{
			CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[Pin].pin_num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
            #if (PORT_DEV_ERROR_DETECT == STD_ON)
	             if (BIT_IS_SET(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[Pin].pin_num))
	             {
	            	 Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinDirection_SID,
	            	 		     PORT_E_DIRECTION_UNCHANGEABLE);
	            	 		error = TRUE ;
	             }
	             else
	             {
	            	 /* Do Nothing */
	             }
            #endif
	             if(Port_PortChannels[Pin].resistor == PULL_UP)
				   {
					   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PortChannels[Pin].pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
				   }
				   else if(Port_PortChannels[Pin].resistor == PULL_DOWN)
				   {
					   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PortChannels[Pin].pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
				   }
				   else
				   {
					   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PortChannels[Pin].pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
					   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PortChannels[Pin].pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
				   }
		}
		else
		{
			/* Do Nothing */
		}

	}
	else
	{
		/* Do Nothing */
	}
}

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): Non
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Refresh The Direction Of Pin.
************************************************************************************/
void Port_RefreshPortDirection(void)
{
	boolean error = FALSE ;

	#if (PORT_DEV_ERROR_DETECT == STD_ON)
	    if (Port_Status == PORT_NOT_INITIALIZED)
		{
			Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_RefreshPortDirection_SID,
					     PORT_E_UNINIT);
			error = TRUE ;
		}
		else
		{
			/* Do Nothing */
		}
	#endif
	if (error == FALSE)
	{
		volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
		volatile uint32 delay = 0;
		volatile uint8 i= 0 ;
			 for (i = 0 ; i < PORT_CONFIGURED_CHANNLES ;i ++)
			 {
			    switch(Port_PortChannels[i].port_num)
			    {
			        case  PortConfig_PORTA_NUM:
			        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
					 break;
				    case  PortConfig_PORTB_NUM:
					PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
					 break;
			    	case  PortConfig_PORTC_NUM:
					PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
					 break;
			        case  PortConfig_PORTD_NUM:
					PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
					 break;
			        case  PortConfig_PORTE_NUM:
			        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
					 break;
			        case  PortConfig_PORTF_NUM:
			        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
					 break;
			    }
			    /* Enable clock for PORT and allow time for clock to start*/
			        SYSCTL_REGCGC2_REG |= (1<<Port_PortChannels[i].port_num);
			        delay = SYSCTL_REGCGC2_REG;

			        if( ((Port_PortChannels[i].port_num == PortConfig_PORTD_NUM) && (Port_PortChannels[i].pin_num == PortConfig_PIN7_NUM)) || ((Port_PortChannels[i].port_num == PortConfig_PORTF_NUM) && (Port_PortChannels[i].pin_num == PortConfig_PIN0_NUM)) ) /* PD7 or PF0 */
			        {
			        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = Magic_Number;                     /* Unlock the GPIOCR register */
			        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PortChannels[i].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
			        }
			        else if( (Port_PortChannels[i].port_num == PortConfig_PORTC_NUM) && (Port_PortChannels[i].pin_num <= PortConfig_PIN3_NUM) ) /* PC0 to PC3 */
			        {
			        /* Do Nothing ...  this is the JTAG pins */
			        return;
			        }
			        else
			        {
			        /* Do Nothing ... No need to unlock the commit register for this pin */
			        }
			        if(Port_PortChannels[i].direction == PORT_PIN_OUT)
			        {
			       	SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[i].pin_num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
			            if(Port_PortChannels[i].initial_value == STD_HIGH)
			            {
			                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PortChannels[i].pin_num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
			            }
			            else
			            {
			                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PortChannels[i].pin_num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
			            }
			        }
			        else if(Port_PortChannels[i].direction == PORT_PIN_IN)
			        {
			            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortChannels[i].pin_num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

			            if(Port_PortChannels[i].resistor == PULL_UP)
			            {
			                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PortChannels[i].pin_num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
			            }
			            else if(Port_PortChannels[i].resistor == PULL_DOWN)
			            {
			                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PortChannels[i].pin_num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
			            }
			            else
			            {
			                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PortChannels[i].pin_num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
			                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PortChannels[i].pin_num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
			            }
			        }
			        else
			        {
			            /* Do Nothing */
			        }
			 }
	}
	else
	{
		/* Do Nothing */
	}

}
/************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): Non
* Parameters (inout): None
* Parameters (out): VersionInfo - Pointer to where to store the version information of this module
* Return value: None
* Description: Function to Get VersionInfo
************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
      #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if input pointer is not Null pointer */
	if(NULL_PTR == versioninfo)
	{
		/* Report to DET  */
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID,
				Port_GetVersionInfo_SID, PORT_E_PARAM_POINTER);
	}
	else if(Port_Status == PORT_NOT_INITIALIZED)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_GetVersionInfo_SID,
				 PORT_E_UNINIT);
	}
	else

        #endif /* (DIO_DEV_ERROR_DETECT == STD_ON) */
	{
		/* Copy the vendor Id */
		versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
		/* Copy the module Id */
		versioninfo->moduleID = (uint16)PORT_MODULE_ID;
		/* Copy Software Major Version */
		versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
		/* Copy Software Minor Version */
		versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
		/* Copy Software Patch Version */
		versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
	}
}
#endif
/************************************************************************************
* Service Name: Port_SetPinMode
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): Pin , Mode ;
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Change Mode Of Pin
************************************************************************************/
void Port_SetPinMode(Port_PinType Pin,Port_PinModeType Mode)
{
	boolean error = FALSE ;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check If Pin Is Correct */
	if (Pin > PORT_CONFIGURED_CHANNLES)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinMode_SID,
		     PORT_E_PARAM_PIN);
		error = TRUE ;
	}
	else
	{
		/* DO Nothing */
	}
	if ( Mode >= PORT_CONFIGURED_MODES)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinMode_SID,
				PORT_E_PARAM_INVALID_MODE);
				error = TRUE ;
	}
	else
	{
		/* Do Nothing */
	}
	if (Port_Status == PORT_NOT_INITIALIZED)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinMode_SID,
				     PORT_E_UNINIT);
		error = TRUE ;
	}
	else
	{
		/* Do Nothing */
	}
#endif
	volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
	volatile uint32 delay = 0;
	if (error == FALSE)
	{
		switch(Port_PortChannels[Pin].port_num)
		{
		case  PortConfig_PORTA_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		break;
		case  PortConfig_PORTB_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		break;
		case  PortConfig_PORTC_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		break;
		case  PortConfig_PORTD_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		break;
		case  PortConfig_PORTE_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		break;
		case  PortConfig_PORTF_NUM:
		PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		break;
		}
		/* Enable clock for PORT and allow time for clock to start*/
		SYSCTL_REGCGC2_REG |= (1<<Port_PortChannels[Pin].port_num);
	        delay = SYSCTL_REGCGC2_REG;

	    if( ((Port_PortChannels[Pin].port_num == PortConfig_PORTD_NUM) && (Port_PortChannels[Pin].pin_num == PortConfig_PIN7_NUM)) || ((Port_PortChannels[Pin].port_num == PortConfig_PORTF_NUM) && (Port_PortChannels[Pin].pin_num == PortConfig_PIN0_NUM)) ) /* PD7 or PF0 */
	    {
	     *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = Magic_Number;                     /* Unlock the GPIOCR register */
	     SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PortChannels[Pin].pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
	     }
	     else if( (Port_PortChannels[Pin].port_num == PortConfig_PORTC_NUM) && (Port_PortChannels[Pin].pin_num <= PortConfig_PIN3_NUM) ) /* PC0 to PC3 */
	     {
	     /* Do Nothing ...  this is the JTAG pins */
	     return;
	     }
	     else
	     {
	     /* Do Nothing ... No need to unlock the commit register for this pin */
	     }


	    if ( Mode == PortConfig_Mode_ADC) /*ADC Mode Select */
	    {
	    	SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].pin_num);/*SET Analog */
	    	CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].pin_num);/* Clear Digital */
	    }
	    else
	    {
	    	CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].pin_num);/*Clear Analog */
	        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortChannels[Pin].pin_num);   /* Set Digital */
	    }

	    if ( Mode == PortConfig_Mode_GPIO )
	    {
	    	CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].pin_num); /* Clear Alternate Function To Select GPIO*/
	    }
	    else
	    {
	    	SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortChannels[Pin].pin_num); /* Set Alternate Function To Use Other Function Of Pin */
	    }

	    switch ( Mode)
	    {
	    case PortConfig_Mode_GPIO:
	    	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortChannels[Pin].pin_num * 4));
	    	break;
	    case PortConfig_Mode_ADC:
	    	/* Do Nothing */
	    	break;
	    case PortConfig_Mode_UART:
	    	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_UART << (Port_PortChannels[Pin].pin_num * 4));
	    	break;
	    case PortConfig_Mode_SSI:
	    	if ( (Port_PortChannels[Pin].port_num == PortConfig_PORTD_NUM ) && ( Port_PortChannels[Pin].pin_num <= PortConfig_PIN3_NUM))
	    	{
	    		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_UART << (Port_PortChannels[Pin].pin_num * 4));
	    	}
	    	else
	    	{
	    		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_SSI << (Port_PortChannels[Pin].pin_num * 4));
	    	}
	    	break;
	    case PortConfig_Mode_I2C:
	    	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_I2C << (Port_PortChannels[Pin].pin_num * 4));
	    	break;
	    case PortConfig_Mode_M0PWM:
	    	if(((Port_PortChannels[Pin].port_num == PortConfig_PORTD_NUM ) && ( Port_PortChannels[Pin].pin_num <= PortConfig_PIN6_NUM)) ||
	    		((Port_PortChannels[Pin].port_num == PortConfig_PORTF_NUM ) && ( Port_PortChannels[Pin].pin_num <= PortConfig_PIN2_NUM)))
	    	{
	    		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_M0FAULT << (Port_PortChannels[Pin].pin_num * 4));
	    	}
	    	else
	    	{
	    		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_M0PWM << (Port_PortChannels[Pin].pin_num * 4));
	    	}
	    	break;
	    case PortConfig_Mode_M1PWM:
	    	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32)PortConfig_Mode_M1PWM << (Port_PortChannels[Pin].pin_num * 4));
	    	break;
	    case PortConfig_Mode_IDX_PHASE:
	    	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConfig_Mode_IDX_PHASE << (Port_PortChannels[Pin].pin_num * 4));
	    	break;
	    case PortConfig_Mode_TIMER:
	    	*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConfig_Mode_TIMER << (Port_PortChannels[Pin].pin_num * 4));
	    	break;
	    case PortConfig_Mode_CAN:
	    	if (((Port_PortChannels[Pin].port_num == PortConfig_PORTF_NUM ) && ( Port_PortChannels[Pin].pin_num <= PortConfig_PIN0_NUM)) ||
	    			((Port_PortChannels[Pin].port_num == PortConfig_PORTF_NUM ) && ( Port_PortChannels[Pin].pin_num <= PortConfig_PIN3_NUM)))
	    	{
	    		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConfig_Mode_I2C << (Port_PortChannels[Pin].pin_num * 4));
	    	}
	    	else if (((Port_PortChannels[Pin].port_num == PortConfig_PORTF_NUM ) && ( Port_PortChannels[Pin].pin_num <= PortConfig_PIN0_NUM)) ||
	    			((Port_PortChannels[Pin].port_num == PortConfig_PORTD_NUM ) && ( Port_PortChannels[Pin].pin_num <= PortConfig_PIN7_NUM)))
	    	{
	    		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConfig_Mode_NMI << (Port_PortChannels[Pin].pin_num * 4));
	    	}
	    	else if (((Port_PortChannels[Pin].port_num == PortConfig_PORTF_NUM ) && ( Port_PortChannels[Pin].pin_num <= PortConfig_PIN4_NUM)) ||
	    			((Port_PortChannels[Pin].port_num == PortConfig_PORTD_NUM ) && ( Port_PortChannels[Pin].pin_num <= PortConfig_PIN3_NUM))  ||
	   			((Port_PortChannels[Pin].port_num == PortConfig_PORTD_NUM ) && ( Port_PortChannels[Pin].pin_num <= PortConfig_PIN2_NUM))  ||
	   			((Port_PortChannels[Pin].port_num == PortConfig_PORTC_NUM ) && ( Port_PortChannels[Pin].pin_num <= PortConfig_PIN6_NUM))  ||
	   			((Port_PortChannels[Pin].port_num == PortConfig_PORTC_NUM ) && ( Port_PortChannels[Pin].pin_num <= PortConfig_PIN7_NUM)))

	    	{
	    		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConfig_Mode_USB << (Port_PortChannels[Pin].pin_num * 4));
	    	}
	    	else
	    	{
	    		*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= ((uint32) PortConfig_Mode_CAN << (Port_PortChannels[Pin].pin_num * 4));
	    	}
	    	break;
	    }
        #if (PORT_DEV_ERROR_DETECT == STD_ON)
	     /* Check If Mode Is Changed */
	    if (Mode == PortConfig_Mode_ADC)
	    {
	    	if (BIT_IS_CLEAR(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortChannels[Pin].pin_num))
	    	{
	    	Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinMode_SID,
	    						     PORT_E_MODE_UNCHANGEABLE);
	    		error = TRUE ;

	    	}
	    	else
	    	{
	    		/* Do Nothing */
	    	}

	    }
	    else
	    {
	    	if (((*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET)) >> ((Port_PortChannels[Pin].pin_num)*4) & 0x0000000F ) ==  Mode)
	    	{
	    		/* Do Nothing */
	    	}
	    	else
	    	{
	    		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SetPinMode_SID,
	    			    						     PORT_E_MODE_UNCHANGEABLE);
	    			    		error = TRUE ;

	    	}
	    }
        #endif

	}

}

