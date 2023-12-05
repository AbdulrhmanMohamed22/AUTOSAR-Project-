/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Abdelrhman Mohamed
 ******************************************************************************/


#include "Port.h"
#include "Port_Regs.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and PORT Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

STATIC const Port_ConfigPins * Port_PortPins = NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): ConfigPtr - Pointer to configuration set.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function for Initializes the PORT Driver module.
************************************************************************************/
void Port_Init(const Port_ConfigType * ConfigPtr)
{   
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    volatile uint32 delay = 0;
    uint8 PinID;
    
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    if(NULL_PTR == ConfigPtr)
    {
      Det_ReportError(PORT_MODULE_ID,PORT_INSTANCE_ID,PORT_INIT_SID,PORT_E_PARAM_CONFIG );
    }
    else
#endif      
    {
      Port_PortPins = ConfigPtr->Pins;
    } 

      /* for loop to configure all port pins */
      for(PinID=0;PinID<PORT_CONFIGURED_CHANNLES;PinID++)
      {
      
      switch(Port_PortPins[PinID].Port_Num)
      {
        case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		 break;
	case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		 break;
	case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
        case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
        case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
      }
      /* Enable clock for PORT and allow time for clock to start*/
      SYSCTL_REGCGC2_REG |= (1<<Port_PortPins[PinID].Port_Num);
      delay = SYSCTL_REGCGC2_REG;
      
      if(((Port_PortPins[PinID].Port_Num == 3) && (Port_PortPins[PinID].Pin_Num == 7)) || ((Port_PortPins[PinID].Port_Num == 5) && (Port_PortPins[PinID].Pin_Num == 0)) ) /* PD7 or PF0 */
      {
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
      }
      else if( (Port_PortPins[PinID].Port_Num == 2) && (Port_PortPins[PinID].Pin_Num <= 3) ) /* PC0 to PC3 */
      {
        /* Do Nothing ...  this is the JTAG pins */
      }
      else
      {
        /* Do Nothing ... No need to unlock the commit register for this pin */
      }
      
      if(Port_PortPins[PinID].Pin_Direction == PORT_PIN_OUT)
      {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
        
        if(Port_PortPins[PinID].initial_value == STD_HIGH)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);          /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);        /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
        }
      }
      else if(Port_PortPins[PinID].Pin_Direction == PORT_PIN_IN)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        if(Port_PortPins[PinID].Resistor == PULL_UP)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);       /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
        }
        else if(Port_PortPins[PinID].Resistor == PULL_DOWN)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);     /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
        }
        else
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);     /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);   /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
        }
      }
      else
      {
        /* Do Nothing */
      }
      
          
      if(Port_PortPins[PinID].Pin_Mode == PORT_PIN_MODE_DIO)
      {  
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);     /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);            /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortPins[PinID].Pin_Num * 4));    /* Clear the PMCx bits for this pin */
      }
      else if (Port_PortPins[PinID].Pin_Mode == PORT_PIN_MODE_ADC)
      {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);      /* Set the corresponding bit in the GPIOAMSEL register to enaable analog functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);     /* clear the corresponding bit in the GPIODEN register to disable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);             /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
      }
      else if (Port_PortPins[PinID].Pin_Mode == PORT_PIN_MODE_UART)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);    /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);       /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);             /* Enable Alternative function for this pin by set the corresponding bit in GPIOAFSEL register */
        if(((Port_PortPins[PinID].Port_Num == PORTC_ID) && (Port_PortPins[PinID].Pin_Num == PIN4_ID)) || ((Port_PortPins[PinID].Port_Num == PORTC_ID) && (Port_PortPins[PinID].Pin_Num == PIN5_ID)))
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_PortPins[PinID].Pin_Num * 4));     /* Set the PMCx bits for this pin */
        }
        else 
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_PortPins[PinID].Pin_Num * 4));     /* Set the PMCx bits for this pin */
        }
      }
      else if (Port_PortPins[PinID].Pin_Mode == PORT_PIN_MODE_I2C)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);    /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);       /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);             /* Enable Alternative function for this pin by Set the corresponding bit in GPIOAFSEL register */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_PortPins[PinID].Pin_Num * 4));    /* Set the PMCx bits for this pin */
      }
      else if (Port_PortPins[PinID].Pin_Mode == PORT_PIN_MODE_CAN)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);    /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);       /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);             /* Enable Alternative function for this pin by Set the corresponding bit in GPIOAFSEL register */
        if(((Port_PortPins[PinID].Port_Num == PORTF_ID) && (Port_PortPins[PinID].Pin_Num == PIN0_ID)) || ((Port_PortPins[PinID].Port_Num == PORTF_ID) && (Port_PortPins[PinID].Pin_Num == PIN3_ID)))
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_PortPins[PinID].Pin_Num * 4));   /* Set the PMCx bits for this pin */
        }
        else 
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_PortPins[PinID].Pin_Num * 4));   /* Set the PMCx bits for this pin */
        }
      }
      else if (Port_PortPins[PinID].Pin_Mode == PORT_PIN_MODE_USB)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);    /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);       /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);             /* Enable Alternative function for this pin by Set the corresponding bit in GPIOAFSEL register */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_PortPins[PinID].Pin_Num * 4));    /* Set the PMCx bits for this pin */
      }
      else if (Port_PortPins[PinID].Pin_Mode == PORT_PIN_MODE_SSI)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);    /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);       /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);             /* Enable Alternative function for this pin by Set the corresponding bit in GPIOAFSEL register */
        
        if(((Port_PortPins[PinID].Port_Num == PORTD_ID) && (Port_PortPins[PinID].Pin_Num == PIN0_ID)) || ((Port_PortPins[PinID].Port_Num == PORTD_ID) && (Port_PortPins[PinID].Pin_Num == PIN1_ID))/
           ((Port_PortPins[PinID].Port_Num == PORTD_ID) && (Port_PortPins[PinID].Pin_Num == PIN2_ID)) || ((Port_PortPins[PinID].Port_Num == PORTD_ID) && (Port_PortPins[PinID].Pin_Num == PIN3_ID)))
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_PortPins[PinID].Pin_Num * 4));     /* Set the PMCx bits for this pin */
        }
        else 
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_PortPins[PinID].Pin_Num * 4));     /* Set the PMCx bits for this pin */
        }
      }
      else if (Port_PortPins[PinID].Pin_Mode == PORT_PIN_MODE_PWM)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);    /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);       /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);             /* Enable Alternative function for this pin by Set the corresponding bit in GPIOAFSEL register */
        
        if(((Port_PortPins[PinID].Port_Num == PORTB_ID) && (Port_PortPins[PinID].Pin_Num == PIN4_ID)) || ((Port_PortPins[PinID].Port_Num == PORTB_ID) && (Port_PortPins[PinID].Pin_Num == PIN5_ID))/
           ((Port_PortPins[PinID].Port_Num == PORTB_ID) && (Port_PortPins[PinID].Pin_Num == PIN6_ID)) || ((Port_PortPins[PinID].Port_Num == PORTB_ID) && (Port_PortPins[PinID].Pin_Num == PIN7_ID))/
           ((Port_PortPins[PinID].Port_Num == PORTC_ID) && (Port_PortPins[PinID].Pin_Num == PIN4_ID)) || ((Port_PortPins[PinID].Port_Num == PORTC_ID) && (Port_PortPins[PinID].Pin_Num == PIN5_ID))/
           ((Port_PortPins[PinID].Port_Num == PORTD_ID) && (Port_PortPins[PinID].Pin_Num == PIN0_ID)) || ((Port_PortPins[PinID].Port_Num == PORTD_ID) && (Port_PortPins[PinID].Pin_Num == PIN1_ID))/
           ((Port_PortPins[PinID].Port_Num == PORTE_ID) && (Port_PortPins[PinID].Pin_Num == PIN4_ID)) || ((Port_PortPins[PinID].Port_Num == PORTE_ID) && (Port_PortPins[PinID].Pin_Num == PIN5_ID)))
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_PortPins[PinID].Pin_Num * 4));     /* Set the PMCx bits for this pin */
        }
        else 
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_PortPins[PinID].Pin_Num * 4));     /* Set the PMCx bits for this pin */
        }
      }
      else 
      {
        /* No Action Required */
      }
      }/* End of For loop */
      
      Port_Status = PORT_INITIALIZED;
}

/************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin - Port Pin ID number 
*                  Direction - Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction.
************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction)
{
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    boolean error = FALSE;
    
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    
    /* Check if the Driver is Not initialized before using this function */
    if(PORT_NOT_INITIALIZED == Port_Status)
    {
      /* Report to DET  */
      Det_ReportError(PORT_MODULE_ID,
                      PORT_INSTANCE_ID,
                      PORT_SET_PIN_DIRECTION_SID,
                      PORT_E_UNINIT );
      error = TRUE;
    }
    else
    {
      /* No Action Required */
    }    
   
    /* Check if Port Pin ID is Incorrect */
    if(PORT_CONFIGURED_CHANNLES <= Pin)
    {
      /* Report to DET  */
      Det_ReportError(PORT_MODULE_ID,
                      PORT_INSTANCE_ID,
                      PORT_SET_PIN_DIRECTION_SID,
                      PORT_E_PARAM_PIN );
      error = TRUE;
    }
    else
    {
      /* No Action Required */
    }
    /* Check if Port Pin not configured as changeable */
    if(Port_PortPins[Pin].Pin_DirectionChangable == STD_OFF)
    {
      /* Report to DET  */
      Det_ReportError(PORT_MODULE_ID,
                      PORT_INSTANCE_ID,
                      PORT_SET_PIN_DIRECTION_SID,
                      PORT_E_DIRECTION_UNCHANGEABLE );
      error = TRUE;
    }
    else
    {
      /* No Action Required */
    }    
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */

    if(FALSE == error)
    {    
      switch(Port_PortPins[Pin].Port_Num)
      {    
      case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
      break;
      case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
      break;
      case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
      break;
      case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
      break;
      case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
      break;
      case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
      break;
      }    
    
    if(PORT_PIN_IN == Direction)
    {
      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
    }
    else
    {
      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);               /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
    }
  
    }
}
#endif /* (PORT_SET_PIN_DIRECTION_API == STD_ON) */

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction..
************************************************************************************/
void Port_RefreshPortDirection(void) 
{
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    uint8 PinID;           
    boolean error = FALSE;
    
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    
    /* Check if the Driver is Not Initialized before using this function */
    if(PORT_NOT_INITIALIZED == Port_Status)
    {
      /* Report to DET  */
      Det_ReportError(PORT_MODULE_ID,
                      PORT_INSTANCE_ID,
                      PORT_REFRESH_PORT_DIRECTION_SID,
                      PORT_E_UNINIT );
      error = TRUE;
    }
    else
    {
      /* No Action Required */
    }        
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */

    if(error == FALSE)       
    {
      /* for loop to configure all port pins */ 
      for(PinID=0;PinID<PORT_CONFIGURED_CHANNLES;PinID++)
      {  
        switch(Port_PortPins[PinID].Port_Num)
        {  
        case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */	  
        break;
        case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
        break;
        case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
        break;
        case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
        break;
        case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
        break;
        case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
        break;   
        }
        
        if( ((Port_PortPins[PinID].Port_Num == 3) && (Port_PortPins[PinID].Pin_Num == 7)) || ((Port_PortPins[PinID].Port_Num == 5) && (Port_PortPins[PinID].Pin_Num == 0)) ) /* PD7 or PF0 */
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
        }
        else if( (Port_PortPins[PinID].Port_Num == 2) && (Port_PortPins[PinID].Pin_Num <= 3) ) /* PC0 to PC3 */
        {
          /* Do Nothing ...  this is the JTAG pins */
        }
        else
        {
          /* Do Nothing ... No need to unlock the commit register for this pin */
        } 
        /* Check if Port Pin not configured as changeable */  
        if(Port_PortPins[PinID].Pin_DirectionChangable == STD_OFF)
        {
          if(Port_PortPins[PinID].Pin_Direction == PORT_PIN_OUT)
          {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);                /* Set the corresponding bit in the GPIODIR register to configure it as output pin */    
          }
          else if(Port_PortPins[PinID].Pin_Direction == PORT_PIN_IN)
          {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[PinID].Pin_Num);             /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */    
          }
          else 
          {
            /* Do Nothing */ 
          }
        }
      }
    }
}


/************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo - Pointer to where to store the version information of this module.
* Return value: None
* Description: Returns the version information of this module.
************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo)
{
    boolean error = FALSE;
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    
    /* Check if the Driver is Not Initialized before using this function */
    if(PORT_NOT_INITIALIZED == Port_Status)
    {
      /* Report to DET  */
      Det_ReportError(PORT_MODULE_ID,
                      PORT_INSTANCE_ID,
                      PORT_GET_VERSION_INFO_SID,
                      PORT_E_UNINIT );
      error = TRUE;
    }
    else
    {
      /* No Action Required */
    }    
   
    /* Check if input pointer is not Null pointer */
    if(NULL_PTR == versioninfo)
    {
      /* Report to DET  */
      Det_ReportError(PORT_MODULE_ID,
                      PORT_INSTANCE_ID,
                      PORT_GET_VERSION_INFO_SID,
                      PORT_E_PARAM_POINTER );
      error = TRUE;
    }
    else
    {
      /* No Action Required */
    }
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
    if(FALSE == error)
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
#endif /* (PORT_VERSION_INFO_API == STD_ON) */

/************************************************************************************
* Service Name: Port_SetPinMode 
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin - Port Pin ID number 
*                  Mode - New Port Pin mode to be set on port pin
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode.
************************************************************************************/
#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode(Port_PinType Pin,Port_PinModeType Mode)
{
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    boolean error = FALSE;
    
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    
    /* Check if the Driver is initialized before using this function */
    if(PORT_NOT_INITIALIZED == Port_Status)
    {
      /* Report to DET  */
      Det_ReportError(PORT_MODULE_ID,
                      PORT_INSTANCE_ID,
                      PORT_SET_PIN_MODE_SID,
                      PORT_E_UNINIT );
      error = TRUE;
    }
    else
    {
      /* No Action Required */
    }    
   
    /* Check if Port Pin ID is Incorrect */
    if(PORT_CONFIGURED_CHANNLES <= Pin)
    {
      /* Report to DET  */
      Det_ReportError(PORT_MODULE_ID,
                      PORT_INSTANCE_ID,
                      PORT_SET_PIN_MODE_SID,
                      PORT_E_PARAM_PIN );
      error = TRUE;
    }
    else
    {
      /* No Action Required */
    }
    /* Check if Port Pin Mode is Incorrect */
    if((Mode > PORT_PIN_MODE_SSI ))
    {
      /* Report to DET  */
      Det_ReportError(PORT_MODULE_ID,
                      PORT_INSTANCE_ID,
                      PORT_SET_PIN_MODE_SID,
                      PORT_E_PARAM_INVALID_MODE);
      error = TRUE;
    }
    else
    {
      /* No Action Required */
    }
    /* Check if Port Pin Mode not configured as changeable */
    if(Port_PortPins[Pin].Pin_ModeChangable != PORT_PIN_MODE_CHANGEABLE)
    {
      /* Report to DET  */
      Det_ReportError(PORT_MODULE_ID,
                      PORT_INSTANCE_ID,
                      PORT_SET_PIN_MODE_SID,
                      PORT_E_MODE_UNCHANGEABLE);
      error = TRUE;
    }
    else
    {
      /* No Action Required */
    } 
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */

    if(error == FALSE)
    {
      switch(Port_PortPins[Pin].Port_Num)
      {
        case  0: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		 break;
	case  1: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		 break;
	case  2: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	case  3: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
        case  4: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
        case  5: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
      }
      if(Mode == PORT_PIN_MODE_DIO)
      {  
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);    /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);       /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);           /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << (Port_PortPins[Pin].Pin_Num * 4));   /* Clear the PMCx bits for this pin */
      }
      else if (Mode == PORT_PIN_MODE_ADC)
      {
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);      /* Set the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);     /* Clear the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);             /* Enable Alternative function for this pin by Set the corresponding bit in GPIOAFSEL register */
      }
      else if (Mode == PORT_PIN_MODE_UART)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);    /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);       /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);             /* Enable Alternative function for this pin by Set the corresponding bit in GPIOAFSEL register */
        if(((Port_PortPins[Pin].Port_Num == PORTC_ID) && (Port_PortPins[Pin].Pin_Num == PIN4_ID)) || ((Port_PortPins[Pin].Port_Num == PORTC_ID) && (Port_PortPins[Pin].Pin_Num == PIN5_ID)))
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_PortPins[Pin].Pin_Num * 4));     /* Set the PMCx bits for this pin */
        }
        else 
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_PortPins[Pin].Pin_Num * 4));     /* Set the PMCx bits for this pin */
        }
      }
      else if (Mode == PORT_PIN_MODE_I2C)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);    /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);       /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);             /* Enable Alternative function for this pin by Set the corresponding bit in GPIOAFSEL register */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_PortPins[Pin].Pin_Num * 4));    /* Set the PMCx bits for this pin */
      }
      else if (Mode == PORT_PIN_MODE_CAN)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);    /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);       /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);             /* Enable Alternative function for this pin by Set the corresponding bit in GPIOAFSEL register */
        if(((Port_PortPins[Pin].Port_Num == PORTF_ID) && (Port_PortPins[Pin].Pin_Num == PIN0_ID)) || ((Port_PortPins[Pin].Port_Num == PORTF_ID) && (Port_PortPins[Pin].Pin_Num == PIN3_ID)))
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000003 << (Port_PortPins[Pin].Pin_Num * 4));  /* Clear the PMCx bits for this pin */
        }
        else 
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_PortPins[Pin].Pin_Num * 4));  /* Clear the PMCx bits for this pin */
        }
      }
      else if (Mode == PORT_PIN_MODE_USB)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);    /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);       /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);             /* Enable Alternative function for this pin by Set the corresponding bit in GPIOAFSEL register */
        *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000008 << (Port_PortPins[Pin].Pin_Num * 4));    /* Set the PMCx bits for this pin */
      }
      else if (Mode == PORT_PIN_MODE_SSI)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);    /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);       /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);             /* Enable Alternative function for this pin by Set the corresponding bit in GPIOAFSEL register */
        
        if(((Port_PortPins[Pin].Port_Num == PORTD_ID) && (Port_PortPins[Pin].Pin_Num == PIN0_ID)) || ((Port_PortPins[Pin].Port_Num == PORTD_ID) && (Port_PortPins[Pin].Pin_Num == PIN1_ID))/
           ((Port_PortPins[Pin].Port_Num == PORTD_ID) && (Port_PortPins[Pin].Pin_Num == PIN2_ID)) || ((Port_PortPins[Pin].Port_Num == PORTD_ID) && (Port_PortPins[Pin].Pin_Num == PIN3_ID)))
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000001 << (Port_PortPins[Pin].Pin_Num * 4));   /* Set the PMCx bits for this pin */
        }
        else 
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000002 << (Port_PortPins[Pin].Pin_Num * 4));   /* Set the PMCx bits for this pin */
        }
      }
      else if (Mode == PORT_PIN_MODE_PWM)
      {
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);     /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);        /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);              /* Enable Alternative function for this pin by Set the corresponding bit in GPIOAFSEL register */
        
        if(((Port_PortPins[Pin].Port_Num == PORTB_ID) && (Port_PortPins[Pin].Pin_Num == PIN4_ID)) || ((Port_PortPins[Pin].Port_Num == PORTB_ID) && (Port_PortPins[Pin].Pin_Num == PIN5_ID))/
           ((Port_PortPins[Pin].Port_Num == PORTB_ID) && (Port_PortPins[Pin].Pin_Num == PIN6_ID)) || ((Port_PortPins[Pin].Port_Num == PORTB_ID) && (Port_PortPins[Pin].Pin_Num == PIN7_ID))/
           ((Port_PortPins[Pin].Port_Num == PORTC_ID) && (Port_PortPins[Pin].Pin_Num == PIN4_ID)) || ((Port_PortPins[Pin].Port_Num == PORTC_ID) && (Port_PortPins[Pin].Pin_Num == PIN5_ID))/
           ((Port_PortPins[Pin].Port_Num == PORTD_ID) && (Port_PortPins[Pin].Pin_Num == PIN0_ID)) || ((Port_PortPins[Pin].Port_Num == PORTD_ID) && (Port_PortPins[Pin].Pin_Num == PIN1_ID))/
           ((Port_PortPins[Pin].Port_Num == PORTE_ID) && (Port_PortPins[Pin].Pin_Num == PIN4_ID)) || ((Port_PortPins[Pin].Port_Num == PORTE_ID) && (Port_PortPins[Pin].Pin_Num == PIN5_ID)))
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000004 << (Port_PortPins[Pin].Pin_Num * 4));     /* Set the PMCx bits for this pin */
        }
        else 
        {
          *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x00000005 << (Port_PortPins[Pin].Pin_Num * 4));     /* Set the PMCx bits for this pin */
        }
      }
      else 
      {
        /* No Action Required */
      }
    }
  
  
}
#endif /* (PORT_SET_PIN_MODE_API == STD_ON) */


#endif
