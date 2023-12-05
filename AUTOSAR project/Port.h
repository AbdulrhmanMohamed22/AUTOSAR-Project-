/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Post Build Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Abdelrhman Mohamed
 ******************************************************************************/

#ifndef PORT_H_
#define PORT_H_


/* Id for the company in the AUTOSAR
 * for example ID = 1000 :) */
#define PORT_VENDOR_ID    (1000U)

/* PORT Module Id */
#define PORT_MODULE_ID    (124U)

/* PORT Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)


/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and PORT Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* PORT Pre-Compile Configuration Header file */
#include "PORT_Cfg.h"

/* AUTOSAR Version checking between PORT_Cfg.h and PORT.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of PORT_Cfg.h does not match the expected version"
#endif

/* Software Version checking between PORT_Cfg.h and PORT.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of PORT_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/
/* Service ID for PORT Init */
#define PORT_INIT_SID         		          (uint8)0x00

/* Service ID for PORT Set Pin Direction */
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
#define PORT_SET_PIN_DIRECTION_SID                (uint8)0x01
#endif

/* Service ID for PORT Refresh Port Direction */
#define PORT_REFRESH_PORT_DIRECTION_SID           (uint8)0x02

/* Service ID for PORT GetVersionInfo */
#if (PORT_VERSION_INFO_API == STD_ON)
#define PORT_GET_VERSION_INFO_SID                 (uint8)0x03
#endif

/* Service ID for PORT Set Pin Mode */
#if (PORT_SET_PIN_MODE_API == STD_ON)
#define PORT_SET_PIN_MODE_SID     	          (uint8)0x04
#endif
   
/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* Det code to report Invalid Port Pin ID  */
#define PORT_E_PARAM_PIN   					 (uint8)0x0A

/* Det code to report Port Pin not configured as changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE  	     (uint8)0x0B

/* Det code to report API Port_Init service called with wrong parameter  */
#define PORT_E_PARAM_CONFIG                  (uint8)0x0C

/* Det code to report API Port_SetPinMode service called when mode is Invalid */
#define PORT_E_PARAM_INVALID_MODE            (uint8)0x0D

/* Det code to report API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE             (uint8)0x0E

/* Det code to report API service called without module initialization */
#define PORT_E_UNINIT                        (uint8)0x0F

/* Det code to report APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER                 (uint8)0x10

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/
/* Type definition for PORT_PinType used by the PORT APIs */
typedef uint8 Port_PinType;

/* Type definition for PORT_PortType used by the DIO APIs */
typedef uint8 Port_PortType;

/* Type definition for Different port pin modes */
typedef uint8 Port_PinModeType;

/* Type definition for digital pin */
typedef uint8 Port_PinDigitalMode;

/* Type definition for digital pin */
typedef uint8 Port_PinDirectionChangable;

/* Type definition for digital pin */
typedef uint8 Port_PinModeChangable;

/* Type definition for setting Direction of a PORT Pin */
typedef enum {
	PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirectionType;

/* Type definition for setting Internal Resistor of a PORT Pin */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

/* Structure containing the initialization data for PORT module */
typedef struct
{
	Port_PortType Port_Num;
	Port_PinType  Pin_Num;
	Port_PinDirectionType Pin_Direction;
	uint8 initial_value;
	Port_PinModeType Pin_Mode;
	Port_InternalResistor Resistor;
        Port_PinDirectionChangable Pin_DirectionChangable;
        Port_PinModeChangable  Pin_ModeChangable;
}Port_ConfigPins;

/* Structure containing the initialization data for PORT module */
typedef struct
{
	Port_ConfigPins Pins[PORT_CONFIGURED_CHANNLES];
}Port_ConfigType;

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/
/* Function for Initializes the PORT Driver module */
void Port_Init(const Port_ConfigType* ConfigPtr);

/* Function for Setting the PORT Pin Direction */
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction);
#endif

/* Function for Refreshes PORT direction */
void Port_RefreshPortDirection(void);

/* Function for PORT Get Version Info API */
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo);
#endif


/* Function for Setting the PORT Pin Mode */
#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode(Port_PinType Pin,Port_PinModeType Mode);
#endif

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/
/* Extern PB structures to be used by PORT and other modules */
extern const Port_ConfigType Port_Configuration;

/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/
#define PORTA_ID               0
#define PORTB_ID               1
#define PORTC_ID               2
#define PORTD_ID               3
#define PORTE_ID               4
#define PORTF_ID               5

#define PIN0_ID                0
#define PIN1_ID                1
#define PIN2_ID                2
#define PIN3_ID                3
#define PIN4_ID                4
#define PIN5_ID                5
#define PIN6_ID                6
#define PIN7_ID                7

#endif /* PORT_H_ */
