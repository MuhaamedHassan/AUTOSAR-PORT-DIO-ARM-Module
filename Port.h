/*
 * Port.h
 *
 *  Created on: Jan 4, 2022
 *      Author: Mohamed
 */

#ifndef PORT_H_
#define PORT_H_



/* Id for the company in the AUTOSAR
 * for example Mohamed Tarek's ID = 1000 :) */
#define PORT_VENDOR_ID    (1000U)

/* Port Module Id */
#define PORT_MODULE_ID    (124U)

/* Port Instance Id */
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
/* Macros For Port Directions */

/* Standard AUTOSAR types */
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Port <pre>-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"


/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/

/* Service ID For Port Init */
#define Port_Init_SID                   (uint8)0x00

/* Service ID For Set Pin Direction */
#define Port_SetPinDirection_SID        (uint8)0x01

/* Service ID For Refresh Direction */
#define Port_RefreshPortDirection_SID   (uint8)0x02

/* Service ID For GetVersion Info */
#define Port_GetVersionInfo_SID         (uint8)0x03

/* Service ID For SetPinMode */
#define Port_SetPinMode_SID             (uint8)0x04

/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/


/* Det Code For Invalid Port Pin ID */
#define PORT_E_PARAM_PIN              (uint8)0x0A

/*ErrorPin Not Configured As Changeable */
#define PORT_E_DIRECTION_UNCHANGEABLE (uint8)0x0B

/*Error While Init Port */
#define PORT_E_PARAM_CONFIG           (uint8)0x0C

/*Error When Trying to Change UnChangeable Mode */
#define PORT_E_PARAM_INVALID_MODE     (uint8)0x0D
#define PORT_E_MODE_UNCHANGEABLE      (uint8)0x0E

/*API Service Without Init First */
#define PORT_E_UNINIT                 (uint8)0x0F

/*Call With Null Pointer */
#define PORT_E_PARAM_POINTER          (uint8)0x010

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/
/* Description: Enum to hold PIN direction */
typedef enum
{
    PORT_PIN_IN,PORT_PIN_OUT
}Port_PinDirectionType;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

/* The Definition Of Port_PinType Used By Port Api */
typedef uint8 Port_PinType ;

/* The Definition Of Port_PinDirectionChangeable To Check If Pin Is Changeable Or Not */
typedef enum {
	DIRECTION_CHANGE_OFF , DIRECTION_CHANGE_ON
}Port_PinDirectionChangeable;

/* The Definition Of Port_PinmodeChangeable To Check If Pin Is Changeable Or Not */
typedef enum {
	MODE_CHANGE_OFF , MODE_CHANGE_ON
}Port_PinModeChangeable;


/* The Definition of Port_PinMode Type Used By Port Api */
typedef uint8 Port_PinModeType ;



/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *  3. the direction of pin --> INPUT or OUTPUT
 *  4. the internal resistor --> Disable, Pull up or Pull down
 *  5. the mode of pin -->GPIO,ADC,UART,......
 */
typedef struct
{
    uint8 port_num;
    uint8 pin_num;
    Port_PinDirectionType direction;
    Port_InternalResistor resistor;
    Port_PinDirectionChangeable direction_changeable ;
    uint8 initial_value;
    Port_PinModeType pin_mode ;
    Port_PinModeChangeable mode_changeable ;
}Port_ConfigChannel;

/* Data Structure Required For Init  The Port Driver */
typedef struct Port_ConfigType
{
	Port_ConfigChannel Channels[PORT_CONFIGURED_CHANNLES];
} Port_ConfigType;
/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/
/* Function For Port Initialization Of API */
void Port_Init(const Port_ConfigType* ConfigPtr);


#if (PORT_SET_PIN_DIRECTION == STD_ON)
/*Function To Change Direction Of Pin */
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction);
#endif

/*Function For Refresh Port Direction */
void Port_RefreshPortDirection(void);

/* Function for PORT Get Version Info API */
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo);
#endif

/* Function To Change Mode Of Pin */
void Port_SetPinMode(Port_PinType Pin,Port_PinModeType Mode);

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by Port and other modules */
#include "Port_PBcfg.h"




#endif /* PORT_H_ */
