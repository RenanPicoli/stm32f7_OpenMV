/**
  ******************************************************************************
  * @file    usbd_uvc.h
  * @author  MCD Application Team
  * @brief   header file for the usbd_UVC.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_UVC_H
#define __USB_UVC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup usbd_UVC
  * @brief This file is the Header file for usbd_UVC.c
  * @{
  */ 


/** @defgroup usbd_UVC_Exported_Defines
  * @{
  */ 
#define UVC_IN_EP                                   0x81  /* EP1 for data IN */
#define UVC_OUT_EP                                  0x01  /* EP1 for data OUT */
#define UVC_CMD_EP                                  0x82  /* EP2 for UVC commands */

/* UVC Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define UVC_DATA_HS_MAX_PACKET_SIZE                 512  /* Endpoint IN & OUT Packet size */
#define UVC_DATA_FS_MAX_PACKET_SIZE                 64  /* Endpoint IN & OUT Packet size */
#define UVC_CMD_PACKET_SIZE                         8  /* Control Endpoint Packet size */

#define USB_UVC_CONFIG_DESC_SIZ                     67
#define UVC_DATA_HS_IN_PACKET_SIZE                  UVC_DATA_HS_MAX_PACKET_SIZE
#define UVC_DATA_HS_OUT_PACKET_SIZE                 UVC_DATA_HS_MAX_PACKET_SIZE

#define UVC_DATA_FS_IN_PACKET_SIZE                  UVC_DATA_FS_MAX_PACKET_SIZE
#define UVC_DATA_FS_OUT_PACKET_SIZE                 UVC_DATA_FS_MAX_PACKET_SIZE

/*---------------------------------------------------------------------*/
/*  UVC definitions                                                    */
/*---------------------------------------------------------------------*/
#define UVC_SEND_ENCAPSULATED_COMMAND               0x00
#define UVC_GET_ENCAPSULATED_RESPONSE               0x01
#define UVC_SET_COMM_FEATURE                        0x02
#define UVC_GET_COMM_FEATURE                        0x03
#define UVC_CLEAR_COMM_FEATURE                      0x04
#define UVC_SET_LINE_CODING                         0x20
#define UVC_GET_LINE_CODING                         0x21
#define UVC_SET_CONTROL_LINE_STATE                  0x22
#define UVC_SEND_BREAK                              0x23

/**
  * @}
  */ 


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

/**
  * @}
  */ 
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
}USBD_UVC_LineCodingTypeDef;

typedef struct _USBD_UVC_Itf
{
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* Control)       (uint8_t, uint8_t * , uint16_t);   
  int8_t (* Receive)       (uint8_t *, uint32_t *);  

}USBD_UVC_ItfTypeDef;


typedef struct
{
  uint32_t data[UVC_DATA_HS_MAX_PACKET_SIZE/4];      /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;    
  uint8_t  *RxBuffer;  
  uint8_t  *TxBuffer;   
  uint32_t RxLength;
  uint32_t TxLength;    
  
  __IO uint32_t TxState;     
  __IO uint32_t RxState;    
}
USBD_UVC_HandleTypeDef;



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 
  
/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

extern USBD_ClassTypeDef  USBD_UVC;
#define USBD_UVC_CLASS    &USBD_UVC
/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t  USBD_UVC_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                      USBD_UVC_ItfTypeDef *fops);

uint8_t  USBD_UVC_SetTxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff,
                                      uint16_t length);

uint8_t  USBD_UVC_SetRxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff);
  
uint8_t  USBD_UVC_ReceivePacket      (USBD_HandleTypeDef *pdev);

uint8_t  USBD_UVC_TransmitPacket     (USBD_HandleTypeDef *pdev);
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif  /* __USB_UVC_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
