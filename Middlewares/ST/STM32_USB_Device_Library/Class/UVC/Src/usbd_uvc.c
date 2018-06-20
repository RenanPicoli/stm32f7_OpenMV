/**
  ******************************************************************************
  * @file    usbd_uvc.c
  * @author  MCD Application Team
  * @brief   This file provides the high layer firmware functions to manage the 
  *          following functionalities of the USB UVC Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as UVC Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *           
  *  @verbatim
  *      
  *          ===================================================================      
  *                                UVC Class Driver Description
  *          =================================================================== 
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus 
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as UVC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class
  * 
  *           These aspects may be enriched or modified for a specific user application.
  *          
  *            This driver doesn't implement the following aspects of the specification 
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
  *      
  *  @endverbatim
  *                                  
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

  /* BSPDependencies
  - "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
  - "stm32xxxxx_{eval}{discovery}_io.c"
  EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_uvc.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"

//Iliasam files
#include "usbd_video_core.h"
#include "uvc.h"
//#include "jprocess.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_UVC
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_UVC_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_UVC_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBD_UVC_Private_Macros
  * @{
  */ 

/**
  * @}
  */ 


/** @defgroup USBD_UVC_Private_FunctionPrototypes
  * @{
  */


static uint8_t  USBD_UVC_Init (USBD_HandleTypeDef *pdev,
                               uint8_t cfgidx);

static uint8_t  USBD_UVC_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx);

static uint8_t  USBD_UVC_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req);

static uint8_t  USBD_UVC_DataIn (USBD_HandleTypeDef *pdev,
                                 uint8_t epnum);

static uint8_t  USBD_UVC_DataOut (USBD_HandleTypeDef *pdev,
                                 uint8_t epnum);

static uint8_t  USBD_UVC_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  *USBD_UVC_GetFSCfgDesc (uint16_t *length);

static uint8_t  *USBD_UVC_GetHSCfgDesc (uint16_t *length);

static uint8_t  *USBD_UVC_GetOtherSpeedCfgDesc (uint16_t *length);

static uint8_t  *USBD_UVC_GetOtherSpeedCfgDesc (uint16_t *length);

uint8_t  *USBD_UVC_GetDeviceQualifierDescriptor (uint16_t *length);

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_UVC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x01,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */ 

/** @defgroup USBD_UVC_Private_Variables
  * @{
  */ 


/* UVC interface class callbacks structure */
USBD_ClassTypeDef  USBD_UVC =
{
  CLASS_TYPEDEF_TYPE,
  USBD_UVC_Init,
  USBD_UVC_DeInit,
  USBD_UVC_Setup,
  NULL,                 /* EP0_TxSent, */
  USBD_UVC_EP0_RxReady,
  USBD_UVC_DataIn,
  USBD_UVC_DataOut,
  NULL,
  NULL,
  NULL,     
  USBD_UVC_GetHSCfgDesc,
  USBD_UVC_GetFSCfgDesc,
  USBD_UVC_GetOtherSpeedCfgDesc,
  USBD_UVC_GetDeviceQualifierDescriptor,
};

/* USB UVC device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_UVC_CfgHSDesc[USB_UVC_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
  USB_UVC_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
  0x00,
  0x02,   /* bNumInterfaces: 2 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 0 mA */
  
  /*---------------------------------------------------------------------------*/
  
  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */
  
  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bUVCDC: spec release number */
  0x01,
  
  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x01,   /* bDataInterface: 1 */
  
  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x00,   /* bMasterInterface: Communication class interface */
  0x01,   /* bSlaveInterface0: Data Class Interface */
  
  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
  UVC_CMD_EP,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(UVC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(UVC_CMD_PACKET_SIZE),
  0x10,                           /* bInterval: */ 
  /*---------------------------------------------------------------------------*/
  
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  0x01,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: UVC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */
  
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  UVC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(UVC_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(UVC_DATA_HS_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  UVC_IN_EP,                         /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(UVC_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(UVC_DATA_HS_MAX_PACKET_SIZE),
  0x00                               /* bInterval: ignore for Bulk transfer */
} ;


/* USB UVC device Configuration Descriptor */
__ALIGN_BEGIN uint8_t USBD_UVC_CfgFSDesc[USB_UVC_CONFIG_DESC_SIZ] __ALIGN_END =
{
		 /* Configuration 1 */
		  USB_CONFIGURATION_DESC_SIZE,               // bLength                  9
		  USB_DESC_TYPE_CONFIGURATION,        		 // bDescriptorType          2
		  WBVAL(USB_VIDEO_DESC_SIZ),
		  0x02,                                      // bNumInterfaces           2
		  0x01,                                      // bConfigurationValue      1 ID of this configuration
		  0x00,                                      // iConfiguration           0 no description available
		  USB_CONFIG_BUS_POWERED ,                   // bmAttributes          0x80 Bus Powered
		  USB_CONFIG_POWER_MA(100),                  // bMaxPower              100 mA


		  /* Interface Association Descriptor */
		  UVC_INTERFACE_ASSOCIATION_DESC_SIZE,       // bLength                  8
		  USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE, // bDescriptorType         11
		  0x00,                                      // bFirstInterface          0
		  0x02,                                      // bInterfaceCount          2
		  CC_VIDEO,                                  // bFunctionClass          14 Video
		  SC_VIDEO_INTERFACE_COLLECTION,             // bFunctionSubClass        3 Video Interface Collection
		  PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
		  0x02,                                      // iFunction                2


		  /* VideoControl Interface Descriptor */


		  /* Standard VC Interface Descriptor  = interface 0 */
		  USB_INTERFACE_DESC_SIZE,                   // bLength                  9
		  USB_DESC_TYPE_INTERFACE,            		 // bDescriptorType          4
		  USB_UVC_VCIF_NUM,                          // bInterfaceNumber         0 index of this interface (VC)
		  0x00,                                      // bAlternateSetting        0 index of this setting
		  0x00,                                      // bNumEndpoints            0 no endpoints
		  CC_VIDEO,                                  // bInterfaceClass         14 Video
		  SC_VIDEOCONTROL,                           // bInterfaceSubClass       1 Video Control
		  PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
		  0x02,                                      // iFunction                2


		  /* Class-specific VC Interface Descriptor */
		  UVC_VC_INTERFACE_HEADER_DESC_SIZE(1),      // bLength                 13 12 + 1 (header + 1*interface
		  CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
		  VC_HEADER,                                 // bDescriptorSubtype      1 (HEADER)
		  WBVAL(UVC_VERSION),                        // bcdUVC                  1.10 or 1.00
		  WBVAL(VC_TERMINAL_SIZ),                    // wTotalLength            header+units+terminals
		  DBVAL(0x005B8D80),                         // dwClockFrequency  6.000000 MHz
		  0x01,                                      // bInCollection            1 one streaming interface
		  0x01,                                      // baInterfaceNr( 0)        1 VS interface 1 belongs to this VC interface


		  /* Input Terminal Descriptor (Camera) */
		  UVC_CAMERA_TERMINAL_DESC_SIZE(2),          // bLength                 17 15 + 2 controls
		  CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
		  VC_INPUT_TERMINAL,                         // bDescriptorSubtype       2 (INPUT_TERMINAL)
		  0x01,                                      // bTerminalID              1 ID of this Terminal
		  WBVAL(ITT_CAMERA),                         // wTerminalType       0x0201 Camera Sensor
		  0x00,                                      // bAssocTerminal           0 no Terminal associated
		  0x00,                                      // iTerminal                0 no description available
		  WBVAL(0x0000),                             // wObjectiveFocalLengthMin 0
		  WBVAL(0x0000),                             // wObjectiveFocalLengthMax 0
		  WBVAL(0x0000),                             // wOcularFocalLength       0
		  0x02,                                      // bControlSize             2
		  0x00, 0x00,                                // bmControls          0x0000 no controls supported

		  /* Output Terminal Descriptor */
		  UVC_OUTPUT_TERMINAL_DESC_SIZE(0),          // bLength                  9
		  CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
		  VC_OUTPUT_TERMINAL,                        // bDescriptorSubtype       3 (OUTPUT_TERMINAL)
		  0x02,                                      // bTerminalID              2 ID of this Terminal
		  WBVAL(TT_STREAMING),                       // wTerminalType       0x0101 USB streaming terminal
		  0x00,                                      // bAssocTerminal           0 no Terminal assiciated
		  0x01,                                      // bSourceID                1 input pin connected to output pin unit 1
		  0x00,                                      // iTerminal                0 no description available


		  /* Video Streaming (VS) Interface Descriptor */


		  /* Standard VS Interface Descriptor  = interface 1 */
		  // alternate setting 0 = Zero Bandwidth
		  USB_INTERFACE_DESC_SIZE,                   // bLength                  9
		  USB_DESC_TYPE_INTERFACE,            		 // bDescriptorType          4
		  USB_UVC_VSIF_NUM,                          // bInterfaceNumber         1 index of this interface
		  0x00,                                      // bAlternateSetting        0 index of this setting
		  0x00,                                      // bNumEndpoints            0 no EP used
		  CC_VIDEO,                                  // bInterfaceClass         14 Video
		  SC_VIDEOSTREAMING,                         // bInterfaceSubClass       2 Video Streaming
		  PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
		  0x00,                                      // iInterface               0 no description available


		  /* Class-specific VS Header Descriptor (Input) */
		  UVC_VS_INTERFACE_INPUT_HEADER_DESC_SIZE(1,1),// bLength               14 13 + (1*1) (no specific controls used)
		  CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
		  VS_INPUT_HEADER,                           // bDescriptorSubtype       1 (INPUT_HEADER)
		  0x01,                                      // bNumFormats              1 one format descriptor follows
		  WBVAL(VC_HEADER_SIZ),
		  USB_ENDPOINT_IN(1),                        // bEndPointAddress      0x83 EP 3 IN
		  0x00,                                      // bmInfo                   0 no dynamic format change supported
		  0x02,                                      // bTerminalLink            2 supplies terminal ID 2 (Output terminal)
		  0x00,                                      // bStillCaptureMethod      0 NO supports still image capture
		  0x01,                                      // bTriggerSupport          0 HW trigger supported for still image capture
		  0x00,                                      // bTriggerUsage            0 HW trigger initiate a still image capture
		  0x01,                                      // bControlSize             1 one byte bmaControls field size
		  0x00,                                      // bmaControls(0)           0 no VS specific controls


		  /* Class-specific VS Format Descriptor  */
		  VS_FORMAT_UNCOMPRESSED_DESC_SIZE,     /* bLength 27*/
		  CS_INTERFACE,                         /* bDescriptorType : CS_INTERFACE */
		  VS_FORMAT_MJPEG,                      /* bDescriptorSubType : VS_FORMAT_MJPEG subtype */
		  0x01,                                 /* bFormatIndex : First (and only) format descriptor */
		  0x01,                                 /* bNumFrameDescriptors : One frame descriptor for this format follows. */
		  0x01,                                 /* bmFlags : Uses fixed size samples.. */
		  0x01,                                 /* bDefaultFrameIndex : Default frame index is 1. */
		  0x00,                                 /* bAspectRatioX : Non-interlaced stream not required. */
		  0x00,                                 /* bAspectRatioY : Non-interlaced stream not required. */
		  0x00,                                 /* bmInterlaceFlags : Non-interlaced stream */
		  0x00,                                 /* bCopyProtect : No restrictions imposed on the duplication of this video stream. */

		  /* Class-specific VS Frame Descriptor */
		  VS_FRAME_UNCOMPRESSED_DESC_SIZE,      /* bLength 30*/
		  CS_INTERFACE,                         /* bDescriptorType : CS_INTERFACE */
		  VS_FRAME_UNCOMPRESSED,                /* bDescriptorSubType : VS_FRAME_UNCOMPRESSED */
		  0x01,                                 /* bFrameIndex : First (and only) frame descriptor */
		  0x02,                                 /* bmCapabilities : Still images using capture method 0 are supported at this frame setting.D1: Fixed frame-rate. */
		  WBVAL(WIDTH),                         /* wWidth (2bytes): Width of frame is 128 pixels. */
		  WBVAL(HEIGHT),                        /* wHeight (2bytes): Height of frame is 64 pixels. */
		  DBVAL(MIN_BIT_RATE),                  /* dwMinBitRate (4bytes): Min bit rate in bits/s  */ // 128*64*16*5 = 655360 = 0x000A0000 //5fps
		  DBVAL(MAX_BIT_RATE),                  /* dwMaxBitRate (4bytes): Max bit rate in bits/s  */ // 128*64*16*5 = 655360 = 0x000A0000
		  DBVAL(MAX_FRAME_SIZE),                /* dwMaxVideoFrameBufSize (4bytes): Maximum video or still frame size, in bytes. */ // 128*64*2 = 16384 = 0x00004000
		  DBVAL(INTERVAL),				        /* dwDefaultFrameInterval : 1,000,000 * 100ns -> 10 FPS */ // 5 FPS -> 200ms -> 200,000 us -> 2,000,000 X 100ns = 0x001e8480
		  0x00,                                 /* bFrameIntervalType : Continuous frame interval */
		  DBVAL(INTERVAL),                      /* dwMinFrameInterval : 1,000,000 ns  *100ns -> 10 FPS */
		  DBVAL(INTERVAL),                      /* dwMaxFrameInterval : 1,000,000 ns  *100ns -> 10 FPS */
		  0x00, 0x00, 0x00, 0x00,               /* dwFrameIntervalStep : No frame interval step supported. */

		  /* Color Matching Descriptor */
		  VS_COLOR_MATCHING_DESC_SIZE,          /* bLength */
		  CS_INTERFACE,                         /* bDescriptorType : CS_INTERFACE */
		  0x0D,                                 /* bDescriptorSubType : VS_COLORFORMAT */
		  0x01,                                 /* bColorPrimarie : 1: BT.709, sRGB (default) */
		  0x01,                                 /* bTransferCharacteristics : 1: BT.709 (default) */
		  0x04,                                 /* bMatrixCoefficients : 1: BT. 709. */


		  /* Standard VS Interface Descriptor  = interface 1 */
		  // alternate setting 1 = operational setting
		  USB_INTERFACE_DESC_SIZE,                   // bLength                  9
		  USB_DESC_TYPE_INTERFACE,            		 // bDescriptorType          4
		  USB_UVC_VSIF_NUM,                          // bInterfaceNumber         1 index of this interface
		  0x01,                                      // bAlternateSetting        1 index of this setting
		  0x01,                                      // bNumEndpoints            1 one EP used
		  CC_VIDEO,                                  // bInterfaceClass         14 Video
		  SC_VIDEOSTREAMING,                         // bInterfaceSubClass       2 Video Streaming
		  PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
		  0x00,                                      // iInterface               0 no description available



		  /* Standard VS Isochronous Video data Endpoint Descriptor */
		  USB_ENDPOINT_DESC_SIZE,                   // bLength                  7
		  USB_DESC_TYPE_ENDPOINT,            		// bDescriptorType          5 (ENDPOINT)
		  USB_ENDPOINT_IN(1),                       // bEndpointAddress      0x83 EP 3 IN
		  USB_ENDPOINT_TYPE_ISOCHRONOUS,            // bmAttributes             1 isochronous transfer type
		  WBVAL(VIDEO_PACKET_SIZE),                 // wMaxPacketSize
		  0x01                                      // bInterval                1 one frame interval
} ;

__ALIGN_BEGIN uint8_t USBD_UVC_OtherSpeedCfgDesc[USB_UVC_CONFIG_DESC_SIZ] __ALIGN_END =
{ 
  0x09,   /* bLength: Configuation Descriptor size */
  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION,   
  USB_UVC_CONFIG_DESC_SIZ,
  0x00,
  0x02,   /* bNumInterfaces: 2 interfaces */
  0x01,   /* bConfigurationValue: */
  0x04,   /* iConfiguration: */
  0xC0,   /* bmAttributes: */
  0x32,   /* MaxPower 100 mA */  
  
  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0xEF,   /* bInterfaceClass: UVC */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */
  
  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bUVCDC: spec release number */
  0x01,
  
  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x01,   /* bDataInterface: 1 */
  
  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x00,   /* bMasterInterface: Communication class interface */
  0x01,   /* bSlaveInterface0: Data Class Interface */
  
  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT      ,   /* bDescriptorType: Endpoint */
  UVC_CMD_EP,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(UVC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
  HIBYTE(UVC_CMD_PACKET_SIZE),
  0xFF,                           /* bInterval: */
  
  /*---------------------------------------------------------------------------*/
  
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
  0x01,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: UVC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */
  
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
  UVC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  0x40,                              /* wMaxPacketSize: */
  0x00,
  0x00,                              /* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,     /* bDescriptorType: Endpoint */
  UVC_IN_EP,                        /* bEndpointAddress */
  0x02,                             /* bmAttributes: Bulk */
  0x40,                             /* wMaxPacketSize: */
  0x00,
  0x00                              /* bInterval */
};

/**
  * @}
  */ 

/** @defgroup USBD_UVC_Private_Functions
  * @{
  */ 

/**
  * @brief  USBD_UVC_Init
  *         Initialize the UVC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_UVC_Init (USBD_HandleTypeDef *pdev,
                               uint8_t cfgidx)
{
  uint8_t ret = 0;
  USBD_UVC_HandleTypeDef   *hUVC;
  
  if(pdev->dev_speed == USBD_SPEED_HIGH  ) 
  {  
    /* Open EP IN */
    USBD_LL_OpenEP(pdev,
                   UVC_IN_EP,
                   USBD_EP_TYPE_BULK,
                   UVC_DATA_HS_IN_PACKET_SIZE);
    
    /* Open EP OUT */
    USBD_LL_OpenEP(pdev,
                   UVC_OUT_EP,
                   USBD_EP_TYPE_BULK,
                   UVC_DATA_HS_OUT_PACKET_SIZE);
    
  }
  else
  {
    /* Open EP IN */
    USBD_LL_OpenEP(pdev,
                   UVC_IN_EP,
                   USBD_EP_TYPE_BULK,
                   UVC_DATA_FS_IN_PACKET_SIZE);
    
    /* Open EP OUT */
    USBD_LL_OpenEP(pdev,
                   UVC_OUT_EP,
                   USBD_EP_TYPE_BULK,
                   UVC_DATA_FS_OUT_PACKET_SIZE);
  }
  /* Open Command IN EP */
  USBD_LL_OpenEP(pdev,
                 UVC_CMD_EP,
                 USBD_EP_TYPE_INTR,
                 UVC_CMD_PACKET_SIZE);
  
    
  pdev->pClassData = USBD_malloc(sizeof (USBD_UVC_HandleTypeDef));
  
  if(pdev->pClassData == NULL)
  {
    ret = 1; 
  }
  else
  {
    hUVC = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
    
    /* Init  physical Interface components */
    ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->Init();
    
    /* Init Xfer states */
    hUVC->TxState =0;
    hUVC->RxState =0;
       
    if(pdev->dev_speed == USBD_SPEED_HIGH  ) 
    {      
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             UVC_OUT_EP,
                             hUVC->RxBuffer,
                             UVC_DATA_HS_OUT_PACKET_SIZE);
    }
    else
    {
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             UVC_OUT_EP,
                             hUVC->RxBuffer,
                             UVC_DATA_FS_OUT_PACKET_SIZE);
    }
    
    
  }
  return ret;
}

/**
  * @brief  USBD_UVC_Init
  *         DeInitialize the UVC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_UVC_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx)
{
  uint8_t ret = 0;
  
  /* Open EP IN */
  USBD_LL_CloseEP(pdev,
              UVC_IN_EP);
  
  /* Open EP OUT */
  USBD_LL_CloseEP(pdev,
              UVC_OUT_EP);
  
  /* Open Command IN EP */
  USBD_LL_CloseEP(pdev,
              UVC_CMD_EP);
  
  
  /* DeInit  physical Interface components */
  if(pdev->pClassData != NULL)
  {
    ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->DeInit();
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }
  
  return ret;
}

/**
  * @brief  USBD_UVC_Setup
  *         Handle the UVC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_UVC_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{
  USBD_UVC_HandleTypeDef   *hUVC = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  static uint8_t ifalt = 0;
    
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS :
    if (req->wLength)
    {
      if (req->bmRequest & 0x80)
      {
        ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest,
                                                          (uint8_t *)hUVC->data,
                                                          req->wLength);
          USBD_CtlSendData (pdev, 
                            (uint8_t *)hUVC->data,
                            req->wLength);
      }
      else
      {
        hUVC->CmdOpCode = req->bRequest;
        hUVC->CmdLength = req->wLength;
        
        USBD_CtlPrepareRx (pdev, 
                           (uint8_t *)hUVC->data,
                           req->wLength);
      }
      
    }
    else
    {
      ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest,
                                                        (uint8_t*)req,
                                                        0);
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        &ifalt,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      break;
    }
 
  default: 
    break;
  }
  return USBD_OK;
}

/**
  * @brief  USBD_UVC_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_UVC_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_UVC_HandleTypeDef   *hUVC = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  
  if(pdev->pClassData != NULL)
  {
    
    hUVC->TxState = 0;

    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

/**
  * @brief  USBD_UVC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_UVC_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{      
  USBD_UVC_HandleTypeDef   *hUVC = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  
  /* Get the received data length */
  hUVC->RxLength = USBD_LL_GetRxDataSize (pdev, epnum);
  
  /* USB data will be immediately processed, this allow next USB traffic being 
  NAKed till the end of the application Xfer */
  if(pdev->pClassData != NULL)
  {
    ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->Receive(hUVC->RxBuffer, &hUVC->RxLength);

    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}



/**
  * @brief  USBD_UVC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_UVC_EP0_RxReady (USBD_HandleTypeDef *pdev)
{ 
  USBD_UVC_HandleTypeDef   *hUVC = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  
  if((pdev->pUserData != NULL) && (hUVC->CmdOpCode != 0xFF))
  {
    ((USBD_UVC_ItfTypeDef *)pdev->pUserData)->Control(hUVC->CmdOpCode,
                                                      (uint8_t *)hUVC->data,
                                                      hUVC->CmdLength);
      hUVC->CmdOpCode = 0xFF;
      
  }
  return USBD_OK;
}

/**
  * @brief  USBD_UVC_GetFSCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_UVC_GetFSCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_UVC_CfgFSDesc);
  return USBD_UVC_CfgFSDesc;
}

/**
  * @brief  USBD_UVC_GetHSCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_UVC_GetHSCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_UVC_CfgHSDesc);
  return USBD_UVC_CfgHSDesc;
}

/**
  * @brief  USBD_UVC_GetCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_UVC_GetOtherSpeedCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_UVC_OtherSpeedCfgDesc);
  return USBD_UVC_OtherSpeedCfgDesc;
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *USBD_UVC_GetDeviceQualifierDescriptor (uint16_t *length)
{
  *length = sizeof (USBD_UVC_DeviceQualifierDesc);
  return USBD_UVC_DeviceQualifierDesc;
}

/**
* @brief  USBD_UVC_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: CD  Interface callback
  * @retval status
  */
uint8_t  USBD_UVC_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                      USBD_UVC_ItfTypeDef *fops)
{
  uint8_t  ret = USBD_FAIL;
  
  if(fops != NULL)
  {
    pdev->pUserData= fops;
    ret = USBD_OK;    
  }
  
  return ret;
}

/**
  * @brief  USBD_UVC_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
uint8_t  USBD_UVC_SetTxBuffer  (USBD_HandleTypeDef   *pdev,
                                uint8_t  *pbuff,
                                uint16_t length)
{
  USBD_UVC_HandleTypeDef   *hUVC = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  
  hUVC->TxBuffer = pbuff;
  hUVC->TxLength = length;
  
  return USBD_OK;  
}


/**
  * @brief  USBD_UVC_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t  USBD_UVC_SetRxBuffer  (USBD_HandleTypeDef   *pdev,
                                   uint8_t  *pbuff)
{
  USBD_UVC_HandleTypeDef   *hUVC = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  
  hUVC->RxBuffer = pbuff;
  
  return USBD_OK;
}

/**
  * @brief  USBD_UVC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  USBD_UVC_TransmitPacket(USBD_HandleTypeDef *pdev)
{      
  USBD_UVC_HandleTypeDef   *hUVC = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  
  if(pdev->pClassData != NULL)
  {
    if(hUVC->TxState == 0)
    {
      /* Tx Transfer in progress */
      hUVC->TxState = 1;
      
      /* Transmit next packet */
      USBD_LL_Transmit(pdev,
                       UVC_IN_EP,
                       hUVC->TxBuffer,
                       hUVC->TxLength);
      
      return USBD_OK;
    }
    else
    {
      return USBD_BUSY;
    }
  }
  else
  {
    return USBD_FAIL;
  }
}


/**
  * @brief  USBD_UVC_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_UVC_ReceivePacket(USBD_HandleTypeDef *pdev)
{      
  USBD_UVC_HandleTypeDef   *hUVC = (USBD_UVC_HandleTypeDef*) pdev->pClassData;
  
  /* Suspend or Resume USB Out process */
  if(pdev->pClassData != NULL)
  {
    if(pdev->dev_speed == USBD_SPEED_HIGH  ) 
    {      
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             UVC_OUT_EP,
                             hUVC->RxBuffer,
                             UVC_DATA_HS_OUT_PACKET_SIZE);
    }
    else
    {
      /* Prepare Out endpoint to receive next packet */
      USBD_LL_PrepareReceive(pdev,
                             UVC_OUT_EP,
                             hUVC->RxBuffer,
                             UVC_DATA_FS_OUT_PACKET_SIZE);
    }
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
