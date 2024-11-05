/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : usb_desc.c
* Author             : MCD Application Team
* Version            : V2.2.1
* Date               : 09/22/2008
* Description        : Descriptors for Custom HID Demo
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* USB Standard Device Descriptor */
const u8 CustomHID_DeviceDescriptor[CUSTOMHID_SIZ_DEVICE_DESC] =
  {
    0x12,                       /*bLength设备描述长度 */
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType设备描述符类*/
    0x00,                       /*bcdUSB USB协议版本*/
    0x02,
    0x00,                       /*bDeviceClass 人机HID类*/
    0x00,                       /*bDeviceSubClass 子类*/
    0x00,                       /*bDeviceProtocol USB协议*/
    0x40,                       /*bMaxPacketSize40 端点0支持字节*/
    0x83,                       /*idVendor (0x0483) PID*/
    0x04,												//PID和VID都是大端对齐，与USB协议版本不同	
		
    0x01,                       /*idProduct = VID*/
    0xA0,										
		
    0x00,                       /*bcdDevice rel. 2.00 厂商产品版本号*/
    0x03,
    1,                          /*Index of string descriptor describing
                                              manufacturer  厂商字符串索引1*/
    2,                          /*Index of string descriptor describing
                                             product 产品字符串索引*/
    3,                          /*Index of string descriptor describing the
                                             device serial number 产品序列号字符串索引*/
    0x01                        /*bNumConfigurations  一个配置描述符*/
  }
  ; /* CustomHID_DeviceDescriptor */


/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const u8 CustomHID_ConfigDescriptor[CUSTOMHID_SIZ_CONFIG_DESC] =
  {
    0x09, /* bLength: Configuation Descriptor size 配置描述符长度*/
    USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration 配置描述符类*/
    CUSTOMHID_SIZ_CONFIG_DESC,	//整个集合的长度，占用2字节，小端对齐
    /* wTotalLength: Bytes returned */
    0x00,
    0x01,         /* bNumInterfaces: 1 interface 配置所支持的接口数量1个 */
    0x01,         /* bConfigurationValue: Configuration value 。set（get）_config获取描述符命令需要的参数*/
    0x00,         /* iConfiguration: Index of string descriptor describing//配置描述符字符串的索引值
                                 the configuration*/
    0xC0,         /* bmAttributes: Bus powered 自供电*/
                  /*Bus powered: 7th bit, Self Powered: 6th bit, Remote wakeup: 5th bit, reserved: 4..0 bits */
    0x32,         /* MaxPower 100 mA: this current is used for detecting Vbus */
//    0x96,         /* MaxPower 300 mA: this current is used for detecting Vbus 供电电流*2mA*/
    /************** Descriptor of Custom HID interface ****************/
    /* 09 */
    0x09,         /* bLength: Interface Descriptor size 接口描述符的长度*/
    USB_INTERFACE_DESCRIPTOR_TYPE,/* bDescriptorType: Interface descriptor type 接口描述符类*/
    0x00,         /* bInterfaceNumber: Number of Interface 接口编号*/
    0x00,         /* bAlternateSetting: Alternate setting 接口备用编号*/
    0x02,         /* bNumEndpoints 接口端点数量*/
    0x03,         /* bInterfaceClass: 接口类型HID */
    0x00,         /* bInterfaceSubClass : 1=BOOT, 0=no boot  */
    0x00,         /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse 接口协议*/
    0,            /* iInterface: Index of string descriptor 接口描述符的字符串索引*/
    /******************** Descriptor of Custom HID HID ********************/
    /* 18 */
    0x09,         /* bLength: HID Descriptor size HID描述符 */
    HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID HID描述符的长度*/
    0x10,         /* bcdHID: HID Class Spec release number HID版本1.1小端对齐*/
    0x01,
    0x00,         /* bCountryCode: Hardware target country 硬件国家*/
    0x01,         /* bNumDescriptors: Number of HID class descriptors to follow HID报告描述符数量*/
    0x22,         /* bDescriptorType HID报告描述类型*/
    CUSTOMHID_SIZ_REPORT_DESC,/* wItemLength: Total length of Report descriptor 报告描述符的长度*/
    0x00,
    /******************** Descriptor of Custom HID endpoints ******************/
    /* 27 */
    0x07,          /* bLength: Endpoint Descriptor size  端点描述符的数据数量*/
    USB_ENDPOINT_DESCRIPTOR_TYPE, /* bDescriptorType: 端点描述符的长度*/

    0x82,          /* bEndpointAddress: Endpoint Address (IN) 端点编号2，输入*/               
    //端点2 输入               // bit 3...0 : the endpoint number
    //向PC发数据               // bit 6...4 : reserved
                    // bit 7     : 0(OUT), 1(IN)
    0x03,          /* bmAttributes: Interrupt endpoint 中断传输*/
    SendLength%256,  /* wMaxPacketSize:SendLength Bytes max *///向PC发送SendLength字节数据
    SendLength/256,
    //0x20,          /* bInterval: Polling Interval (21 ms) 扫描间隔*/
		0x01,//1ms
    /* 34 */
    	
    0x07,	/* bLength: Endpoint Descriptor size 端点描数据述符*/
    USB_ENDPOINT_DESCRIPTOR_TYPE,	/* bDescriptorType: 端点描述符长度*/
			/*	Endpoint descriptor type */
    0x01,	/* bEndpointAddress: 端点号1*/
	//端点1 输出  		/*	Endpoint Address (OUT) */
	//接收PC向单片机发送的数据
	0x03,	/* bmAttributes: Interrupt endpoint 中断传输*/
    ReceiveLength%256,/* wMaxPacketSize: ReceiveLength Bytes max  自大传输数量*/
    ReceiveLength/256,//接收pc发过来的数据 ReceiveLength个 字节
    //0x10,	/* bInterval: Polling Interval (32 ms) */
		0x01,
    /* 41 */
  }
  ; /* CustomHID_ConfigDescriptor */
const u8 CustomHID_ReportDescriptor[CUSTOMHID_SIZ_REPORT_DESC] = 
{ 
//#ifdef 0 
//局部项目，ST项目
0x05, 0x8c, /* USAGE_PAGE (ST Page)  */ 
//用途：demo
0x09, 0x01, /* USAGE (Demo Kit) */ 
//开一个物理描述集合
0xa1, 0x01, /* COLLECTION (Application) */ 
/* 6 */ 
// The Input report 
//输入项目
0x09,0x03, // USAGE ID - Vendor defined 
//数据最小值0
0x15,0x00, // LOGICAL_MINIMUM (0)
//数据最大值1	
0x26,0x00, 0xFF, // LOGICAL_MAXIMUM (255) 
//8bit
0x75,0x08, // REPORT_SIZE (8) 
//255个字节
0x95,SendLength, // REPORT_COUNT :SendLength 
//输入的绝对值变量数据
0x81,0x02, // INPUT (Data,Var,Abs) 
//19
// The Output report 
0x09,0x04, // USAGE ID - Vendor defined 
0x15,0x00, // LOGICAL_MINIMUM (0) 
0x26,0x00,0xFF, // LOGICAL_MAXIMUM (255) 
0x75,0x08, // REPORT_SIZE (8) 
0x95,ReceiveLength, // REPORT_COUNT:ReceiveLength 
0x91,0x02, // OUTPUT (Data,Var,Abs) 
//32
// The Feature report
/*
0x09, 0x05, // USAGE ID - Vendor defined 
0x15,0x00, // LOGICAL_MINIMUM (0) 
0x26,0x00,0xFF, // LOGICAL_MAXIMUM (255) 
0x75,0x08, // REPORT_SIZE (8) 
0x95,0x02, // REPORT_COUNT (2) 
0xB1,0x02, 
*/
/* 45 */ 
0xc0 /* END_COLLECTION */ 
//#endif 
}; /* CustomHID_ReportDescriptor */ 


/* USB String Descriptors (optional) */
const u8 CustomHID_StringLangID[CUSTOMHID_SIZ_STRING_LANGID] =
  {
    CUSTOMHID_SIZ_STRING_LANGID,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09,
    0x04
  }
  ; /* LangID = 0x0409: U.S. English */

const u8 CustomHID_StringVendor[CUSTOMHID_SIZ_STRING_VENDOR] =
  {
    CUSTOMHID_SIZ_STRING_VENDOR, /* Size of Vendor string */
    USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
    // Manufacturer: "STMicroelectronics" 
    'Z', 0, 'I', 0, 'N', 0,'X', 0,'D', 0, '_', 0, 'H', 0,'I',0,'D',0
  };

const u8 CustomHID_StringProduct[CUSTOMHID_SIZ_STRING_PRODUCT] =
  {
    CUSTOMHID_SIZ_STRING_PRODUCT,          /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'Z', 0, 'I', 0, 'N', 0,'_',0 ,'Q', 0, 'U', 0, 'A', 0,'D',0
  };
u8 CustomHID_StringSerial[CUSTOMHID_SIZ_STRING_SERIAL] =
  {
    CUSTOMHID_SIZ_STRING_SERIAL,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'x', 0, 'x', 0, 'x', 0,'x', 0,'x', 0, 'x', 0, 'x', 0
  };

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

