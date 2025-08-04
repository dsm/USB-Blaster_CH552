// USB-Blaster instance on CH55x MCU.
// Author: Duan
// License: MIT
// Based on USB-MIDI by Zhiyuan Wan

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ch554.h"
#include "ch554_usb.h"
#include "debug.h"
#include "ftdi.h"
#include "spi.h"

// uncomment to enable AS mode
#define FTDI_AS_MODE
// uncomment to enable Hardware SPI
#define HARDWARE_SPI
// um: uncomment to replace memcpy by fastcpy
//#define FAST_COPY
// um: uncomment to use UID SN
#define UID_SN
// gpio
SBIT(LED, 0x90, 1); // P1.1
SBIT(TMS, 0xB0, 2); // P3.2
SBIT(NCS, 0x90, 4); // P1.4
SBIT(NCE, 0xB0, 4); // P3.4
SBIT(ASDO, 0xB0, 3); // P3.3
#define TCK SCK // P1.7
#define TDI MOSI // P1.5
#define TDO MISO // P1.6

// bit-bang
SBIT(P2B7, 0xA0, 7);
SBIT(P2B6, 0xA0, 6);
SBIT(P2B5, 0xA0, 5);
SBIT(P2B4, 0xA0, 4);
SBIT(P2B3, 0xA0, 3);
SBIT(P2B2, 0xA0, 2);
SBIT(P2B1, 0xA0, 1);
SBIT(P2B0, 0xA0, 0);

__xdata __at(0x0000) uint8_t transmit_buffer[128]; // fixed address for ringbuf
__xdata __at(0x0080) uint8_t receive_buffer[64];
__xdata __at(0x00C0) uint8_t serno[22];
__xdata __at(0x0100) uint8_t Ep0Buffer[0x08+2]; // Endpoint 0 OUT & IN buffer, must be an even address
__xdata __at(0x0140) uint8_t Ep1Buffer[0x40]; // Endpoint 1 IN buffer
__xdata __at(0x0180) uint8_t Ep2Buffer[0x40]; // Endpoint 2 OUT buffer, must be an even address

uint16_t SetupLen;
uint8_t SetupReq, Count, UsbConfig;
uint8_t vendor_control;
uint8_t send_dummy;

//const uint8_t* pDescr; // um: moved USB descriptor pointer
//USB_SETUP_REQ SetupReqBuf; // um: not used Temporarily save the Setup package
#define UsbSetupBuf ((PUSB_SETUP_REQ)Ep0Buffer)

__code uint8_t ftdi_rom[] = {
    0x00, 0x00, 0xfb, 0x09, 0x01, 0x60, 0x00, 0x04,
    0x80, 0xe1, 0x1c, 0x00, 0x00, 0x02, 0x94, 0x0e,
    0xa2, 0x18, 0xba, 0x12, 0x0e, 0x03, 0x41, 0x00,
    0x6c, 0x00, 0x74, 0x00, 0x65, 0x00, 0x72, 0x00,
    0x61, 0x00, 0x18, 0x03, 0x55, 0x00, 0x53, 0x00,
    0x42, 0x00, 0x2d, 0x00, 0x42, 0x00, 0x6c, 0x00,
    0x61, 0x00, 0x73, 0x00, 0x74, 0x00, 0x65, 0x00,
    0x72, 0x00, 0x12, 0x03, 0x43, 0x00, 0x30, 0x00,
    0x42, 0x00, 0x46, 0x00, 0x41, 0x00, 0x36, 0x00,
    0x44, 0x00, 0x37, 0x00, 0x02, 0x03, 0x01, 0x00,
    0x52, 0x45, 0x56, 0x42, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xb5, 0xb2
};

__code uint8_t DevDesc[] = {
    0x12, 0x01, 0x00, 0x02,
    0x00, 0x00, 0x00, 0x08,
    0xFB, 0x09, 0x01, 0x60, 0x00, 0x04, 0x01, 0x02, 0x03, /* VID PID bString */
    0x01
};

__code uint8_t CfgDesc[] = {
    0x09, 0x02, sizeof(CfgDesc) & 0xff, sizeof(CfgDesc) >> 8,
    0x01, 0x01, 0x00, 0x80, 0xe1,
    /* Interface Descriptor */
    0x09, 0x04, 0x00, 0x00, 0x02, 0xff, 0xff, 0xff, 0x00,
    /* Endpoint Descriptor */
    0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x01, // EP1_IN
    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x01, // EP2_OUT
};

/* USB String Descriptors (optional) */
unsigned char __code LangDes[] = { 0x04, 0x03, 0x09, 0x04 }; // EN_US
#ifndef UID_SN
unsigned char __code SerDes[] = {
    // TODO: variable SN.
    sizeof(SerDes), 0x03,
    'C', 0, '0', 0, 'B', 0, 'F', 0, 'A', 0, '6', 0, 'D', 0, '7', 0 /* "C0BFA6D7" */
};
#endif
unsigned char __code Prod_Des[] = {
    sizeof(Prod_Des),
    0x03,
    'U', 0, 'S', 0, 'B', 0, '-', 0, 'B', 0, 'l', 0, 'a', 0, 's', 0, 't', 0, 'e', 0, 'r', 0 /* "USB-Blaster" */
};

unsigned char __code Manuf_Des[] = {
    sizeof(Manuf_Des),
    0x03,
    'A', 0, 'l', 0, 't', 0, 'e', 0, 'r', 0, 'a', 0 /* Manufacturer: "Altera" */
};

//um: locate it in data
volatile __data uint8_t USBByteCount = 0; // Represents the data received by the USB endpoint
volatile __data uint8_t USBBufOutPoint = 0; // Get data pointer
volatile __data uint16_t sof_count = 0;
volatile __data uint8_t ep1_in_busy = 0; // Flag indicating whether the upload endpoint is busy
volatile __data uint8_t latency_timer = 4;

/*******************************************************************************
 * Function Name  : USBDeviceCfg()
 * Description  : Configure USB
 * Input      : None
 * Output    : None
 * Return    : None
 *******************************************************************************/
void USBDeviceCfg(void)
{
    USB_CTRL = 0x00; // Clear USB control register
    //USB_CTRL &= ~bUC_HOST_MODE; // um: This bit selects the device mode already zero
    USB_CTRL |= bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN; // USB device and internal pull-up enabled, automatically returns NAK during interrupt before interrupt flag is cleared
    USB_DEV_AD = 0x00; // Device address initialization
    //   USB_CTRL |= bUC_LOW_SPEED;
    //   UDEV_CTRL |= bUD_LOW_SPEED;      //Select low speed 1.5M mode
    USB_CTRL &= ~bUC_LOW_SPEED;
    UDEV_CTRL &= ~bUD_LOW_SPEED; // Select full speed 12M mode, the default mode
    UDEV_CTRL = bUD_PD_DIS; // Disable DP/DM pull-down resistor
    UDEV_CTRL |= bUD_PORT_EN; // Enable physical port
}

/*******************************************************************************
 * Function Name  : USBDeviceIntCfg()
 * Description  : USB device mode interrupt initialization
 * Input      : None
 * Output    : None
 * Return    : None
 *******************************************************************************/
void USBDeviceIntCfg(void)
{
    USB_INT_EN |= bUIE_SUSPEND; // Enable device hang interrupt
    USB_INT_EN |= bUIE_TRANSFER; // Enable USB transfer completion interrupt
    USB_INT_EN |= bUIE_BUS_RST; // Enable device mode USB bus reset interrupt
    USB_INT_EN |= bUIE_DEV_SOF; // For timeout count.
    USB_INT_FG |= 0x1F; // Clear interrupt flag
    IE_USB = 1; // Enable USB interrupt
    EA = 1; // Enable MCU interrupt
}

/*******************************************************************************
 * Function Name  : USBDeviceEndPointCfg()
 * Description  : USB device mode endpoint configuration, emulation compatible HID device,
 *      in addition to endpoint 0 control transmission, also includes endpoint 2 batch up and down transmission
 * Input      : None
 * Output    : None
 * Return    : None
 *******************************************************************************/
void USBDeviceEndPointCfg(void)
{
    UEP1_DMA = (uint16_t)Ep1Buffer; // Endpoint 1 IN data transmission address
    UEP2_DMA = (uint16_t)Ep2Buffer; // Endpoint 2 OUT data transmission address
    UEP2_3_MOD = 0x08;
    UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK; // Endpoint 2 automatically flips the synchronization flag bit, and OUT returns ACK
    UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK; // Endpoint 1 automatically flips the synchronization flag bit, and the IN transaction returns NAK
    UEP0_DMA = (uint16_t)Ep0Buffer; // Endpoint 0 data transmission address
    UEP4_1_MOD = 0x40; // Endpoint 1 upload buffer; endpoint 0 single 64-byte send and receive buffer
    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK; // Manual flip, OUT transaction returns ACK, IN transaction returns NAK
}
/*******************************************************************************
 * Function Name : fastcpy()
 * Description   : custom memcopy function for __code and __xdata only
 * Input         : ptr for src and dest, len (8 Bit)
 * Output        : none 
 * Attention     : src and dest params are switched for more compact code
*******************************************************************************/
#ifdef FAST_COPY
void fastcpy(void *src, void *dest, uint8_t len)
{ 
  dest; src; len;
   __asm
       mov    A,_fastcpy_PARM_3
       jnz    0$                      ; nothing todo -> exit
       ret
0$:
       push   ar7
       mov    R7,A                    ; Param 3 is loop counter
       push   _XBUS_AUX               ; save aux reg
       orl    _XBUS_AUX,#0x05         ; select DPTR1 + autoinc
       mov    DPL,_fastcpy_PARM_2+0   ; and load &dest to DPTR1
       mov    DPH,_fastcpy_PARM_2+1   ;
       dec    _XBUS_AUX               ; DPTR0 = *src
       mov    A,B                     ; 0x00 -> XDATA 
       JNZ    2$

1$:                                   ; xmem loop 
       movx   A,@DPTR                 ; read source (xmem) with autoinc
       .db    0xA5                    ; MOVX @DPTR1,A & INC DPTR1
       djnz   R7,1$
       sjmp   3$                      ;done 

2$:                                   ; cmem loop 
       clr    A
       movc   A,@A+DPTR               ; read source (codemem)
       inc    DPTR                    ; src ++ no autoinc 
       .db    0xA5                    ; MOVX @DPTR1,A & INC DPTR1
       djnz   R7,2$    
3$:    pop    _XBUS_AUX               ; done 
       pop    ar7
       ret
   __endasm;
}
#endif

#ifdef UID_SN
/*
  um: Create a unique serial nummber based on CH552 UID
*/
#define CBYTE ((uint8_t volatile __code  *) 0)
__code uint8_t alpha[16] = {'0','1','2','3','4','5','6','7',
                            '8','9','A','B','C','D','E','F'};
void Init_UsbSerialNo(void)
{
   uint8_t b;
   serno[0] = 22;  serno[1] = 3;

   b = CBYTE[ROM_CHIP_ID_HX];
   serno[2] = alpha [b >> 4];   serno[3]=0;
   serno[4] = alpha [b & 0x0F]; serno[5]=0;

   b = CBYTE[ROM_CHIP_ID_LO];
   serno[6] = alpha [b >> 4];   serno[7]=0;
   serno[8] = alpha [b & 0x0F]; serno[9]=0;

   b = CBYTE[ROM_CHIP_ID_LO+1];
   serno[10] = alpha [b >> 4];   serno[11]=0;
   serno[12] = alpha [b & 0x0F]; serno[13]=0;

   b = CBYTE[ROM_CHIP_ID_LO+2];
   serno[14] = alpha [b >> 4];   serno[15]=0;
   serno[16] = alpha [b & 0x0F]; serno[17]=0;

   b = CBYTE[ROM_CHIP_ID_LO+3];
   serno[18] = alpha [b >> 4];   serno[19]=0;
   serno[20] = alpha [b & 0x0F]; serno[21]=0;
}
#endif                           
/*******************************************************************************
 * Function Name  : DeviceInterrupt()
 * Description  : CH55xUSB interrupt processing function
 *******************************************************************************/
void DeviceInterrupt(void) __interrupt(INT_NO_USB) // USB interrupt service routine, using register bank 1
{
    static const uint8_t* pDescr; // um: USB descriptor pointer
    uint8_t len;                  // um: 8bit is enough
    
    if (UIF_TRANSFER) // USB transfer completion flag
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP)) {
        case UIS_TOKEN_SOF | 0: 
        case UIS_TOKEN_SOF | 1:
        case UIS_TOKEN_SOF | 2:
            sof_count++;
            break;

        case UIS_TOKEN_IN | 1: // endpoint 1
            UEP1_T_LEN = 0;
            UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // Default response NAK
            ep1_in_busy = 0;
            break;
        case UIS_TOKEN_OUT | 2: // endpoint 2
            if (U_TOG_OK) // Out-of-sync packets will be discarded
            {
                USBByteCount = USB_RX_LEN;
                USBBufOutPoint = 0; // Reset data pointer
                UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_NAK; // NAK is sent when a packet of data is received. After the main function completes the processing, the main function modifies the response mode.
            }
            break;
        case UIS_TOKEN_SETUP | 0: // SETUP transaction
            UEP0_CTRL &= 0xF2;    // um: if the previous request was stalled this fixes a second stall
            len = USB_RX_LEN;
            if (len == (sizeof(USB_SETUP_REQ))) {
                uint8_t addr;
                SetupLen = ((uint16_t)UsbSetupBuf->wLengthH << 8) | (UsbSetupBuf->wLengthL);
                len = 0; // The default is success and upload length is 0
                vendor_control = 0; // Default non-vendor
                SetupReq = UsbSetupBuf->bRequest;
                if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) == USB_REQ_TYP_VENDOR) {
                    vendor_control = 1;
                    if (SetupLen == 0) {
                        // No Data
                        switch (SetupReq) {
                        case FTDI_VEN_REQ_RESET:
                            break;
                        case FTDI_VEN_REQ_SET_BAUDRATE:
                            break;
                        case FTDI_VEN_REQ_SET_DATA_CHAR:
                            break;
                        case FTDI_VEN_REQ_SET_FLOW_CTRL:
                            break;
                        case FTDI_VEN_REQ_SET_MODEM_CTRL:
                            break;
                        default:
                            break;
                        }
                    } else {
                        // Data
                        switch (SetupReq) {
                        case FTDI_VEN_REQ_RD_EEPROM:
                            addr = UsbSetupBuf->wIndexL << 1; //((req->wIndex >> 8) & 0x3F) << 1;
                            Ep0Buffer[0] = ftdi_rom[addr];
                            Ep0Buffer[1] = ftdi_rom[addr + 1];
                            len = 2;
                            break;
                        case FTDI_VEN_REQ_GET_MODEM_STA:
                            // return fixed modem status
                            Ep0Buffer[0] = FTDI_MODEM_STA_DUMMY0;
                            Ep0Buffer[1] = FTDI_MODEM_STA_DUMMY1;
                            len = 2;
                            break;
                        case FTDI_VEN_REQ_SET_LAT_TIMER:
                            latency_timer = UsbSetupBuf->wValueL;
                            len = 0;
                            break;
                        default:
                            // return dummy data
                            Ep0Buffer[0] = 0x0;
                            Ep0Buffer[1] = 0x0;
                            len = 2;
                            break;
                        }
                    }
                } else if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD) {
                    switch (SetupReq) // Request Code
                    {
                    case USB_GET_DESCRIPTOR:
                        switch (UsbSetupBuf->wValueH) {
                        case 1: // Device Descriptor
                            pDescr = DevDesc; // Send the device descriptor to the buffer to be sent
                            len = sizeof(DevDesc);
                            break;
                        case 2: // Configuration Descriptor
                            pDescr = CfgDesc; // Send the device descriptor to the buffer to be sent
                            len = sizeof(CfgDesc);
                            break;
                        case 3:
                            if (UsbSetupBuf->wValueL == 0) {
                                pDescr = LangDes;
                                len = sizeof(LangDes);
                            } else if (UsbSetupBuf->wValueL == 1) {
                                pDescr = Manuf_Des;
                                len = sizeof(Manuf_Des);
                            } else if (UsbSetupBuf->wValueL == 2) {
                                pDescr = Prod_Des;
                                len = sizeof(Prod_Des);
                            } else if (UsbSetupBuf->wValueL == 3) {
#ifdef UID_SN
                                pDescr = serno;
                                len    = sizeof(serno);
#else
                                pDescr = SerDes;
                                len = sizeof(SerDes);
#endif
                            }
                            else len = 0xff; // um: reject all strings exept id0..id3 unsupported string
                            break;
                        default:
                            len = 0xff; // Unsupported command or error
                            break;
                        }
                        if (SetupLen > len) {
                            SetupLen = len; // Limit total length
                        }
                        len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen; // The length of this transmission
#ifndef FAST_COPY
                        memcpy(Ep0Buffer, pDescr, len); // Loading Upload Data
#else
                        fastcpy(pDescr,Ep0Buffer, len);
#endif
                        SetupLen -= len;
                        pDescr += len;
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL; // Temporarily save USB device address
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if (SetupLen >= 1) {
                            len = 1;
                        }
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        break;
                    case USB_GET_INTERFACE:
                        break;
                    case USB_CLEAR_FEATURE: // Clear Feature
                        if ((UsbSetupBuf->bRequestType & 0x1F) == USB_REQ_RECIP_DEVICE) /* Clear Device */
                        {
                            if ((((uint16_t)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x01) {
                                if (CfgDesc[7] & 0x20) {
                                    /* Wake up */
                                } else {
                                    len = 0xFF; /* Operation failed */
                                }
                            } else {
                                len = 0xFF; /* Operation failed */
                            }
                        } else if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) // Endpoints
                        {
                            switch (UsbSetupBuf->wIndexL) {
                            case 0x02:
                                UEP2_CTRL = UEP2_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
                                break;
                            case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                break;
                            default:
                                len = 0xFF; // Unsupported endpoint
                                break;
                            }
                            ep1_in_busy = 0;
                        } else {
                            len = 0xFF; // Not that the endpoint does not support
                        }
                        break;
                    case USB_SET_FEATURE: /* Set Feature */
                        if ((UsbSetupBuf->bRequestType & 0x1F) == USB_REQ_RECIP_DEVICE) /* Setting up the device */
                        {
                            if ((((uint16_t)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x01) {
                                if (CfgDesc[7] & 0x20) {
                                    /* Hibernation */
#ifdef DE_PRINTF
                                    printf("suspend\r\n"); // Sleep state
#endif
                                    while (XBUS_AUX & bUART0_TX) {
                                        ; // Waiting for sending to complete
                                    }
                                    SAFE_MOD = 0x55;
                                    SAFE_MOD = 0xAA;
                                    WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO; // Can be woken up when USB or RXD0/1 has a signal
                                    PCON |= PD; // Sleep
                                    SAFE_MOD = 0x55;
                                    SAFE_MOD = 0xAA;
                                    WAKE_CTRL = 0x00;
                                } else {
                                    len = 0xFF; /* Operation failed */
                                }
                            } else {
                                len = 0xFF; /* Operation failed */
                            }
                        } else if ((UsbSetupBuf->bRequestType & 0x1F) == USB_REQ_RECIP_ENDP) /* Setting up the endpoint */
                        {
                            if ((((uint16_t)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x00) {
                                switch (((uint16_t)UsbSetupBuf->wIndexH << 8) | UsbSetupBuf->wIndexL) {
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* Set endpoint 2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* Set endpoint 2 OUT Stall */
                                    break;
                                case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* Set endpoint 1 IN STALL */
                                    break;
                                case 0x01:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* Set endpoint 1 OUT Stall */
                                default:
                                    len = 0xFF; /* Operation failed */
                                    break;
                                }
                            } else {
                                len = 0xFF; /* Operation failed */
                            }
                        } else {
                            len = 0xFF; /* Operation failed */
                        }
                        break;
                    case USB_GET_STATUS:
                        Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        if (SetupLen >= 2) {
                            len = 2;
                        } else {
                            len = SetupLen;
                        }
                        break;
                    default:
                        len = 0xff; // Operation failed
                        break;
                    }
                } else {
                    switch (SetupReq) {

                    default:
                        len = 0xFF; /*Command not supported*/
                        break;
                    }
                }
            } 
            else len = 0xff; // Packet length error
            
            if (len == 0xff) {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; // STALL
            } else if (len <= DEFAULT_ENDP0_SIZE) // Returns a 0-length packet during the data upload or status upload phase
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; // The default data packet is DATA1, and the response ACK is returned
            } else {
                UEP0_T_LEN = 0; // Although it has not yet reached the status stage, a 0-length data packet is uploaded in advance to prevent the host from entering the status stage early.
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; // The default data packet is DATA1, and the response is ACK.
            }
            break;
        case UIS_TOKEN_IN | 0: // endpoint0 IN
            switch (SetupReq) {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen; // The length of this transmission
#ifndef FAST_COPY
                memcpy(Ep0Buffer, pDescr, len); // Loading Upload Data
#else
                fastcpy(pDescr,Ep0Buffer, len);
#endif
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG; // Synchronous flag bit flip
                break;
            case USB_SET_ADDRESS:
                if (!vendor_control) {
                    USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                    UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                }
                break;
            default:
                UEP0_T_LEN = 0; // The status stage is completed and the interrupt is completed or the zero-length data packet is forced to upload to end the control transmission.
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0: // endpoint0 OUT
/*          if(SetupReq ==SET_LINE_CODING)  //Set serial port properties
            {
               if( U_TOG_OK )
               {
                  //  memcpy(LineCoding,UsbSetupBuf,USB_RX_LEN);
                  //  Config_Uart1(LineCoding);
                  UEP0_T_LEN = 0;
                  UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;  // Prepare to upload 0 packages
               }
            }
            else
            {*/
            UEP0_T_LEN = 0;
            UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK; // Status phase, respond to IN with NAK
            //}
            break;

        default:
            break;
        }
        UIF_TRANSFER = 0; // Write 0 to clear the interrupt
    }
    if (UIF_BUS_RST) // Device mode USB bus reset interrupt
    {
#ifdef DE_PRINTF
        printf("reset\r\n"); // Sleep state
#endif
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
        UIF_BUS_RST = 0; // Clear interrupt flag

        USBByteCount = 0; // The length received by the USB endpoint
        UsbConfig = 0; // Clearing Configuration Values
        ep1_in_busy = 0;
    }
    if (UIF_SUSPEND) // USB bus suspend or wake up completed
    {
        UIF_SUSPEND = 0;
        if (USB_MIS_ST & bUMS_SUSPEND) // Suspend
        {
#ifdef DE_PRINTF
            printf("suspend\r\n"); // Sleep state
#endif
            while (XBUS_AUX & bUART0_TX) {
                ; // Waiting for sending to complete
            }
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO; // Can be woken up when USB or RXD0/1 has a signal
            PCON |= PD; // Sleep
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = 0x00;
        }
    } else { // Unexpected interruption, impossible situation
        USB_INT_FG = 0xFF; // Clear interrupt flag
    }
}
// um: locate it in __data
__data uint8_t transmit_buffer_in_offset;
__data uint8_t transmit_buffer_out_offset;
__data uint8_t send_len;

static inline uint8_t shift_data(void)
{

#ifndef HARDWARE_SPI
    TDI = P2B0;
    P2B0 = TDO;
    TCK = 1;
    TCK = 0;

    TDI = P2B1;
    P2B1 = TDO;
    TCK = 1;
    TCK = 0;

    TDI = P2B2;
    P2B2 = TDO;
    TCK = 1;
    TCK = 0;

    TDI = P2B3;
    P2B3 = TDO;
    TCK = 1;
    TCK = 0;

    TDI = P2B4;
    P2B4 = TDO;
    TCK = 1;
    TCK = 0;

    TDI = P2B5;
    P2B5 = TDO;
    TCK = 1;
    TCK = 0;

    TDI = P2B6;
    P2B6 = TDO;
    TCK = 1;
    TCK = 0;

    TDI = P2B7;
    P2B7 = TDO;
    TCK = 1;
    TCK = 0;
    return P2;
#else
    CH554SPIMasterWrite(P2);
    return SPI0_DATA;
#endif
}

static inline uint8_t shift_data_AS(void)
{

    TDI = P2B0;
    P2B0 = ASDO;
    TCK = 1;
    TCK = 0;

    TDI = P2B1;
    P2B1 = ASDO;
    TCK = 1;
    TCK = 0;

    TDI = P2B2;
    P2B2 = ASDO;
    TCK = 1;
    TCK = 0;

    TDI = P2B3;
    P2B3 = ASDO;
    TCK = 1;
    TCK = 0;

    TDI = P2B4;
    P2B4 = ASDO;
    TCK = 1;
    TCK = 0;

    TDI = P2B5;
    P2B5 = ASDO;
    TCK = 1;
    TCK = 0;

    TDI = P2B6;
    P2B6 = ASDO;
    TCK = 1;
    TCK = 0;

    TDI = P2B7;
    P2B7 = ASDO;
    TCK = 1;
    TCK = 0;

    return P2;
}

void main(void)
{
    uint8_t length = 0;
    uint8_t read_buffer_index = 0;
    uint8_t shift_count = 0;
    uint8_t operand = 0;
    uint8_t shift_en = 0;
    uint8_t read_en = 0;
    uint16_t timeout_count = 0;

    //check for a connection between pin9 and pin 10
    if (TDI==0) //um: goto bootloader if TDI is shortened to gnd
    {
       __asm
         LJMP BOOT_LOAD_ADDR
       __endasm;
    } 
    CfgFsys(); // CH552 clock selection configuration
    mDelaymS(5); // wait for clock become stabilize.

#ifdef UID_SN
    Init_UsbSerialNo();
#endif

#ifdef HARDWARE_SPI
    SPIMasterModeSet(0);
    SPI_CK_SET(4);
#endif
    USBDeviceCfg();
    USBDeviceEndPointCfg(); // Endpoint configuration
    USBDeviceIntCfg(); // Interrupt initialization

#ifdef FTDI_AS_MODE

    // P1.1, 1.5, 1.7 P1.4  output push-pull,P1.6 input
    P1_MOD_OC &= ~((1 << 1) | (1 << 5) | (1 << 7) | (1 << 4));
    P1_MOD_OC |= ((1 << 6));
    P1_DIR_PU |= ((1 << 1) | (1 << 5) | (1 << 7) | (1 << 6) | (1 << 4));
    // P3.2 P3.4 output push-pull,P3.3 INPUT
    P3_MOD_OC &= ~((1 << 2) | (1 << 4));
    P3_MOD_OC |= (1 << 3);
    P3_DIR_PU |= ((1 << 2) | (1 << 4) | (1 << 3));

#else

    // P1.1, 1.5, 1.7 output push-pull, P1.6 input
    P1_MOD_OC &= ~((1 << 1) | (1 << 5) | (1 << 7));
    P1_MOD_OC |= (1 << 6);
    P1_DIR_PU |= ((1 << 1) | (1 << 5) | (1 << 7) | (1 << 6));
    // P3.2 output push-pull
    P3_MOD_OC &= ~(1 << 2);
    P3_DIR_PU |= (1 << 2);

#endif

    TDO = 1;

#ifdef FTDI_AS_MODE
    ASDO = 1;
#endif

    UEP0_T_LEN = 0;
    UEP1_T_LEN = 0; // The pre-used sending length must be cleared

    Ep1Buffer[0] = FTDI_MODEM_STA_DUMMY0;
    Ep1Buffer[1] = FTDI_MODEM_STA_DUMMY1;

    transmit_buffer_in_offset = 0;
    transmit_buffer_out_offset = 0;

    length = 0;
    send_dummy = 1;
    LED = 1;

    while (1) {
        if (UsbConfig) {
            length = 0;
            if (USBByteCount) // The USB receiving endpoint has data
            {
                // memcpy(receive_buffer, Ep2Buffer, USBByteCount);
#ifdef FAST_COPY                
                EA=0;
                fastcpy(Ep2Buffer, receive_buffer, USBByteCount);
                EA=1;
#else
               /* clang-format off */
                __asm
                  push ar7
                  push a
                  push _XBUS_AUX        // store XBUS too 
                  orl  _XBUS_AUX, #DPS  // only set the DPS bit for DPTR1
                  //inc _XBUS_AUX //dptr1
                  mov dptr, #_receive_buffer  //target receive_buffer
                  dec _XBUS_AUX //dptr0
                  mov dptr, #_Ep2Buffer //source Ep2Buffer
                  mov ar7, _USBByteCount
                1$: 
                  movx a, @dptr
                  inc dptr
                  .db #0xA5 //WCH 0xA5 instruction
                  djnz ar7, 1$
                  pop _XBUS_AUX
                  pop a
                  pop ar7
                __endasm;

                /* clang-format on */
#endif
                UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_ACK;
                length = USBByteCount;
                USBByteCount = 0;
            }

            read_buffer_index = 0;
            while (read_buffer_index < length) {
                P2 = receive_buffer[read_buffer_index];
                read_buffer_index++;
                // TODO: Assembly implementation for IO control.
                if (shift_count == 0) {
#ifdef HARDWARE_SPI
                    SPI0_CTRL = 0x00;
#endif
                    shift_en = P2B7;
                    read_en = P2B6;
                    if (shift_en) {
                        shift_count = P2 & 0x3f;
#ifdef HARDWARE_SPI
#ifdef FTDI_AS_MODE
                        if (!((!NCS) && (read_en)))
                            SPI0_CTRL = 0x60;
#else
                        SPI0_CTRL = 0x60;
#endif
#endif
                    } else if (read_en) {

                        LED = !P2B5;
                        TDI = P2B4;
                        TMS = P2B1;
                        TCK = P2B0;

#ifdef FTDI_AS_MODE
                        NCE = P2B2;
                        NCS = P2B3;
#endif

#ifdef FTDI_AS_MODE
                        transmit_buffer[transmit_buffer_in_offset] = TDO;
                        transmit_buffer[transmit_buffer_in_offset] |= (ASDO << 1);
                        transmit_buffer_in_offset++;
                        transmit_buffer_in_offset &= 0x7f; // %= sizeof(transmit_buffer);
#else

                        transmit_buffer[transmit_buffer_in_offset] = TDO;
                        transmit_buffer_in_offset++;
                        transmit_buffer_in_offset &= 0x7f; // %= sizeof(transmit_buffer);

#endif

                    } else {
                        LED = !P2B5;
                        TDI = P2B4;
                        TMS = P2B1;
                        TCK = P2B0;

#ifdef FTDI_AS_MODE
                        NCE = P2B2;
                        NCS = P2B3;
#endif
                    }

                } else {
                    shift_count--;
                    if (read_en) {

#ifdef FTDI_AS_MODE

                        if (!NCS) {
                            transmit_buffer[transmit_buffer_in_offset] = shift_data_AS();
                            transmit_buffer_in_offset++;
                            transmit_buffer_in_offset &= 0x7f;
                        } else {
                            transmit_buffer[transmit_buffer_in_offset] = shift_data();
                            transmit_buffer_in_offset++;
                            transmit_buffer_in_offset &= 0x7f;
                        }

#else
                        transmit_buffer[transmit_buffer_in_offset] = shift_data();
                        transmit_buffer_in_offset++;
                        transmit_buffer_in_offset &= 0x7f;
#endif
                    } else {

                        shift_data();
                    }
                }
            }

            if (ep1_in_busy == 0) // The endpoint is not busy (the first packet of data after being idle is only used to trigger upload)
            {
                int8_t data_len = transmit_buffer_in_offset - transmit_buffer_out_offset;
                data_len = data_len < 0 ? 128 + data_len : data_len;
                if (data_len > 0) // 2 for modem bytes.
                {
                    uint8_t i;
                    send_len = (data_len >= 62) ? 62 : data_len;

                    for (i = 0; i < send_len; i++) {
                        Ep1Buffer[i + 2] = transmit_buffer[transmit_buffer_out_offset];
                        transmit_buffer_out_offset++;
                        transmit_buffer_out_offset &= 0x7f; // %= sizeof(transmit_buffer);
                    }

                    ep1_in_busy = 1;
                    UEP1_T_LEN = send_len + 2;
                    UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; // ACK
                } else if ((sof_count - timeout_count) > latency_timer) {
                    timeout_count = sof_count;
                    ep1_in_busy = 1;
                    UEP1_T_LEN = 2; // The pre-used sending length must be cleared
                    UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; // ACK
                } else if (send_dummy) {
                    send_dummy--;
                    ep1_in_busy = 1;
                    UEP1_T_LEN = 2; // The pre-used sending length must be cleared
                    UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK; // ACK
                }
            }
        }
    }
}