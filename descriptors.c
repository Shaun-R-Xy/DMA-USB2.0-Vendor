/******************************************************************************
*
* Copyright https://github.com/Shaun-R-Xy/DMA-USB2.0-Vendor/commits?author=Shaun-R-Xy.  All rights reserved.
*
******************************************************************************/

/***************************** Include Files *********************************/
#include "descriptors.h"
#include <string.h>
#include "xil_types.h"
#include "xstatus.h"

/************************** Constant Definitions *****************************/

// 1. Device Descriptor (USB2.0 HS, Vendor-specific class)
const uint8_t DeviceDesc[] = {
    0x12,        /* bLength                */
    0x01,        /* bDescriptorType DEVICE */
    0x00,0x02,   /* bcdUSB = 2.00 (High-Speed) */
    0xFF,        /* bDeviceClass = Vendor-specific */
    0x00,        /* bDeviceSubClass        */
    0x00,        /* bDeviceProtocol        */
    0x40,        /* bMaxPacketSize0 = 64   */
    LO_UINT16(USB_VENDOR_ID), HI_UINT16(USB_VENDOR_ID),  /* idVendor */
    LO_UINT16(USB_PRODUCT_ID), HI_UINT16(USB_PRODUCT_ID), /* idProduct */
    0x00,0x01,   /* bcdDevice = 1.00       */
    0x01,        /* iManufacturer          */
    0x02,        /* iProduct               */
    0x03,        /* iSerialNumber          */
    0x01         /* bNumConfigurations     */
};

// 2. Configuration Descriptor with one interface and one bulk IN endpoint
const uint8_t ConfigDesc[] = {
    /* Config */
    0x09,        /* bLength */
    0x02,        /* bDescriptorType CONFIG */
    0x20,0x00,   /* wTotalLength = 32 bytes */
    0x01,        /* bNumInterfaces */
    0x01,        /* bConfigurationValue */
    0x00,        /* iConfiguration */
    0x80,        /* bmAttributes: Bus-Powered */
    0xFA,        /* bMaxPower = 500mA */

    /* Interface */
    0x09,        /* bLength */
    0x04,        /* bDescriptorType INTERFACE */
    0x00,        /* bInterfaceNumber = 0 */
    0x00,        /* bAlternateSetting */
    0x01,        /* bNumEndpoints = 1 */
    0xFF,        /* bInterfaceClass = Vendor-specific */
    0x00,        /* bInterfaceSubClass */
    0x00,        /* bInterfaceProtocol */
    0x00,        /* iInterface */

    /* Endpoint IN */
    0x07,        /* bLength */
    0x05,        /* bDescriptorType ENDPOINT */
    0x81,        /* bEndpointAddress = 0x81 (IN, EP1) */
    0x02,        /* bmAttributes = Bulk */
    0x00,0x02,   /* wMaxPacketSize = 512 (high-speed bulk max) */
    0x00         /* bInterval (ignored for Bulk) */
};

// 3. String Descriptors
// Language ID descriptor (English - US)
const uint8_t StringLangDesc[] = {
    0x04,        /* bLength */
    0x03,        /* bDescriptorType STRING */
    0x09,0x04    /* wLANGID (0x0409 = English-US) */
};

// Manufacturer string descriptor
const uint8_t StringManufacturerDesc[] = {
    0x22,        /* bLength (34 bytes) */
    0x03,        /* bDescriptorType STRING */
    'X',0, 'i',0, 'n',0, 'y',0, 'i',0, ' ',0,
    'R',0, 'e',0, 'n',0, ' ',0,
    'L',0, 'a',0, 'b',0, 's',0
};

// Product string descriptor
const uint8_t StringProductDesc[] = {
    0x28,        /* bLength (40 bytes) */
    0x03,        /* bDescriptorType STRING */
    'A',0, 'D',0, 'C',0, ' ',0,
    'D',0, 'a',0, 't',0, 'a',0, ' ',0,
    'A',0, 'c',0, 'q',0, 'u',0, 'i',0, 's',0, 'i',0, 't',0, 'i',0, 'o',0, 'n',0
};

// Serial number string descriptor
const uint8_t StringSerialDesc[] = {
    0x12,        /* bLength (18 bytes) */
    0x03,        /* bDescriptorType STRING */
    '1',0, '2',0, '3',0, '4',0, '5',0, '6',0, '7',0, '8',0
};

/************************** Function Implementations *************************/

/*****************************************************************************/
/**
* This function responds to a GET_DESCRIPTOR request for the device descriptor.
******************************************************************************/
int Ch9SetupDevDescReply(u8 *BufPtr, u32 BufLen)
{
    /* Check buffer pointer and length */
    if (!BufPtr || BufLen < sizeof(DeviceDesc)) {
        return XST_FAILURE;
    }

    /* Copy the device descriptor into the buffer */
    memcpy(BufPtr, DeviceDesc, sizeof(DeviceDesc));

    return sizeof(DeviceDesc);
}

/*****************************************************************************/
/**
* This function responds to a GET_DESCRIPTOR request for the configuration
*
******************************************************************************/
int Ch9SetupCfgDescReply(u8 *BufPtr, u32 BufLen)
{
    /* Check buffer pointer and length */
    if (!BufPtr || BufLen < sizeof(ConfigDesc)) {
        return XST_FAILURE;
    }

    /* Copy the configuration descriptor into the buffer */
    memcpy(BufPtr, ConfigDesc, sizeof(ConfigDesc));

    return sizeof(ConfigDesc);
}

/*****************************************************************************/
/**
* This function responds to a GET_DESCRIPTOR request for a string descriptor.
*
******************************************************************************/
int Ch9SetupStrDescReply(u8 *BufPtr, u32 BufLen, u8 Index)
{
    /* Select which string descriptor to return */
    switch (Index) {
        case 0:  /* Language ID */
            if (BufLen < sizeof(StringLangDesc)) {
                return XST_FAILURE;
            }
            memcpy(BufPtr, StringLangDesc, sizeof(StringLangDesc));
            return sizeof(StringLangDesc);

        case 1:  /* Manufacturer */
            if (BufLen < sizeof(StringManufacturerDesc)) {
                return XST_FAILURE;
            }
            memcpy(BufPtr, StringManufacturerDesc, sizeof(StringManufacturerDesc));
            return sizeof(StringManufacturerDesc);

        case 2:  /* Product */
            if (BufLen < sizeof(StringProductDesc)) {
                return XST_FAILURE;
            }
            memcpy(BufPtr, StringProductDesc, sizeof(StringProductDesc));
            return sizeof(StringProductDesc);

        case 3:  /* Serial Number */
            if (BufLen < sizeof(StringSerialDesc)) {
                return XST_FAILURE;
            }
            memcpy(BufPtr, StringSerialDesc, sizeof(StringSerialDesc));
            return sizeof(StringSerialDesc);

        default:
            return XST_FAILURE;
    }
}
