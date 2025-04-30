#ifndef DESCRIPTORS_H
#define DESCRIPTORS_H

#include <stdint.h>
#include "xil_types.h"
#include "xstatus.h"

// Helper macros
#define LO_UINT16(x) ((uint8_t)((x) & 0xFF))
#define HI_UINT16(x) ((uint8_t)(((x) >> 8) & 0xFF))

// Standard VID/PID values
#define USB_VENDOR_ID    0x1234
#define USB_PRODUCT_ID   0x0001

// 1. Device Descriptor (USB2.0 HS, Vendor-specific class)
extern const uint8_t DeviceDesc[];

// 2. Configuration Descriptor with one interface and one bulk IN endpoint
extern const uint8_t ConfigDesc[];

// 3. String Descriptors
// Language ID descriptor (English - US)
extern const uint8_t StringLangDesc[];

// Manufacturer string descriptor
extern const uint8_t StringManufacturerDesc[];

// Product string descriptor
extern const uint8_t StringProductDesc[];

// Serial number string descriptor
extern const uint8_t StringSerialDesc[];

// Functions to handle USB Chapter 9 descriptor requests
int Ch9SetupDevDescReply(u8 *BufPtr, u32 BufLen);
int Ch9SetupCfgDescReply(u8 *BufPtr, u32 BufLen);
int Ch9SetupStrDescReply(u8 *BufPtr, u32 BufLen, u8 Index);

#endif // DESCRIPTORS_H
