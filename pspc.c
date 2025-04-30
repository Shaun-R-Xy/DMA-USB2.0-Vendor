/******************************************************************************
*
* Copyright https://github.com/Shaun-R-Xy/DMA-USB2.0-Vendor/commits?author=Shaun-R-Xy.  All rights reserved.
*
******************************************************************************/

/***************************** Include Files *********************************/

#include "xparameters.h"        /* XPAR parameters */
#include "xusbps.h"             /* USB controller driver */
#include "xscugic.h"            /* Interrupt controller driver */
#include "xusbps_ch9.h"         /* Generic Chapter 9 handling code */
#include "xil_exception.h"
#include "xpseudo_asm.h"
#include "xreg_cortexa9.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "descriptors.h"        /* USB descriptors */
#include "xusbps_hw.h"


/************************AXI DMA UART Include Files *********************************/
#include "xuartps.h"
#include "xaxidma.h"

/************************** Constant Definitions *****************************/
#define MEMORY_SIZE (64 * 1024)
#ifdef __ICCARM__
#pragma data_alignment = 32
u8 Buffer[MEMORY_SIZE];
#pragma data_alignment = 4
#else
u8 Buffer[MEMORY_SIZE] ALIGNMENT_CACHELINE;
#endif

/**************************AXI DMA Uart Constant Definitions ************************/
#define UART_DEVICE_ID      XPAR_XUARTPS_0_DEVICE_ID
#define UART_BAUD           115200

#define DMA_DEV_ID          XPAR_AXIDMA_0_DEVICE_ID
#define DMA_IRQ_ID          XPAR_FABRIC_AXI_DMA_0_S2MM_INTROUT_INTR

// Buffer size and data definitions
#define DATA_BUFFER_SIZE    4096            // Size of each data buffer
#define USB_PACKET_SIZE     512             // USB high-speed bulk packet size
#define ADC_SAMPLE_SIZE     4               // 32-bit (4 byte) samples

// Boolean definitions
#define MY_TRUE             1
#define MY_FALSE            0
typedef int MY_BOOL;

/**************************** Type Definitions *******************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/

static int UsbIntrExample(XScuGic *IntcInstancePtr, XUsbPs *UsbInstancePtr,
                          u16 UsbDeviceId, u16 UsbIntrId);

static void UsbIntrHandler(void *CallBackRef, u32 Mask);
static void XUsbPs_Ep0EventHandler(void *CallBackRef, u8 EpNum,
                    u8 EventType, void *Data);
static void XUsbPs_Ep1EventHandler(void *CallBackRef, u8 EpNum,
                    u8 EventType, void *Data);
static int UsbSetupIntrSystem(XScuGic *IntcInstancePtr,
                      XUsbPs *UsbInstancePtr, u16 UsbIntrId);
static void UsbDisableIntrSystem(XScuGic *IntcInstancePtr, u16 UsbIntrId);

/**************************AXI DMA UART Function Prototypes ******************************/

static int  UartInit(void);
static void DmaIntrHandler(void *Callback);
static void SendBufferToUsb(int bufferIndex);
static void ProcessBuffer(int bufferIndex);
static void FormatAdcData(u8 *buffer, u32 size);

/************************** Variable Definitions *****************************/

static XScuGic IntcInstance;    /* The instance of the IRQ Controller */
static XUsbPs UsbInstance;      /* The instance of the USB Controller */

static volatile int NumIrqs = 0;
static volatile int NumReceivedFrames = 0;

/**************************AXI DMA UART Variable Definitions *****************************/
static XUartPs  Uart_Ps;
static XAxiDma  AxiDma;

// Double-buffering system variables
static u8 DataBuffers[2][DATA_BUFFER_SIZE] ALIGNMENT_CACHELINE;
static volatile int CurrentBuffer = 0;
static volatile MY_BOOL BufferReady[2] = {MY_FALSE, MY_FALSE};
static volatile MY_BOOL UsbTransmissionReady = MY_TRUE;

/*****************************************************************************/
/**
*
* Process a completed ADC data buffer before sending it to USB
*
*
******************************************************************************/
static void ProcessBuffer(int bufferIndex)
{
    // Format ADC data
    FormatAdcData(DataBuffers[bufferIndex], DATA_BUFFER_SIZE);

    // Ensure data is visible
    Xil_DCacheFlushRange((UINTPTR)DataBuffers[bufferIndex], DATA_BUFFER_SIZE);
}

/*****************************************************************************/
/**
*
* Format the ADC data
*
******************************************************************************/
static void FormatAdcData(u8 *buffer, u32 size)
{

    NumReceivedFrames++;

    // Print status
    if ((NumReceivedFrames % 1000) == 0) {
        xil_printf("Processed %d data frames\r\n", NumReceivedFrames);
    }
}

/*****************************************************************************/
/**
*
* Send a buffer of data to the USB host
*
******************************************************************************/
static void SendBufferToUsb(int bufferIndex)
{
    XUsbPs *UsbInstancePtr = &UsbInstance;
    int Status;

    // Ensure not already in the middle of a transfer
    if (!UsbTransmissionReady) {
        return;
    }

    // Send buffer through bulk IN endpoint
    Status = XUsbPs_EpBufferSend(UsbInstancePtr, 1,
                              DataBuffers[bufferIndex],
                              DATA_BUFFER_SIZE);

    if (Status != XST_SUCCESS) {
        xil_printf("Error sending buffer to USB: %d\r\n", Status);
        return;
    }

    // Mark USB as busy with transmission
    UsbTransmissionReady = MY_FALSE;
};


/*****************************************************************************/
/**
 *
 * Main function to call the USB interrupt example.
 *
 *
 ******************************************************************************/

int main(void)
{
    int Status;

    // Initialize UART for debug output
    if ((Status = UartInit()) != XST_SUCCESS) {
        return XST_FAILURE;
    }
    xil_printf("\r\n--- USB ADC Data Acquisition System Initializing ---\r\n");

    // Initialize DMA
    XAxiDma_Config *CfgPtr;
    CfgPtr = XAxiDma_LookupConfig(DMA_DEV_ID);
    if (!CfgPtr) {
        xil_printf("Error looking up DMA config\r\n");
        return XST_FAILURE;
    }

    Status = XAxiDma_CfgInitialize(&AxiDma, CfgPtr);
    if (Status != XST_SUCCESS) {
        xil_printf("Error initializing DMA\r\n");
        return XST_FAILURE;
    }

    // Check for scatter gather mode
    if (XAxiDma_HasSg(&AxiDma)) {
        xil_printf("Error: Device configured as SG mode\r\n");
        return XST_FAILURE;
    }

    // Configure DMA interrupts
    XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);
    XAxiDma_IntrEnable(&AxiDma, XAXIDMA_IRQ_IOC_MASK, XAXIDMA_DEVICE_TO_DMA);

    // Initialize data buffers
    memset(DataBuffers[0], 0, DATA_BUFFER_SIZE);
    memset(DataBuffers[1], 0, DATA_BUFFER_SIZE);

    // Run the USB Interrupt example
    Status = UsbIntrExample(&IntcInstance, &UsbInstance,
                XPAR_XUSBPS_0_DEVICE_ID, XPAR_XUSBPS_0_INTR);
    if (Status != XST_SUCCESS) {
        xil_printf("USB Example Failed\r\n");
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

/*****************************************************************************/
/**
 *
 * @return
 *         - XST_SUCCESS if successful
 *         - XST_FAILURE on error
 *
 ******************************************************************************/
static int UsbIntrExample(XScuGic *IntcInstancePtr, XUsbPs *UsbInstancePtr,
                    u16 UsbDeviceId, u16 UsbIntrId)
{
    int    Status;
    u8    *MemPtr = NULL;
    int    ReturnStatus = XST_FAILURE;

    const u8 NumEndpoints = 2;

    XUsbPs_Config        *UsbConfigPtr;
    XUsbPs_DeviceConfig    DeviceConfig;

    /* Initialize the USB driver
     */
    UsbConfigPtr = XUsbPs_LookupConfig(UsbDeviceId);
    if (NULL == UsbConfigPtr) {
        xil_printf("Error looking up USB configuration\r\n");
        goto out;
    }


    Status = XUsbPs_CfgInitialize(UsbInstancePtr,
                       UsbConfigPtr,
                       UsbConfigPtr->BaseAddress);
    if (XST_SUCCESS != Status) {
        xil_printf("Error initializing USB core\r\n");
        goto out;
    }

    /* Set up the interrupt subsystem.
     */
    Status = UsbSetupIntrSystem(IntcInstancePtr,
                    UsbInstancePtr,
                    UsbIntrId);
    if (XST_SUCCESS != Status) {
        xil_printf("Error setting up interrupt system\r\n");
        goto out;
    }

    /* Check if cable is connected
     */
    if (XUsbPs_ReadReg(UsbInstancePtr->Config.BaseAddress, XUSBPS_PORTSCR1_OFFSET) &
        XUSBPS_PORTSCR_CCS_MASK) {
        xil_printf("Cable already connected, resetting the core...\r\n");

        // Reset the core
        XUsbPs_Reset(UsbInstancePtr);
    }

    /* Set up the Device Config structure and configure the core.
     */
    memset(&DeviceConfig, 0, sizeof(XUsbPs_DeviceConfig));

    DeviceConfig.EpCfg[0].Out.Type        = XUSBPS_EP_TYPE_CONTROL;
    DeviceConfig.EpCfg[0].Out.NumBufs    = 2;
    DeviceConfig.EpCfg[0].Out.BufSize    = 64;
    DeviceConfig.EpCfg[0].Out.MaxPacketSize    = 64;
    DeviceConfig.EpCfg[0].In.Type        = XUSBPS_EP_TYPE_CONTROL;
    DeviceConfig.EpCfg[0].In.NumBufs    = 2;
    DeviceConfig.EpCfg[0].In.MaxPacketSize    = 64;

    // Configure EP1
    DeviceConfig.EpCfg[1].In.Type        = XUSBPS_EP_TYPE_BULK;
    DeviceConfig.EpCfg[1].In.NumBufs    = 8;  // More buffers for efficient continuous transfers
    DeviceConfig.EpCfg[1].In.MaxPacketSize    = 512;  // Max high-speed bulk packet size

    DeviceConfig.NumEndpoints = NumEndpoints;

    // Use Buffer for USB configuration
    MemPtr = (u8 *)&Buffer[0];
    memset(MemPtr, 0, MEMORY_SIZE);
    Xil_DCacheFlushRange((unsigned int)MemPtr, MEMORY_SIZE);

    /* Finish the configuration of the DeviceConfig structure and configure
     */
    DeviceConfig.DMAMemPhys = (u32) MemPtr;

    Status = XUsbPs_ConfigureDevice(UsbInstancePtr, &DeviceConfig);
    if (XST_SUCCESS != Status) {
        xil_printf("Error configuring USB device\r\n");
        goto out;
    }

    extern int XUsbPs_VendorHandleSetupPacket(XUsbPs *InstancePtr, XUsbPs_SetupData *SetupData);

    /* Set the handler */
    Status = XUsbPs_IntrSetHandler(UsbInstancePtr, UsbIntrHandler, UsbInstancePtr,
                        XUSBPS_IXR_UE_MASK);
    if (XST_SUCCESS != Status) {
        xil_printf("Error setting USB interrupt handler\r\n");
        goto out;
    }

    /* Set the handler
     */
    Status = XUsbPs_EpSetHandler(UsbInstancePtr, 0,
                XUSBPS_EP_DIRECTION_OUT,
                XUsbPs_Ep0EventHandler, UsbInstancePtr);
    if (Status != XST_SUCCESS) {
        xil_printf("Error setting EP0 handler\r\n");
        goto out;
    }

    /* Set the handler
     */
    Status = XUsbPs_EpSetHandler(UsbInstancePtr, 1,
                XUSBPS_EP_DIRECTION_IN,
                XUsbPs_Ep1EventHandler, UsbInstancePtr);
    if (Status != XST_SUCCESS) {
        xil_printf("Error setting EP1 handler\r\n");
        goto out;
    }

    /* Enable the interrupts. */
    XUsbPs_IntrEnable(UsbInstancePtr, XUSBPS_IXR_UR_MASK |
                       XUSBPS_IXR_UI_MASK);

    /* Start the USB engine */
    XUsbPs_Start(UsbInstancePtr);


    xil_printf("USB device started and ready for enumeration\r\n");

    // Initialize buffer system
    CurrentBuffer = 0;
    BufferReady[0] = MY_FALSE;
    BufferReady[1] = MY_FALSE;
    UsbTransmissionReady = MY_TRUE;

    // Start first DMA transfer
    Status = XAxiDma_SimpleTransfer(&AxiDma,
                                   (u32)DataBuffers[CurrentBuffer],
                                   DATA_BUFFER_SIZE,
                                   XAXIDMA_DEVICE_TO_DMA);
    if (Status != XST_SUCCESS) {
        xil_printf("Error starting DMA transfer\r\n");
        goto out;
    }

    xil_printf("DMA transfer started. System operational.\r\n");

    /* Enter main processing loop */
    while (1) {
        if (BufferReady[0] && UsbTransmissionReady) {
            SendBufferToUsb(0);
        } else if (BufferReady[1] && UsbTransmissionReady) {
            SendBufferToUsb(1);
        }
    }

    ReturnStatus = XST_SUCCESS;

out:
    /* Clean up.
     */
    XUsbPs_Stop(UsbInstancePtr);
    XUsbPs_IntrDisable(UsbInstancePtr, XUSBPS_IXR_ALL);
    (void) XUsbPs_IntrSetHandler(UsbInstancePtr, NULL, NULL, 0);

    UsbDisableIntrSystem(IntcInstancePtr, UsbIntrId);

    return ReturnStatus;
}

/*****************************************************************************/
/**
 *
 * @return   None
 *
 ******************************************************************************/
static void UsbIntrHandler(void *CallBackRef, u32 Mask)
{
    XUsbPs *UsbInstancePtr = (XUsbPs *)CallBackRef;

    // Increment interrupt counter
    NumIrqs++;

    // Check for USB reset condition
    if (Mask & XUSBPS_IXR_UR_MASK) {
        xil_printf("USB Reset Detected\r\n");

        // Reset buffer system on USB reset
        BufferReady[0] = MY_FALSE;
        BufferReady[1] = MY_FALSE;
        UsbTransmissionReady = MY_TRUE;
    }

    // Check for USB disconnect
    if (Mask & XUSBPS_IXR_UI_MASK) {
        u32 usbStatus = XUsbPs_ReadReg(UsbInstancePtr->Config.BaseAddress, XUSBPS_PORTSCR1_OFFSET);

        if (!(usbStatus & XUSBPS_PORTSCR_CCS_MASK)) {
            xil_printf("USB Disconnect Detected\r\n");

            // Reset buffer system on disconnect
            BufferReady[0] = MY_FALSE;
            BufferReady[1] = MY_FALSE;
            UsbTransmissionReady = MY_FALSE;
        }
    }
}

/*****************************************************************************/
/**
*
* @return    None.
*
******************************************************************************/
static void XUsbPs_Ep0EventHandler(void *CallBackRef, u8 EpNum,
                u8 EventType, void *Data)
{
    XUsbPs            *InstancePtr;
    int                Status;
    XUsbPs_SetupData    SetupData;
    u8                *BufferPtr;
    u32                BufferLen;
    u32                Handle;
    int                ReplyLen;
    u8                Reply[XUSBPS_REQ_REPLY_LEN];

    Xil_AssertVoid(NULL != CallBackRef);

    InstancePtr = (XUsbPs *) CallBackRef;

    switch (EventType) {

    /* Handle */
    case XUSBPS_EP_EVENT_SETUP_DATA_RECEIVED:
        Status = XUsbPs_EpGetSetupData(InstancePtr, EpNum, &SetupData);
        if (XST_SUCCESS == Status) {

            if ((SetupData.bmRequestType & XUSBPS_REQ_TYPE_MASK) == XUSBPS_CMD_STDREQ &&
                SetupData.bRequest == XUSBPS_REQ_GET_DESCRIPTOR) {


                switch ((SetupData.wValue >> 8) & 0xff) {
                case XUSBPS_TYPE_DEVICE_DESC:

                    Status = Ch9SetupDevDescReply(Reply, XUSBPS_REQ_REPLY_LEN);
                    if (Status > 0) {
                        ReplyLen = Status > SetupData.wLength ? SetupData.wLength : Status;
                        Status = XUsbPs_EpBufferSend(InstancePtr, 0, Reply, ReplyLen);
                    }
                    break;

                case XUSBPS_TYPE_CONFIG_DESC:

                    Status = Ch9SetupCfgDescReply(Reply, XUSBPS_REQ_REPLY_LEN);
                    if (Status > 0) {
                        ReplyLen = Status > SetupData.wLength ? SetupData.wLength : Status;
                        Status = XUsbPs_EpBufferSend(InstancePtr, 0, Reply, ReplyLen);
                    }
                    break;

                case XUSBPS_TYPE_STRING_DESC:

                    Status = Ch9SetupStrDescReply(Reply, XUSBPS_REQ_REPLY_LEN,
                                                SetupData.wValue & 0xFF);
                    if (Status > 0) {
                        ReplyLen = Status > SetupData.wLength ? SetupData.wLength : Status;
                        Status = XUsbPs_EpBufferSend(InstancePtr, 0, Reply, ReplyLen);
                    }
                    break;

                case XUSBPS_TYPE_DEVICE_QUALIFIER:

                    Status = Ch9SetupDevDescReply(Reply, XUSBPS_REQ_REPLY_LEN);
                    if (Status > 0) {

                        Reply[0] = (u8)Status;          // Length
                        Reply[1] = (u8)0x6;             // Device qualifier descriptor
                        Reply[2] = (u8)0x0;             // USB 2.0 (low)
                        Reply[3] = (u8)0x2;             // USB 2.0 (high)
                        Reply[4] = (u8)0xFF;            // Vendor-specific device class
                        Reply[5] = (u8)0x00;            // Subclass
                        Reply[6] = (u8)0x0;             // Protocol
                        Reply[7] = (u8)0x40;            // Max packet size for EP0
                        Reply[8] = (u8)0x1;             // Number of configurations
                        Reply[9] = (u8)0x0;             // Reserved

                        ReplyLen = Status > SetupData.wLength ? SetupData.wLength : Status;
                        Status = XUsbPs_EpBufferSend(InstancePtr, 0, Reply, ReplyLen);
                    }
                    break;

                default:

                    Status = XUsbPs_Ch9HandleSetupPacket(InstancePtr, &SetupData);
                    break;
                }
            } else {

                Status = XUsbPs_Ch9HandleSetupPacket(InstancePtr, &SetupData);
            }

            if (Status != XST_SUCCESS) {
                xil_printf("Error handling setup packet\r\n");
            }
        }
        break;

    case XUSBPS_EP_EVENT_DATA_RX:
        /* Get the data buffer. */
        Status = XUsbPs_EpBufferReceive(InstancePtr, EpNum,
                    &BufferPtr, &BufferLen, &Handle);
        if (XST_SUCCESS == Status) {
            /* Return the buffer. */
            XUsbPs_EpBufferRelease(Handle);
        }
        break;

    default:
        /* Unhandled event */
        break;
    }
}


/*****************************************************************************/
/**
* @return    None.
*
******************************************************************************/
static void XUsbPs_Ep1EventHandler(void *CallBackRef, u8 EpNum,
                    u8 EventType, void *Data)
{
    static int CompletedBuffer = 0;

    if (EventType == XUSBPS_EP_EVENT_DATA_TX) {
        // Free the buffer handle
        XUsbPs_EpBufferRelease((u32)Data);

        // Mark USB as ready
        UsbTransmissionReady = MY_TRUE;

        // Mark buffer as available
        BufferReady[CompletedBuffer] = MY_FALSE;

        int nextBuffer = (CompletedBuffer + 1) % 2;
        if (BufferReady[nextBuffer]) {
            SendBufferToUsb(nextBuffer);
            CompletedBuffer = nextBuffer;
        }
    }
}

/*****************************************************************************/
/**
*
* @return
*         - XST_SUCCESS if successful
*         - XST_FAILURE on error
*
******************************************************************************/
static int UsbSetupIntrSystem(XScuGic *IntcInstancePtr,
                  XUsbPs *UsbInstancePtr, u16 UsbIntrId)
{
    int Status;
    XScuGic_Config *IntcConfig;

    /*
     * Initialize the interrupt controller driver
     */
    IntcConfig = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
    if (NULL == IntcConfig) {
        return XST_FAILURE;
    }
    Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
                    IntcConfig->CpuBaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    Xil_ExceptionInit();

    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
                    (Xil_ExceptionHandler)XScuGic_InterruptHandler,
                    IntcInstancePtr);

    Status = XScuGic_Connect(IntcInstancePtr, UsbIntrId,
                (Xil_ExceptionHandler)XUsbPs_IntrHandler,
                (void *)UsbInstancePtr);
    if (Status != XST_SUCCESS) {
        return Status;
    }
    /*
     * Enable the interrupt
     */
    XScuGic_Enable(IntcInstancePtr, UsbIntrId);

    /*
     * Enable interrupts
     */
    Xil_ExceptionEnableMask(XIL_EXCEPTION_IRQ);

    // Connect DMA interrupt handler
    Status = XScuGic_Connect(IntcInstancePtr,
                             DMA_IRQ_ID,
                             (Xil_ExceptionHandler)DmaIntrHandler,
                             &AxiDma);
    if (Status != XST_SUCCESS) {
        xil_printf("Error connecting DMA interrupt\r\n");
        return XST_FAILURE;
    }
    XScuGic_Enable(IntcInstancePtr, DMA_IRQ_ID);

    return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* @return   None.
*
******************************************************************************/
static void UsbDisableIntrSystem(XScuGic *IntcInstancePtr, u16 UsbIntrId)
{
    /* Disconnect and disable the interrupt */
    XScuGic_Disable(IntcInstancePtr, UsbIntrId);
    XScuGic_Disconnect(IntcInstancePtr, UsbIntrId);

    // Disconnect and disable DMA interrupt
    XScuGic_Disable(IntcInstancePtr, DMA_IRQ_ID);
    XScuGic_Disconnect(IntcInstancePtr, DMA_IRQ_ID);
}

/*****************************************************************************/
/**
*
* Initialize the UART for debug output.
*
******************************************************************************/
static int UartInit(void)
{
    int Status;
    XUartPs_Config *Config;

    // Look up the UART configuration
    Config = XUartPs_LookupConfig(UART_DEVICE_ID);
    if (NULL == Config) {
        return XST_FAILURE;
    }

    // Initialize the UART driver
    Status = XUartPs_CfgInitialize(&Uart_Ps, Config, Config->BaseAddress);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    // Set UART in normal mode
    XUartPs_SetOperMode(&Uart_Ps, XUARTPS_OPER_MODE_NORMAL);

    // Set the baud rate
    Status = XUartPs_SetBaudRate(&Uart_Ps, UART_BAUD);
    if (Status != XST_SUCCESS) {
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

/*****************************************************************************/
/**
*
* Handler for DMA interrupts
*
******************************************************************************/
static void DmaIntrHandler(void *Callback)
{
    XAxiDma *AxiDmaInst = (XAxiDma *)Callback;
    u32 IrqStatus;

    // Read pending interrupts
    IrqStatus = XAxiDma_IntrGetIrq(AxiDmaInst, XAXIDMA_DEVICE_TO_DMA);

    // Acknowledge pending interrupts
    XAxiDma_IntrAckIrq(AxiDmaInst, IrqStatus, XAXIDMA_DEVICE_TO_DMA);

    // If no interrupt is pending, exit
    if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {
        return;
    }

    // Check for completion interrupt
    if (IrqStatus & XAXIDMA_IRQ_IOC_MASK) {
        // Process the completed buffer
        ProcessBuffer(CurrentBuffer);

        // Mark current buffer as ready for USB transmission
        BufferReady[CurrentBuffer] = MY_TRUE;

        // Switch to the other buffer
        int nextBuffer = (CurrentBuffer + 1) % 2;
        CurrentBuffer = nextBuffer;

        if (UsbTransmissionReady && BufferReady[(CurrentBuffer + 1) % 2]) {
            SendBufferToUsb((CurrentBuffer + 1) % 2);
        }

        // Start the next DMA transfer if the next buffer is free
        if (!BufferReady[CurrentBuffer]) {
            int Status = XAxiDma_SimpleTransfer(AxiDmaInst,
                                             (u32)DataBuffers[CurrentBuffer],
                                             DATA_BUFFER_SIZE,
                                             XAXIDMA_DEVICE_TO_DMA);
            if (Status != XST_SUCCESS) {
                xil_printf("Error starting next DMA transfer\r\n");
            }
        }
    }

    // Check for error
    if (IrqStatus & XAXIDMA_IRQ_ERROR_MASK) {
        xil_printf("DMA Error IRQ\r\n");

        // Reset the DMA engine
        XAxiDma_Reset(AxiDmaInst);

        // Wait for reset to complete
        int TimeOut = 1000000;
        while (TimeOut) {
            if (XAxiDma_ResetIsDone(AxiDmaInst)) {
                break;
            }
            TimeOut--;
        }

        if (!TimeOut) {
            xil_printf("DMA reset timeout\r\n");
        } else {
            // Restart DMA transfer
            int Status = XAxiDma_SimpleTransfer(AxiDmaInst,
                                             (u32)DataBuffers[CurrentBuffer],
                                             DATA_BUFFER_SIZE,
                                             XAXIDMA_DEVICE_TO_DMA);
            if (Status != XST_SUCCESS) {
                xil_printf("Error restarting DMA transfer after error\r\n");
            }
        }
    }
}
