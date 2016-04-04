//*****************************************************************************
//! @file       main.c
//! @brief      Main file for CC2538 HID example. In this example, the
//!             directional keys are mapped as directional keys on the PC.
//!             The SmartRF06EB SELECT key is mapped as Return on the PC.
//!             SmartRF06EB LEDs indicate Num lock, scroll lock and caps lock.
//!
//! Revised     $Date: 2013-03-26 09:47:02 +0100 (Tue, 26 Mar 2013) $
//! Revision    $Revision: 9535 $
//
//  Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/

//*****************************************************************************
//
// Include driver libraries
//
//*****************************************************************************
#include "bsp.h"
#include "ioc.h"
#include "gptimer.h"
// #include "bsp_key.h"
#include "bsp_led.h"
#include "string.h"
#include "usb_hid.h"
#include "usb_firmware_library_headers.h"


//*****************************************************************************
//
// Local functions
//
//*****************************************************************************
void selKeyRemoteWakeupIsr(void) {
    usbsuspDoRemoteWakeup();
    IntDisable(INT_GPIOA);
}


void dirKeyRemoteWakeupIsr(void) {
    usbsuspDoRemoteWakeup();
    IntDisable(INT_GPIOC);
}


//*****************************************************************************
//
// Implementations of function that are required by usb framework.
//
//*****************************************************************************
void usbsuspHookEnteringSuspend(bool remoteWakeupAllowed) {
    if (remoteWakeupAllowed) {
        GPIOPowIntClear(BSP_KEY_SEL_BASE, BSP_KEY_SELECT);
        GPIOPowIntEnable(BSP_KEY_SEL_BASE, BSP_KEY_SELECT);
        IntPendClear(INT_GPIOA);
        IntEnable(INT_GPIOA);

        GPIOPowIntClear(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL);
        GPIOPowIntEnable(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL);
        IntPendClear(INT_GPIOC);
        IntEnable(INT_GPIOC);
    }
}


void usbsuspHookExitingSuspend(void) {
    IntDisable(INT_GPIOA);
    GPIOPowIntDisable(BSP_KEY_SEL_BASE, BSP_KEY_SELECT);

    IntDisable(INT_GPIOC);
    GPIOPowIntDisable(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL);
}

uint8_t send = 0;
uint8_t count = 0;
// void buttonPress(void) {
//      bspLedToggle(BSP_LED_1);
//     send = 1;
// }




void
GPIOBIntHandler(void)
{
    uint32_t ui32GPIOIntStatus;

    bspLedToggle(BSP_LED_1);

    //
    // Get the masked interrupt status.
    //
    ui32GPIOIntStatus = GPIOPinIntStatus(GPIO_B_BASE, true);

    //
    // Simple debounce function wait for button de-press
    //
    while(GPIOPinRead(GPIO_B_BASE, GPIO_PIN_6))
    {
    }

    //
    // Acknowledge the GPIO  - Pin n interrupt by clearing the interrupt flag.
    //
    GPIOPinIntClear(GPIO_B_BASE, ui32GPIOIntStatus);

    //
    // Set an interrupt flag to indicate an interrupt has occurred.
    //
    // send = 1;

    count += 1;


    // Start a timeout timer
    TimerDisable(GPTIMER0_BASE, GPTIMER_A);
    TimerLoadSet(GPTIMER0_BASE, GPTIMER_A, SysCtrlClockGet() * 3);
    TimerEnable(GPTIMER0_BASE, GPTIMER_A);

}


void
Timer0AIntHandler(void)
{
    //
    // Clear the timer interrupt flag.
    //
    TimerIntClear(GPTIMER0_BASE, GPTIMER_TIMA_TIMEOUT);

    //
    // Set a flag to indicate that the interrupt occurred.
    //
   send = 1;
}




void send_char (char c, bool shift) {
    KEYBOARD_IN_REPORT keybReport;
    memset(&keybReport, 0x00, sizeof(KEYBOARD_IN_REPORT));

    if (shift) {
       keybReport.modifiers    = 2;
       keybReport.pKeyCodes[1] = 0xE1;
    }

    if (c >= 97 && c <= 122) {
        keybReport.pKeyCodes[0] = c - 93;
    } else if (c >= 49 && c <= 57) {
        keybReport.pKeyCodes[0] = c - 19;
    } else if (c == '0') {
        keybReport.pKeyCodes[0] = 39;
    } else {
        keybReport.pKeyCodes[0] = 0x38; // "?"
        keybReport.modifiers    = 2;
        keybReport.pKeyCodes[1] = 0xE1;
    }
    hidUpdateKeyboardInReport(&keybReport);

    while (!hidSendKeyboardInReport()) {
        usbHidProcessEvents();
    }

    // Clear button press
    memset(&keybReport, 0x00, sizeof(KEYBOARD_IN_REPORT));
    hidUpdateKeyboardInReport(&keybReport);

    while (!hidSendKeyboardInReport()) {
        usbHidProcessEvents();
    }
}


//
// Application entry point
//
int main(void)
{
    KEYBOARD_IN_REPORT keybReport;
    uint8_t keybReportSendReq = false;
    uint8_t currKey = 0x17;

    //
    // Initialize board and system clock
    //
    bspInit(SYS_CTRL_32MHZ);

    //
    // Enable the USB interface
    //
    usbHidInit();

    //
    // Initialize GPIO pins for keyboard LEDs (LED 1 on PC0 is used by USB to
    // control D+ pull-up)
    //
    GPIOPinTypeGPIOOutput(BSP_LED_BASE, BSP_LED_2 | BSP_LED_3 | BSP_LED_1);

    bspLedSet(BSP_LED_2);
    bspLedSet(BSP_LED_3);
    bspLedSet(BSP_LED_1);


    //
    // Configure interrupt with wakeup for all buttons
    //
    // IntRegister(INT_GPIOB, buttonPress);
    // GPIOPowIntTypeSet(GPIO_B_BASE, GPIO_PIN_6, GPIO_POW_RISING_EDGE);
    // IntRegister(INT_GPIOC, dirKeyRemoteWakeupIsr);
    // GPIOPowIntTypeSet(BSP_KEY_DIR_BASE, BSP_KEY_DIR_ALL, GPIO_POW_RISING_EDGE);

    GPIOPinTypeGPIOInput(GPIO_B_BASE, GPIO_PIN_6);
    IOCPadConfigSet(GPIO_B_BASE, GPIO_PIN_6, IOC_OVERRIDE_PUE);
    GPIOIntTypeSet(GPIO_B_BASE, GPIO_PIN_6, GPIO_RISING_EDGE);
    GPIOPinIntEnable(GPIO_B_BASE, GPIO_PIN_6);

    IntMasterEnable();
    IntEnable(INT_GPIOB);




    // setup timer
    SysCtrlPeripheralEnable(SYS_CTRL_PERIPH_GPT0);
    TimerConfigure(GPTIMER0_BASE, GPTIMER_CFG_ONE_SHOT);
    TimerIntEnable(GPTIMER0_BASE, GPTIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);





    //
    // Initialize button polling for keyboard HID reports
    //
    // memset(&keybReport, 0x00, sizeof(KEYBOARD_IN_REPORT));


    bspLedClear(BSP_LED_2);

    //
    // Main loop
    //
    while (1)
    {

        //
        // Process USB events
        //
        usbHidProcessEvents();

        if (send) {
            send = 0;


            bspLedToggle(BSP_LED_3);



            // keybReport.modifiers    = 2;
            // keybReport.pKeyCodes[0] = 0xE1;
            // keybReport.pKeyCodes[1] = 0x05;  // b
            // hidUpdateKeyboardInReport(&keybReport);
            // hidSendKeyboardInReport();


            send_char('b', true);
            send_char('i', true);
            send_char('l', true);
            send_char('l', true);

            if (count > 0 && count < 3) {
                send_char('0', false);
                send_char('0', false);
                send_char('1', false);
            } else if (count >= 3 && count < 7) {
                send_char('0', false);
                send_char('0', false);
                send_char('5', false);
            } else if (count >= 8 && count < 12) {
                send_char('0', false);
                send_char('1', false);
                send_char('0', false);
            } else if (count >= 17 && count < 22) {
                send_char('0', false);
                send_char('2', false);
                send_char('0', false);
            } else if (count >= 45 && count < 55) {
                send_char('0', false);
                send_char('5', false);
                send_char('0', false);
            } else if (count >= 90 && count < 110) {
                send_char('1', false);
                send_char('0', false);
                send_char('0', false);
            } else {
                send_char('E', false);
                send_char('R', false);
                send_char('R', false);
            }

            count = 0;



        }

        //
        // Generate keyboard input
        //
        // if (!keybReportSendReq)
        // {

        //     bspLedToggle(BSP_LED_1);

        //     // switch (bspKeyPushed(BSP_KEY_ALL))
        //     // {
        //     // case BSP_KEY_LEFT:
        //         currKey += 1;
        //         if (currKey > 0x25) {
        //             currKey = 0x17;
        //         }
        //     //     break;
        //     // case BSP_KEY_RIGHT:
        //     //     currKey = 0x4F;
        //     //     break;
        //     // case BSP_KEY_UP:
        //     //     currKey = 0x52;
        //     //     break;
        //     // case BSP_KEY_DOWN:
        //     //     currKey = 0x51;
        //     //     break;
        //     // case BSP_KEY_SELECT:
        //     //     currKey = 0x28;
        //     //     break;
        //     // default:
        //     //     currKey = 0x00;
        //     //     break;
        //     // }
        //     if (currKey != keybReport.pKeyCodes[0])
        //     {
        //         keybReport.pKeyCodes[0] = currKey;
        //         hidUpdateKeyboardInReport(&keybReport);
        //         keybReportSendReq = true;
        //     }
        // }
        // if (keybReportSendReq)
        // {
        //     if (hidSendKeyboardInReport())
        //     {
        //         keybReportSendReq = false;
        //     }
        // }
    }

}


//
// Callback function for HID application
//
void usbHidAppPoll(void) {

    //
    // Output keyboard LED status on LEDs 2-4
    //
    // if (hidData.keyboardOutReport.ledStatus & 0x01)
    // {
    //     bspLedSet(BSP_LED_2);
    // }
    // else
    // {
    //     bspLedClear(BSP_LED_2);
    // }
    // if (hidData.keyboardOutReport.ledStatus & 0x02)
    // {
    //     bspLedSet(BSP_LED_3);
    // }
    // else
    // {
    //     bspLedClear(BSP_LED_3);
    // }
    // if (hidData.keyboardOutReport.ledStatus & 0x04)
    // {
    //     bspLedSet(BSP_LED_4);
    // }
    // else
    // {
    //     bspLedClear(BSP_LED_4);
    // }
}
