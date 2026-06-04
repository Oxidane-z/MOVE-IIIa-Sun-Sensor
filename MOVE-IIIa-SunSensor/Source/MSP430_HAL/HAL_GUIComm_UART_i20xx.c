/* --COPYRIGHT--,BSD
 * Copyright (c) 2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//*****************************************************************************
//        GUI HAL for MSP430i20xx using UART
//
// Driver to send and receive data from GUI using i20xx UART
// Texas Instruments, Inc.
// *****************************************************************************
#include "HAL_Config_Private.h"
#include "HAL.h"

tGUICommRXCharCallback RxByteISRCallback;

void HAL_GUI_Init(tGUICommRXCharCallback RxByteCallback)
{
    // Store callback for ISR RX Byte
    RxByteISRCallback = RxByteCallback;

    // Configure UART
    UCA0CTL1 |= UCSWRST;            // Hold eUSCI in reset

#if (HAL_GUICOMM_BAUDRATE == 921600)
    // 921600 baud
    UCA0CTL1 |= UCSSEL__SMCLK;      // SMCLK
    UCA0BR0   = 1;                  // 921600 baud
    UCA0BR1   = 0;
    UCA0MCTLW = 0xDD10 | UCOS16;    // 16.384MHz/921600 = 17.7778 (See UG)
#elif (HAL_GUICOMM_BAUDRATE == 115200)
    // 115200 baud
    UCA0CTL1 |= UCSSEL__SMCLK;      // SMCLK
    UCA0BR0   = 142;                // 115200 baud
    UCA0BR1   = 0;
    UCA0MCTLW = 0x2200;             // 16.384MHz/115200 = 142.22 (See UG)
#elif (HAL_GUICOMM_BAUDRATE == 9600)
    // 9600 baud
    UCA0CTL1 |= UCSSEL__SMCLK;      // SMCLK
    UCA0BR0   = 0xAA;               // 9600 baud
    UCA0BR1   = 0x06;
    UCA0MCTLW = 0xD600;             // 16.384MHz/9600 = 1706.6667 (See UG)*/
#else
#error "Define UART baudrate registers based on desired frequency"
#endif

    UCA0CTL1 &= ~UCSWRST;           // Release from reset
    UCA0IE   |= UCRXIE;             // Enable RX interrupt
}

void HAL_GUI_TransmitCharBlocking(char character)
{
    // Transmit Character
    while (UCA0STATW & UCBUSY)
        ;
    while (!(UCA0IFG & UCTXIFG))
        ;
    UCA0TXBUF = character;
    while (UCA0STATW & UCBUSY)
        ;
}

// EUSCI interrupt service routine
// In SPI-slave builds the UCA0 vector is owned by the SPI ISR in main.c,
// so this UART RX ISR is only compiled when the UART path is selected.
#ifdef USE_UART_OUTPUT
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG:
            if (RxByteISRCallback != NULL)
            {
                 if (RxByteISRCallback(UCA0RXBUF) == true)
                 {
                     __bic_SR_register_on_exit(LPM3_bits);   // Exit LPM
                 }
            }
            break;
        case USCI_UART_UCTXIFG: break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
        default: break;
    }
}
#endif  // USE_UART_OUTPUT
