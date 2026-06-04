/* --COPYRIGHT--,BSD_EX
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
 *
 *******************************************************************************
 * 
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430i20xx Sigma-Delta ADC Demo
//
//  Description: This code uses the SD24 ADC module to perform continuous
//  conversions on a group of two ADC channels (A0.0+/- and A1.0+/-) at a
//  sampling frequency (data rate) of 4kHz. A SD24 ADC interrupt occurs whenever
//  the conversions have been completed. The MSP430i2041's MSP-TS430RHB32A
//  target socket board and even the EVM430-i2040S evaluation board can be used
//  for testing and development.
//
//  Test by applying signals to the 2 ADC channels and by viewing the
//  conversion results (upper 16 bits only) in the GUI. Adjust the gain and
//  preload settings for each ADC channel in the GUI, but be aware that the
//  full-scale range (FSR) changes with gain. Status LED D1 is used to indicate
//  that the MSP430i20xx device has powered up and also toggles when commands are
//  received and processed from the GUI.
//
//  ACLK = 32kHz, MCLK = SMCLK = Calibrated DCO = 16.384MHz, SD_CLK = 1.024MHz
//  * Ensure low_level_init.c is included when building/running this example *
//
//  NOTES: 1) For minimum VCC required for SD24 module, see the datasheet
//         2) 100nF cap between VREF and AVSS is recommended when using 1.2V REF
//
//               MSP430i20xx
//             ----------------
//            |                |
//            |   P1.2/UCA0RXD |<-- From PC
//        /|\ |   P1.3/UCA0TXD |--> To PC
//         |  |                |
//         ---|RST        P1.4 |--> D1 (status LED)
//            |                |
//   Vin0+ -->| A0.0+     VREF |---+
//   Vin0- -->| A0.0-          |   |
//   Vin1+ -->| A1.0+          |  -+- 100nF
//   Vin1- -->| A1.0-          |  -+-
//            |                |   |
//            |           AVSS |---+
//
//  James Evans
//  Texas Instruments, Inc
//  November 2020
//  Built with Code Composer Studio v10
//******************************************************************************

// #includes for header files
#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <HAL.h>
#include <GUI_mpack.h>
#include <GUIComm.h>
#include <QmathLib.h>
#include <callbacks_mpack.h>

// # defines for application code
#define STR_LEN_ONE      1

// Global variables
volatile uint8_t  command, adcChannel, adcA0Preload, adcA1Preload, adcA0Gain, adcA1Gain, adcReady;
volatile uint16_t adcA0Data, adcA1Data, adcA2Data, adcA3Data;
volatile uint32_t sample_count;                 // Incremented every ADC group; doubles as timestamp
static   int16_t  ext_temp_c100;                 // Latest AT30TS74 reading in 0.01 degC (0x8000 = sensor invalid)

//*****************************************************************************
// SPI slave protocol (SUN_SENSOR_SPI v2) -- see top-of-file design comment
//*****************************************************************************
#ifdef USE_SPI_OUTPUT

#define SPI_PROTOCOL_VERSION  (0x02)   // v2: FRAME temp field is now centi-degC, not the raw AT30TS74 register

// Commands (first byte master sends; slave's same-byte response is 0xFF idle)
#define CMD_READ_ID       (0xA0u)   // -> 4 bytes : 'S' 'U' 'N' version
#define CMD_READ_STATUS   (0xA1u)   // -> 2 bytes : flags, new_data
#define CMD_READ_FRAME    (0xA2u)   // -> 27 bytes : full atomic frame + CRC16
#define CMD_NOP           (0xAFu)   // -> 1 byte  : 0x00 (sanity ping)

#define SPI_FRAME_LEN     (27u)     // FRAME response length
#define SPI_IDLE_BYTE     (0xFFu)   // pre-loaded TX when no transaction is active
#define SPI_ERR_BYTE      (0xFFu)   // returned for unknown commands

// Constant responses (kept in flash via `static const`)
static const uint8_t id_response[4] = { 'S', 'U', 'N', SPI_PROTOCOL_VERSION };
static const uint8_t nop_response[1] = { 0x00 };
static const uint8_t err_response[1] = { SPI_ERR_BYTE };

// Double-buffered FRAME payload: main loop builds into the non-published
// buffer, then atomically flips `spi_ready_idx` to publish.  SPI ISR snapshots
// `spi_ready_idx` at the start of a transaction so the frame seen by the
// master is always coherent (never mid-build).  The atomicity guard is the
// volatile single-byte index, not the buffers themselves.
static uint8_t          spi_tx_buf[2][SPI_FRAME_LEN];
static volatile uint8_t spi_ready_idx = 0;

// 2-byte STATUS response, rebuilt on each READ_STATUS transaction.
static uint8_t spi_status_buf[2];

// SPI slave response-streaming state (4-wire hardware-framed, always-RX model).
//   spi_state = index of the NEXT response byte to queue into UCA0TXBUF.
// A received byte in the command range (0xA0..0xAF) starts a new transaction:
// it (re)loads spi_response_ptr/len and sets spi_state = 1.  Any other received
// byte is a dummy that advances the response.  There is NO CS-edge interrupt in
// 4-wire mode (eUSCI SPI exposes no STE interrupt), so the command opcode --
// not a GPIO edge -- is what reframes the byte stream.
static volatile uint8_t  spi_state;
static          uint8_t  spi_response_len;
static const uint8_t    *spi_response_ptr;
static volatile uint8_t  new_data_pending;          // 1 = fresh frame ready

// Bring-up aid: a non-zero .data global so the CCS Expressions view can confirm
// the new binary is actually on the chip (Build does NOT auto-flash -- you must
// Run -> Load -> Load Program).  Reads 0x4D when this 4-wire build is loaded;
// reads 0x00 if the chip is still running an older binary.  Safe to delete.
volatile uint8_t spi_build_sentinel = 0x4Du;
#endif  // USE_SPI_OUTPUT

//! \brief RX Command structure.
//!         The corresponding callback will be called when the command is
//!         received from GUI.
//! Note: Shorter command names take less time to process
const tGUI_RxCmd GUI_RXCommands[] =
{
    {"3",  GUICallback_SetA0Preload},
    {"4",  GUICallback_SetA1Preload},
    {"5",  GUICallback_SetA0Gain},
    {"6",  GUICallback_SetA1Gain},
};

#ifdef USE_UART_OUTPUT
// Helpers for raw ASCII output over the UART (bypasses the mpack/GUI layer
// so a plain serial terminal can read the four ADC channels as CSV).
static void uart_send_int16(int16_t v)
{
    char buf[6];                                    // max "-32768" fits in 6 chars
    int32_t sv = v;                                 // promote so -INT16_MIN doesn't overflow
    bool neg = (sv < 0);
    if (neg) sv = -sv;
    uint16_t uv = (uint16_t)sv;
    uint8_t n = 0;
    do {
        buf[n++] = (char)('0' + (uv % 10));
        uv /= 10;
    } while (uv);
    if (neg) HAL_GUI_TransmitCharBlocking('-');
    while (n > 0) HAL_GUI_TransmitCharBlocking(buf[--n]);
}

static void uart_send_uint32(uint32_t v)
{
    char buf[10];                                   // max uint32 = "4294967295" = 10 chars
    uint8_t n = 0;
    do {
        buf[n++] = (char)('0' + (v % 10));
        v /= 10;
    } while (v);
    while (n > 0) HAL_GUI_TransmitCharBlocking(buf[--n]);
}

static void uart_send_str(const char *s)
{
    while (*s) HAL_GUI_TransmitCharBlocking(*s++);
}
#endif  // USE_UART_OUTPUT

//*****************************************************************************
// External temperature sensor: AT30TS74 on UCB0 I2C
//   P1.6 = UCB0SCL, P1.7 = UCB0SDA (4.7k pull-ups on the board, not internal)
//   Slave address 0x48 (A0=A1=A2 grounded per schematic)
//   Default config: 9-bit resolution, continuous conversion, ~25 ms conversion
//   time -- we poll once per second to stay well clear of mid-conversion reads.
//*****************************************************************************

#define AT30TS74_ADDR       (0x48u)
#define AT30TS74_REG_TEMP   (0x00u)
#define TEMP_C100_INVALID   ((int16_t)0x8000)   // -327.68 degC sentinel: sensor NACKed / absent

static void i2c_init(void)
{
    // Pin function: P1.6 = UCB0SCL, P1.7 = UCB0SDA (function 1 on i2041).
    // P1SEL0=1, P1SEL1=0 -- matches TI's official msp430i20xx_euscib0_i2c_10.
    P1SEL0 |=  BIT6 | BIT7;
    P1SEL1 &= ~(BIT6 | BIT7);

    // eUSCI_B0 as I2C master.  Use the 16-bit UCB0CTLW0 word (the bit macros
    // UCMST/UCMODE_3/UCSYNC/UCSSEL_2 are defined against the 16-bit register;
    // writing the 8-bit CTL0/CTL1 aliases would truncate them).
    // Byte-counter + automatic STOP (UCASTP_2 + UCB0TBCNT) is the reliable
    // eUSCI_B read mechanism -- manual repeated-start/STOP timing is fragile.
    UCB0CTLW0 |= UCSWRST;                       // hold in reset
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSYNC | UCSSEL_2;   // I2C master, SMCLK
    UCB0CTLW1 |= UCASTP_2;                      // auto STOP after UCB0TBCNT bytes
    UCB0BRW    = 0x0080;                        // SMCLK / 128 ~= 128 kHz SCL
    UCB0TBCNT  = 2;                             // 2 bytes per transaction
    UCB0I2CSA  = AT30TS74_ADDR;
    UCB0CTLW0 &= ~UCSWRST;                      // release
}

// Polled I2C read of the AT30TS74, returned as a signed temperature in units
// of 0.01 degC (centi-degC).  The chip powers up with its register pointer at
// 0x00 (temperature) and we never move it, so a pure 2-byte read returns the
// temperature register -- no register-pointer write needed.
//
// The register is 16-bit, left-justified, two's complement, so T = raw/256 degC
// for any resolution (unused low bits read 0; 9-bit default = 0.5 degC steps).
// We convert on-chip: centi-degC = raw * 100 / 256, rounded to nearest.
//
// Returns TEMP_C100_INVALID (0x8000) if the slave NACKs (sensor missing / wrong
// address) rather than hanging the main loop.
static int16_t i2c_read_temp(void)
{
    while (UCB0CTLW0 & UCTXSTP);                // wait for any prior STOP
    UCB0IFG &= ~UCNACKIFG;

    UCB0CTLW0 &= ~UCTR;                         // receiver mode
    UCB0CTLW0 |= UCTXSTT;                       // START + slave address (read)

    // First byte (MSB) or NACK on the address phase
    while (!(UCB0IFG & (UCRXIFG0 | UCNACKIFG)));
    if (UCB0IFG & UCNACKIFG) {
        UCB0CTLW0 |= UCTXSTP;
        while (UCB0CTLW0 & UCTXSTP);
        return TEMP_C100_INVALID;              // no slave responding
    }
    uint8_t msb = UCB0RXBUF;                    // reading clears UCRXIFG0

    // Second byte (LSB); hardware auto-generates STOP after UCB0TBCNT=2 bytes
    while (!(UCB0IFG & UCRXIFG0));
    uint8_t lsb = UCB0RXBUF;

    while (UCB0CTLW0 & UCTXSTP);                // wait auto-STOP to complete

    int16_t raw = (int16_t)(((uint16_t)msb << 8) | lsb);
    return (int16_t)(((int32_t)raw * 100 + 128) >> 8);   // raw/256 -> 0.01 degC, rounded
}

#ifdef USE_SPI_OUTPUT

//*****************************************************************************
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xor-out).
// Bit-by-bit; ~30 cyc/byte.  Computing over 25 bytes is ~750 cycles ~= 46 us
// at 16.384 MHz, well within the 8 ms frame budget.
//*****************************************************************************
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    while (len--) {
        crc ^= ((uint16_t)*data++) << 8;
        uint8_t i;
        for (i = 0; i < 8; i++) {
            if (crc & 0x8000u) crc = (uint16_t)((crc << 1) ^ 0x1021u);
            else               crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

//*****************************************************************************
// SPI slave init: UCA0 as 4-pin hardware-framed SPI slave (UCMODE_2, STE
// active-low on P1.0).  The eUSCI peripheral handles chip-select framing in
// hardware: while STE (CS) is low the slave shifts; when CS goes high MISO is
// tri-stated and the bit counter realigns, so every transaction starts on a
// clean byte boundary with no software CS handling.
//
// We deliberately do NOT use a P1.0 GPIO port interrupt anymore (the pin is
// muxed to UCA0STE).  eUSCI SPI exposes no STE-edge interrupt, so the byte
// stream is reframed in software on the command opcode (see USCI_A0_SPI_ISR).
//
// SPI Mode 0: CPOL = 0 (clock idle low), CPHA = 0 (capture on first edge).
// eUSCI encoding: UCCKPH = 1 (capture on first edge), UCCKPL = 0 (idle low).
//*****************************************************************************
static void spi_init(void)
{
    // 4-pin SPI: P1.0 = UCA0STE (CS in), P1.1 = UCA0CLK (in),
    // P1.2 = UCA0SOMI (out), P1.3 = UCA0SIMO (in).
    P1SEL0 |=  (BIT0 | BIT1 | BIT2 | BIT3);
    P1SEL1 &= ~(BIT0 | BIT1 | BIT2 | BIT3);

    // eUSCI_A0 as 4-pin SPI slave, STE active-low (UCMODE_2).  Bit macros are
    // 16-bit oriented -> use the UCA0CTLW0 word (avoids 8-bit alias truncation).
    // UCMST = 0 -> slave (clock from master).  UCSTEM is a master-only bit and
    // is ignored in slave mode, so it is left clear.
    UCA0CTLW0  = UCSWRST;                           // hold in reset; clear all else
    UCA0CTLW0 |= UCMSB | UCSYNC | UCCKPH | UCMODE_2;  // MSB first, Mode 0, 4-pin STE-low
    UCA0CTLW0 &= ~UCSWRST;                          // release

    // Preload TXBUF with the idle byte: the master's first two MISO bytes (CMD
    // slot + the unavoidable decode-latency byte) read back 0xFF, then the
    // response follows at a deterministic offset of 2.  Arm RXIE only: every
    // received byte drives the response stream from the RX ISR (TXIE is not
    // used -- the always-RX model gives each response byte a full byte-time of
    // load margin while keeping a fixed 2-byte lead).
    UCA0TXBUF = SPI_IDLE_BYTE;
    UCA0IE   |= UCRXIE;
}

//*****************************************************************************
// Build a full FRAME payload (27 bytes including CRC) into the non-published
// half of the double buffer, then atomically publish by flipping the index.
// Called from main loop (NOT ISR) so the heavier compute stays out of ISR.
//*****************************************************************************
static void spi_publish_frame(uint32_t sc,
                              int16_t a0, int16_t a1, int16_t a2, int16_t a3,
                              int16_t sx_i, int16_t sy_i, int16_t sz_i,
                              uint32_t sum_u, int16_t tc100, uint8_t flags)
{
    uint8_t bi = (uint8_t)(spi_ready_idx ^ 1u);     // build into the OTHER buffer
    uint8_t *b = spi_tx_buf[bi];

    b[ 0] = (uint8_t)(sc >> 24);
    b[ 1] = (uint8_t)(sc >> 16);
    b[ 2] = (uint8_t)(sc >>  8);
    b[ 3] = (uint8_t)(sc      );

    b[ 4] = (uint8_t)(sx_i >> 8); b[ 5] = (uint8_t)sx_i;
    b[ 6] = (uint8_t)(sy_i >> 8); b[ 7] = (uint8_t)sy_i;
    b[ 8] = (uint8_t)(sz_i >> 8); b[ 9] = (uint8_t)sz_i;

    b[10] = (uint8_t)(a0 >> 8);   b[11] = (uint8_t)a0;
    b[12] = (uint8_t)(a1 >> 8);   b[13] = (uint8_t)a1;
    b[14] = (uint8_t)(a2 >> 8);   b[15] = (uint8_t)a2;
    b[16] = (uint8_t)(a3 >> 8);   b[17] = (uint8_t)a3;

    b[18] = (uint8_t)(sum_u >> 24);
    b[19] = (uint8_t)(sum_u >> 16);
    b[20] = (uint8_t)(sum_u >>  8);
    b[21] = (uint8_t)(sum_u      );

    b[22] = (uint8_t)(tc100 >> 8); b[23] = (uint8_t)tc100;

    b[24] = flags;

    uint16_t crc = crc16_ccitt(b, 25);
    b[25] = (uint8_t)(crc >> 8);
    b[26] = (uint8_t)crc;

    spi_ready_idx = bi;                             // publish atomically (1-byte write)
    new_data_pending = 1u;
}

#endif  // USE_SPI_OUTPUT

//*****************************************************************************
// Sun sensor algorithm (pinhole aperture + 2x2 quadrant photodiode)
//*****************************************************************************

// Per-channel calibration — refine with dark-cover and uniform-light routines.
// On this board the analog chain delivers ~0 LSB dark, ~32767 LSB saturated.
static int16_t dark_off[4] = { 0, 0, 0, 0 };
static float   gain_k[4]   = { 1.0f, 1.0f, 1.0f, 1.0f };

// Geometry of the aperture / photodiode pack (mm).
// h = 0.8 mm is the mechanical minimum; chosen to give per-axis linear FOV
// of ~+/-56 deg so cube-corner directions (54.7 deg from any face normal)
// land inside the linear FOV of adjacent sun sensors with ~10 deg overlap.
#define APERTURE_MM         (2.6f)
#define APERTURE_HEIGHT_MM  (0.8f)
#define ARRAY_SIDE_MM       (5.0f)
#define K_GEOM              (APERTURE_MM / (2.0f * APERTURE_HEIGHT_MM))  // ~1.625
#define OFF_FOV_LIMIT       (0.9f)
#define SUN_PRESENT_LSB     (200)    // sum (post-offset) below this -> "no sun"

// Status bits OR'd into the last UART field.
#define FLAG_NO_SUN     (1u)
#define FLAG_OFF_FOV    (2u)
#define FLAG_SATURATED  (4u)

// Saturation threshold: 80% of VFSR per i2041 datasheet recommendation.
// VFSR+ = +VREF/GAIN = +1.25V at PGA gain=1; 80% of VFSR = +1.0V differential,
// which is also where R_f*I_max ~= 180 Ohm * 5.5 mA puts a perfectly-sunlit
// quadrant.  ADC count for 1.0V signal = 1.0/1.25 * 32767 = 26213.6.
#define SAT_THRESHOLD_LSB   (26214)

//! \brief Compute sun direction (unit vector in sensor body frame) from 4 ADCs.
//!
//! Quadrant map: A0=BL, A1=TL, A2=TR, A3=BR.  Body frame: +X right, +Y top,
//! +Z sensor normal.  Returns status flag bitmask; *sx/*sy/*sz are always
//! written (zeroed when no sun is present); *sum_out is the dark-offset-
//! corrected sum of all four channels (used host-side to detect
//! spot-clipping past the linear FOV — sum drops as the spot leaves the
//! photodiode array even when the centroid saturates).
static uint8_t compute_sun_vector(int16_t a0, int16_t a1, int16_t a2, int16_t a3,
                                  float *sx, float *sy, float *sz,
                                  int32_t *sum_out)
{
    uint8_t flags = 0;

    // V+ = VREF, V- = TIA out (drops with light) -> readings are always in
    // [0, +32767] on this board.  Flag well before the hard rail: anything
    // above 80% of VFSR is in the SD24's non-linear region per datasheet.
    if (a0 >= SAT_THRESHOLD_LSB || a1 >= SAT_THRESHOLD_LSB ||
        a2 >= SAT_THRESHOLD_LSB || a3 >= SAT_THRESHOLD_LSB) {
        flags |= FLAG_SATURATED;
    }

    // Dark-offset subtract + per-channel gain trim.
    float p0 = gain_k[0] * (float)((int32_t)a0 - dark_off[0]);   // BL
    float p1 = gain_k[1] * (float)((int32_t)a1 - dark_off[1]);   // TL
    float p2 = gain_k[2] * (float)((int32_t)a2 - dark_off[2]);   // TR
    float p3 = gain_k[3] * (float)((int32_t)a3 - dark_off[3]);   // BR

    float sum = p0 + p1 + p2 + p3;
    *sum_out = (int32_t)sum;

    if (sum < (float)SUN_PRESENT_LSB) {
        flags |= FLAG_NO_SUN;
        *sx = 0.0f; *sy = 0.0f; *sz = 0.0f;
        return flags;
    }

    // Normalized centroid: p2,p3 = right column; p1,p0 = left column.
    //                     p1,p2 = top row;       p0,p3 = bottom row.
    float x_c = ((p2 + p3) - (p1 + p0)) / sum;
    float y_c = ((p1 + p2) - (p0 + p3)) / sum;

    if (fabsf(x_c) > OFF_FOV_LIMIT || fabsf(y_c) > OFF_FOV_LIMIT) {
        flags |= FLAG_OFF_FOV;
    }

    // Sun vector from the centroid.  With tan(alpha)=u and tan(beta)=v, the
    // identities sin(atan u)=u/sqrt(1+u^2) and cos(atan u)=1/sqrt(1+u^2) let
    // the original sin/cos products collapse to a closed form needing ONE
    // sqrt and no transcendental trig:
    //     sx = sin(a)cos(b) = u / sqrt((1+u^2)(1+v^2))
    //     sy = cos(a)sin(b) = v / sqrt((1+u^2)(1+v^2))
    //     sz = cos(a)cos(b) = 1 / sqrt((1+u^2)(1+v^2))
    // This is exact (not an approximation) and ~10x faster.  The previous
    // 6 software-float trig calls (2x atan2f + 2x sinf + 2x cosf) cost ~12 ms
    // per frame on this FPU-less MCU, throttling the 125 Hz frame stream down
    // to ~79 Hz of actually-sent frames.  We only emit the vector (not the
    // angles), so atan2 is never needed.
    float u = x_c * K_GEOM;
    float v = y_c * K_GEOM;
    float inv_denom = 1.0f / sqrtf((1.0f + u * u) * (1.0f + v * v));
    *sx = u * inv_denom;
    *sy = v * inv_denom;
    *sz = inv_denom;

    return flags;
}

// Watchdog: ACLK-sourced auto-reset, 1000 ms timeout.  Petted every main-loop
// iteration (~8 ms cadence, so we always pet well before timeout).  The
// header pre-defines WDT_ARST_1000 = WDTPW + WDTCNTCL + WDTSSEL for exactly
// this configuration.
#define WDT_RUN  WDT_ARST_1000

// Main
void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                       // Stop WDT during init

    // Capture reset reason before anything clears it; emitted on UART once
    // the link is up so the host knows whether boot was POR / BOR / WDT / etc.
    // i2041 uses the legacy IFG1-based reset flag scheme (no SYSRSTIV).
    // Bits: WDTIFG=0x01 (WDT timeout or password violation), BORIFG=0x04,
    // RSTIFG=0x08 (external RST/NMI pin).  Value 0x00 = clean POR / power-up.
    uint8_t reset_flags = IFG1 & (WDTIFG | BORIFG | RSTIFG);
    IFG1 &= ~(WDTIFG | BORIFG | RSTIFG);

    // Initialize clocks, GPIOs
    HAL_System_Init();

    // Initialize variables
    adcReady = 0;                                   // Clear ADC ready flag
    command = 0;                                    // Clear command

    // Configure SD24 reference
    SD24CTL = SD24REFS;                             // Internal reference
    __delay_cycles(3600);                           // Delay ~200us for 1.2V reference to settle

    // Configure SD24 ADC channels (quadrant photodiode: 4 TIAs on A0..A3).
    // i2041 SD24 has only 4 channels; internal Ts would require sacrificing
    // one channel (via SD24INCH MUX) -- not done here.  Temperature should
    // come from the external I2C sensor instead (TBD which chip).
    SD24INCTL0 |= SD24GAIN_1;                       // PGA gain for all four channels
    SD24INCTL1 |= SD24GAIN_1;
    SD24INCTL2 |= SD24GAIN_1;
    SD24INCTL3 |= SD24GAIN_1;
    SD24CCTL0  |= SD24OSR_256 | SD24DF | SD24GRP;   // OSR = 256, 2's complement; group A0..A3
    SD24CCTL1  |= SD24OSR_256 | SD24DF | SD24GRP;
    SD24CCTL2  |= SD24OSR_256 | SD24DF | SD24GRP;
    SD24CCTL3  |= SD24OSR_256 | SD24DF | SD24IE;    // Group end: interrupt fires when all 4 ready

    // Host interface init: SPI slave (default/production) or UART (legacy/dev).
    // UCA0 is shared by both modes -- they cannot coexist at runtime.
#ifdef USE_SPI_OUTPUT
    spi_init();
    (void)reset_flags;                              // not exposed over SPI for now
#else
    GUI_Init();                                                                              // Initialize GUI layer
    GUI_InitRxCmd( &GUI_RXCommands[0],(sizeof(GUI_RXCommands)/sizeof(GUI_RXCommands[0])) );  // Initialize GUI receive
#endif

    // Initialize I2C master for the external AT30TS74 temperature sensor
    i2c_init();
    ext_temp_c100 = i2c_read_temp();                 // prime with one reading

    // Start ADC conversions (trigger on group end)
    SD24CCTL3  |= SD24SC;

    // Enable interrupts
    __bis_SR_register(GIE);

#ifdef USE_UART_OUTPUT
    // Header — A0..A3 raw signed 16-bit ADC counts, sx/sy/sz unit-vector x10000,
    // sum is the dark-offset-corrected total (use to detect spot clipping past
    // linear FOV), temp_c100 is the on-chip temperature in 0.01 degC
    // (host: T_C = temp_c100 / 100; 0x8000 = sensor invalid), flags: 1=no_sun,
    // 2=off_fov, 4=saturated.
    uart_send_str("# timestamp A0 A1 A2 A3 sx_x10000 sy_x10000 sz_x10000 sum temp_c100 flags\r\n");

    // Emit reset cause so host can correlate boot events.  i2041 IFG1 bitmask:
    // 0x00 = clean POR, 0x01 = WDTIFG (watchdog timeout or password violation),
    // 0x04 = BORIFG (brown-out), 0x08 = RSTIFG (external RST pin).
    // Bits can OR together if multiple events accumulated.
    uart_send_str("# reset_flags ");
    uart_send_uint32((uint32_t)reset_flags);
    uart_send_str("\r\n");
#endif

    // Start the watchdog now that init is complete and the main loop is
    // about to start petting it.  ACLK keeps running through LPM0 so the
    // WDT counter still progresses while we sleep -- this is what we want
    // (a hung main loop will reset us after ~1 sec).
    WDTCTL = WDT_RUN;

    // Communication state machine
    while(1)
    {
        WDTCTL = WDT_RUN;                           // Pet watchdog every iteration

        // Process ADC_A0_PRELOAD command
        if(command == ADC_A0_PRELOAD)
        {
            P1OUT ^= BIT4;                          // Toggle status LED to indicate RX

            // Set preload based on GUI inputs
            SD24PRE0 = adcA0Preload;                // Set ADC channel preload

            // Start ADC conversions
            SD24CCTL3 |= SD24IE;                    // Enable ADC interrupts
            SD24CCTL3 |= SD24SC;                    // Set bit to start ADC conversions (group end)
            command = 0;                            // Clear command
        }

        // Process ADC_A1_PRELOAD command
        else if(command == ADC_A1_PRELOAD)
        {
            P1OUT ^= BIT4;                          // Toggle status LED to indicate RX

            // Set preload based on GUI inputs
            SD24PRE1 = adcA1Preload;                // Set ADC channel preload

            // Start ADC conversions
            SD24CCTL3 |= SD24IE;                    // Enable ADC interrupts
            SD24CCTL3 |= SD24SC;                    // Set bit to start ADC conversions (group end)
            command = 0;                            // Clear command
        }

        // Process ADC_A0_GAIN command
        else if(command == ADC_A0_GAIN)
        {
            P1OUT ^= BIT4;                          // Toggle status LED to indicate RX

            // Reset PGA gain to 1
            SD24INCTL0 &= ~(SD24GAIN0 | SD24GAIN1 | SD24GAIN2);

            // Set PGA gain based on GUI inputs
            if(adcA0Gain == 1)
            {
                SD24INCTL0 |= SD24GAIN_1;           // Set PGA gain to 1
            }
            else if(adcA0Gain == 2)
            {
                SD24INCTL0 |= SD24GAIN_2;           // Set PGA gain to 2
            }
            else if(adcA0Gain == 4)
            {
                SD24INCTL0 |= SD24GAIN_4;           // Set PGA gain to 4
            }
            else if(adcA0Gain == 8)
            {
                SD24INCTL0 |= SD24GAIN_8;           // Set PGA gain to 8
            }
            else if(adcA0Gain == 16)
            {
                SD24INCTL0 |= SD24GAIN_16;          // Set PGA gain to 16
            }

            GUIComm_sendUInt8("5", STR_LEN_ONE, adcA0Gain);     // Send received gain to GUI for confirmation

            // Start ADC conversions
            SD24CCTL3 |= SD24IE;                    // Enable ADC interrupts
            SD24CCTL3 |= SD24SC;                    // Set bit to start ADC conversions (group end)
            command = 0;                            // Clear command
        }

        // Process ADC_A1_GAIN command
        else if(command == ADC_A1_GAIN)
        {
            P1OUT ^= BIT4;                          // Toggle status LED to indicate RX

            // Reset PGA gain to 1
            SD24INCTL1 &= ~(SD24GAIN0 | SD24GAIN1 | SD24GAIN2);

            // Set PGA gain based on GUI inputs
            if(adcA1Gain == 1)
            {
                SD24INCTL1 |= SD24GAIN_1;           // Set PGA gain to 1
            }
            else if(adcA1Gain == 2)
            {
                SD24INCTL1 |= SD24GAIN_2;           // Set PGA gain to 2
            }
            else if(adcA1Gain == 4)
            {
                SD24INCTL1 |= SD24GAIN_4;           // Set PGA gain to 4
            }
            else if(adcA1Gain == 8)
            {
                SD24INCTL1 |= SD24GAIN_8;           // Set PGA gain to 8
            }
            else if(adcA1Gain == 16)
            {
                SD24INCTL1 |= SD24GAIN_16;          // Set PGA gain to 16
            }

            GUIComm_sendUInt8("6", STR_LEN_ONE, adcA1Gain);     // Send received gain back to GUI

            // Start ADC conversions
            SD24CCTL3 |= SD24IE;                    // Enable ADC interrupts
            SD24CCTL3 |= SD24SC;                    // Set bit to start ADC conversions (group end)
            command = 0;                            // Clear command
        }

        // Process a fresh averaged frame (every 8 ms = 125 Hz).  Common path
        // for both interfaces: snapshot -> temp poll -> compute vector ->
        // then either UART-emit (legacy text) or SPI-publish (production).
        else if(adcReady == 1)
        {
            int16_t a0, a1, a2, a3;
            uint32_t sc;

            // Atomic snapshot of all volatile values updated by the SD24 ISR.
            // The ISR writes them all at once (every 32 raw samples = 8 ms);
            // a snapshot guarantees the row we emit is a coherent frame.
            __bic_SR_register(GIE);
            a0 = adcA0Data; a1 = adcA1Data;
            a2 = adcA2Data; a3 = adcA3Data;
            sc = sample_count;
            adcReady = 0;
            __bis_SR_register(GIE);

            // Refresh temperature about once a second (125 Hz output rate).
            // I2C read is blocking ~200 us; doesn't disturb the main cadence.
            static uint8_t temp_decim = 0;
            if (++temp_decim >= 125) {
                ext_temp_c100 = i2c_read_temp();
                temp_decim = 0;
            }

            float sx, sy, sz;
            int32_t sum;
            uint8_t flags = compute_sun_vector(a0, a1, a2, a3,
                                               &sx, &sy, &sz, &sum);
            int16_t sx_i = (int16_t)(sx * 10000.0f);
            int16_t sy_i = (int16_t)(sy * 10000.0f);
            int16_t sz_i = (int16_t)(sz * 10000.0f);
            uint32_t sum_u = (sum < 0) ? 0u : (uint32_t)sum;

#ifdef USE_SPI_OUTPUT
            spi_publish_frame(sc, a0, a1, a2, a3,
                              sx_i, sy_i, sz_i,
                              sum_u, ext_temp_c100, flags);
#else
            uart_send_uint32(sc);
            HAL_GUI_TransmitCharBlocking(' ');
            uart_send_int16(a0);
            HAL_GUI_TransmitCharBlocking(' ');
            uart_send_int16(a1);
            HAL_GUI_TransmitCharBlocking(' ');
            uart_send_int16(a2);
            HAL_GUI_TransmitCharBlocking(' ');
            uart_send_int16(a3);
            HAL_GUI_TransmitCharBlocking(' ');
            uart_send_int16(sx_i);
            HAL_GUI_TransmitCharBlocking(' ');
            uart_send_int16(sy_i);
            HAL_GUI_TransmitCharBlocking(' ');
            uart_send_int16(sz_i);
            HAL_GUI_TransmitCharBlocking(' ');
            uart_send_uint32(sum_u);
            HAL_GUI_TransmitCharBlocking(' ');
            uart_send_int16(ext_temp_c100);
            HAL_GUI_TransmitCharBlocking(' ');
            uart_send_uint32((uint32_t)flags);
            HAL_GUI_TransmitCharBlocking('\r');
            HAL_GUI_TransmitCharBlocking('\n');
#endif
        }

        // Nothing to do: sleep in LPM0 (CPU off, SMCLK + ACLK still running
        // so SD24, UART, and WDT keep ticking).  Woken by SD24 ISR every 8 ms
        // (32-sample average complete) or by UART RX ISR if a command arrives.
        else
        {
            __bis_SR_register(LPM0_bits | GIE);
        }
    }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=SD24_VECTOR
__interrupt void SD24_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(SD24_VECTOR))) SD24_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch (__even_in_range(SD24IV,SD24IV_SD24MEM3)) {
        case SD24IV_NONE: break;
        case SD24IV_SD24OVIFG: break;
        case SD24IV_SD24MEM0: break;
        case SD24IV_SD24MEM1: break;
        case SD24IV_SD24MEM2: break;
        case SD24IV_SD24MEM3:
        {
            // Block-average ADC_AVG_N raw samples per emitted frame.  i20xx
            // SD24 hardware OSR caps at 256, so we get extra noise rejection
            // by averaging in the digital domain.  Output rate is
            // 4 kHz / ADC_AVG_N; with N=32 this is 125 Hz which already
            // satisfies the 100 Hz vector update requirement.
            // Power-of-2 N lets us divide via shift in the ISR.
            #define ADC_AVG_N        (32u)
            #define ADC_AVG_SHIFT    (5u)            // log2(ADC_AVG_N)

            static int32_t a0_acc = 0, a1_acc = 0, a2_acc = 0, a3_acc = 0;
            static uint8_t avg_cnt = 0;

            a0_acc += SD24MEM0;                     // Reads clear IFGs
            a1_acc += SD24MEM1;
            a2_acc += SD24MEM2;
            a3_acc += SD24MEM3;

            if (++avg_cnt >= ADC_AVG_N) {
                adcA0Data = (int16_t)(a0_acc >> ADC_AVG_SHIFT);
                adcA1Data = (int16_t)(a1_acc >> ADC_AVG_SHIFT);
                adcA2Data = (int16_t)(a2_acc >> ADC_AVG_SHIFT);
                adcA3Data = (int16_t)(a3_acc >> ADC_AVG_SHIFT);
                a0_acc = a1_acc = a2_acc = a3_acc = 0;
                avg_cnt = 0;
                sample_count += ADC_AVG_N;          // keep timestamp in raw-sample units
                adcReady = 1;
                __bic_SR_register_on_exit(LPM0_bits); // wake main loop from LPM0
            }
            break;
        }
        default: break;
    }
}

//*****************************************************************************
// SPI slave ISR -- always-RX streaming model (4-wire hardware-framed).
//
// Every received byte (command + dummies) raises UCRXIFG and is handled here;
// there is no TXIE handoff and no CS-edge interrupt.  On each RXIFG we queue
// the NEXT outgoing byte into TXBUF, which the hardware loads into TXSHIFT at
// the following byte boundary -- giving a full byte-time of margin per byte.
//
// Framing: a received byte in the command range (0xA0..0xAF) starts a new
// transaction (decode -> response pointer/len, queue response[0]).  Any other
// received byte is a dummy that advances the response.  The master therefore
// MUST send command opcodes only in the CMD slot and keep dummy bytes out of
// 0xA0..0xAF (0x00 is the canonical dummy).  In 4-wire mode the hardware
// realigns the bit counter on every CS edge, so a glitched transaction self-
// heals on the next command.
//
// Lead byte is a DETERMINISTIC 2: byte 0 (CMD slot) and byte 1 read back the
// preloaded 0xFF (the decode ISR physically cannot update TXSHIFT before
// byte 1 -- RXIFG for the CMD and byte 1's TXSHIFT load happen on the same
// edge), and response[0] appears at byte 2.  The protocol's overclock (K=4)
// leaves TXBUF = 0xFF at rest so the next transaction's byte 0 reads 0xFF too.
//*****************************************************************************
#ifdef USE_SPI_OUTPUT

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_SPI_ISR(void)
{
    switch (__even_in_range(UCA0IV, USCI_SPI_UCTXIFG)) {
    case USCI_NONE:
        break;

    case USCI_SPI_UCRXIFG: {
        uint8_t b = UCA0RXBUF;                      // clears UCRXIFG

        if (b >= 0xA0u && b <= 0xAFu) {
            // Command opcode -> start of a new transaction.  Select the
            // response and queue its first byte; it appears at MISO byte 2
            // (bytes 0 and 1 are the deterministic 0xFF lead).
            const uint8_t *p;
            uint8_t len;
            switch (b) {
            case CMD_READ_FRAME:
                // Snapshot the published buffer; the SD24 main-loop path
                // rebuilds into the OTHER buffer so it can't corrupt us.
                p = spi_tx_buf[spi_ready_idx];
                len = SPI_FRAME_LEN;
                new_data_pending = 0u;
                break;
            case CMD_READ_STATUS:
                // Built fresh: flags from the latest published frame plus the
                // new-data sticky flag (cleared on read).
                spi_status_buf[0] = spi_tx_buf[spi_ready_idx][24];
                spi_status_buf[1] = new_data_pending;
                new_data_pending  = 0u;
                p = spi_status_buf;
                len = sizeof(spi_status_buf);
                break;
            case CMD_READ_ID:
                p = id_response;
                len = sizeof(id_response);
                break;
            case CMD_NOP:
                p = nop_response;
                len = sizeof(nop_response);
                break;
            default:                                // unknown 0xAx command
                p = err_response;
                len = sizeof(err_response);
                break;
            }
            spi_response_ptr = p;
            spi_response_len = len;
            UCA0TXBUF = p[0];
            spi_state = 1u;                          // next dummy queues response[1]
        } else {
            // Dummy byte -> advance the response, or idle once exhausted.
            // Each write has a full byte-time before the hardware loads it.
            if (spi_state < spi_response_len) {
                UCA0TXBUF = spi_response_ptr[spi_state];
                spi_state++;
            } else {
                UCA0TXBUF = SPI_IDLE_BYTE;
            }
        }
        break;
    }

    default:
        break;
    }
}

#endif  // USE_SPI_OUTPUT
