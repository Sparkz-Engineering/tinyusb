/*
 * The MIT License (MIT)
 *
 * Copyright(c) N Conrad
 * Copyright(c) 2024 Brandon Lane (Sparkz Engineering)
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the TinyUSB stack.
 */


#ifndef TUSB_FSDEV_APM32_H
#define TUSB_FSDEV_APM32_H

#if CFG_TUSB_MCU == OPT_MCU_APM32F0
  #include "apm32f0xx.h"
  #define FSDEV_PMA_SIZE (512u)
  #define FSDEV_REG_BASE USBD_BASE
  #define FSDEV_PMA_BASE (USBD_BASE + 0x00000400)

  // Convert APM32 CMSIS register names to ST standard names

  /* ------------------------- Interrupt Register Bits ------------------------ */
  #define USB_ISTR_CTR ((uint16_t) 0x8000U)   // CTFLG - Correct Transfer Flag
  #define USB_ISTR_PMAOVR ((uint16_t) 0x4000U)// PMOFLG - Packet Memory Overflow Flag
  #define USB_ISTR_ERR ((uint16_t) 0x2000U)   // ERRFLG - Failure of Transfer Flag
  #define USB_ISTR_WKUP ((uint16_t) 0x1000U)  // WUPREQ - Wakeup Request
  #define USB_ISTR_SUSP ((uint16_t) 0x0800U)  // SUSREQ - Suspend Mode Request
  #define USB_ISTR_RESET ((uint16_t) 0x0400U) // RSTREQ - USBD Reset Request
  #define USB_ISTR_SOF ((uint16_t) 0x0200U)   // SOFFLG - Start of Frame Flag
  #define USB_ISTR_ESOF ((uint16_t) 0x0100U)  // ESOFFLG - Expected Start of Frame Flag
  #define USB_ISTR_DIR ((uint16_t) 0x0010U)   // DOT - Direction of Transfer
  #define USB_ISTR_EP_ID ((uint16_t) 0x000FU) // EPID[3:0] - Endpoint Identifier

  /* Legacy defines */
  #define USB_ISTR_PMAOVRM USB_ISTR_PMAOVR

  #define USB_CLR_CTR (~USB_ISTR_CTR)      // Clear CTFLG
  #define USB_CLR_PMAOVR (~USB_ISTR_PMAOVR)// Clear PMOFLG
  #define USB_CLR_ERR (~USB_ISTR_ERR)      // CLear ERRFLG
  #define USB_CLR_WKUP (~USB_ISTR_WKUP)    // Clear WUPREQ
  #define USB_CLR_SUSP (~USB_ISTR_SUSP)    // Clear SUSREQ
  #define USB_CLR_RESET (~USB_ISTR_RESET)  // Clear RSTREQ
  #define USB_CLR_SOF (~USB_ISTR_SOF)      // Clear SOFFLG
  #define USB_CLR_ESOF (~USB_ISTR_ESOF)    // Clear ESOFFLG

  /* Legacy defines */
  #define USB_CLR_PMAOVRM USB_CLR_PMAOVR

  /* -------------------------- Control Register Bits ------------------------- */
  #define USB_CNTR_CTRM ((uint16_t) 0x8000U)  // CTRIEN - Corrent Transfer Interrupt Enable
  #define USB_CNTR_PMAOVR ((uint16_t) 0x4000U)// PMAOUIEN - Packet Memory Area Over / Underrun Interrupt Enable
  #define USB_CNTR_ERRM ((uint16_t) 0x2000U)  // ERRIEN - Error Interrupt Enable
  #define USB_CNTR_WKUPM ((uint16_t) 0x1000U) // WKUPIEN - Wakeup Interrupt Enable
  #define USB_CNTR_SUSPM ((uint16_t) 0x0800U) // SUSIEN - Suspend Mode Interrupt Enable
  #define USB_CNTR_RESETM ((uint16_t) 0x0400U)// RSTIEN - USBD Reset Interrupt Enable
  #define USB_CNTR_SOFM ((uint16_t) 0x0200U)  // SOFIEN - Start of Frame Interrupt Enable
  #define USB_CNTR_ESOFM ((uint16_t) 0x0100U) // ESOFIEN - Expected Start of Frame Interrupt Enable
  #define USB_CNTR_RESUME ((uint16_t) 0x0010U)// WKUPREQ - Wakeup Request
  #define USB_CNTR_FSUSP ((uint16_t) 0x0008U) // FORSUS - Force Suspend
  #define USB_CNTR_LPMODE ((uint16_t) 0x0004U)// LPWREN - Lowpower Mode Enable
  #define USB_CNTR_PDWN ((uint16_t) 0x0002U)  // PWRDOWN - Power Down
  #define USB_CNTR_FRES ((uint16_t) 0x0001U)  // FORSUS - Force Suspend

  /* Legacy defines */
  #define USB_CNTR_PMAOVRM USB_CNTR_PMAOVR
  #define USB_CNTR_LP_MODE USB_CNTR_LPMODE


  /* ------------------------ Frame State Register Bits ----------------------- */
  #define USB_FNR_RXDP ((uint16_t) 0x8000U)// RXDPSTS - Receive Data+ Line Status
  #define USB_FNR_RXDM ((uint16_t) 0x4000U)// RXDMSTS - Receive Data- Line Status
  #define USB_FNR_LCK ((uint16_t) 0x2000U) // LOCK - Lock
  #define USB_FNR_LSOF ((uint16_t) 0x1800U)// LSOFNUM[1:0] - Continuous Lost SOF Number
  #define USB_FNR_FN ((uint16_t) 0x07FFU)  // FRANUM[10:0] - Frame Sequence Numbers

  /* ---------------------- Device Address Register Bits ---------------------- */
  #define USB_DADDR_EF ((uint8_t) 0x80U) // USBDEN - USBD Enable
  #define USB_DADDR_ADD ((uint8_t) 0x7FU)// ADDR[6:0] - Device Address

  /* ---------------------------- Enpoint Register ---------------------------- */
  #define USB_EP0R USB_BASE          //Endpoint 0 Address
  #define USB_EP1R (USB_BASE + 0x04U)// Endpoint 1 Address
  #define USB_EP2R (USB_BASE + 0x08U)// Endpoint 2 Address
  #define USB_EP3R (USB_BASE + 0x0CU)// Endpoint 3 Address
  #define USB_EP4R (USB_BASE + 0x10U)// Endpoint 4 Address
  #define USB_EP5R (USB_BASE + 0x14U)// Endpoint 5 Address
  #define USB_EP6R (USB_BASE + 0x18U)// Endpoint 6 Address
  #define USB_EP7R (USB_BASE + 0x1CU)// Endpoint 7 Address

  // Register Bits
  #define USB_EP_CTR_RX ((uint16_t) 0x8000U)   // CTFR - Correct Transfer Flag for Reception
  #define USB_EP_DTOG_RX ((uint16_t) 0x4000U)  // RXDTOG - Data Toggle for Reception Transfers
  #define USB_EPRX_STAT ((uint16_t) 0x3000U)   // RXSTS[1:0] - Status Bits for Reception Transfers
  #define USB_EP_SETUP ((uint16_t) 0x0800U)    //SETUP - Endpoint Setup
  #define USB_EP_T_FIELD ((uint16_t) 0x0600U)  // TYPE - Endpoint Type
  #define USB_EP_KIND ((uint16_t) 0x0100U)     // KIND - Endpoint Kind
  #define USB_EP_CTR_TX ((uint16_t) 0x0080U)   // CTFT - Correct Transfer Flag for Transmission
  #define USB_EP_DTOG_TX ((uint16_t) 0x0040U)  // TXDTOG - Data Toggle for Transmission Transfers
  #define USB_EPTX_STAT ((uint16_t) 0x0030U)   // TXSTS[1:0] - Status Bits for Transmission Transfers
  #define USB_EPADDR_FIELD ((uint16_t) 0x000FU)// ADDR[3:0] - Endpoint Address

  /* EndPoint REGister MASK (no toggle fields) */
  #define USB_EPREG_MASK (USB_EP_CTR_RX | USB_EP_SETUP | USB_EP_T_FIELD | USB_EP_KIND | USB_EP_CTR_TX | USB_EPADDR_FIELD)

  // Endpoint Type Codes
  #define USB_EP_TYPE_MASK ((uint16_t) 0x0600U)  // Endpoint TYPE Mask
  #define USB_EP_BULK ((uint16_t) 0x0000U)       // BULK: Bulk Endpoint
  #define USB_EP_CONTROL ((uint16_t) 0x0200U)    // CONTROL: Control Endpoint
  #define USB_EP_ISOCHRONOUS ((uint16_t) 0x0400U)// ISO: Synchronous Endpoint
  #define USB_EP_INTERRUPT ((uint16_t) 0x0600U)  // INTERRUPT: Interrupt Endpoint
  #define USB_EP_T_MASK ((uint16_t) ~USB_EP_T_FIELD & USB_EPREG_MASK)

  // Endpoint Kind
  #define USB_EPKIND_MASK ((uint16_t) ~USB_EP_KIND & USB_EPREG_MASK)

  // Transmitt State Codes
  #define USB_EP_TX_DIS ((uint16_t) 0x0000U)  // DISABLED: All rx req's are ignored by the endpoint
  #define USB_EP_TX_STALL ((uint16_t) 0x0010U)// STALL: The endpoint responds to all rx req's with STALL packet
  #define USB_EP_TX_NAK ((uint16_t) 0x0020U)  // NAK: The endpoint responds to all rx req's with NAK packet
  #define USB_EP_TX_VALID ((uint16_t) 0x0030U)// VALID: The endpoint can be used for rx
  #define USB_EPTX_DTOG1 ((uint16_t) 0x0010U) // Toggle Bit 1
  #define USB_EPTX_DTOG2 ((uint16_t) 0x0020U) // Toggle Bit 2
  #define USB_EPTX_DTOGMASK (USB_EPTX_STAT | USB_EPREG_MASK)

  //Receive State Codes
  #define USB_EP_RX_DIS ((uint16_t) 0x0000U)  // DISABLED: All tx req's are ignored by the endpoint
  #define USB_EP_RX_STALL ((uint16_t) 0x1000U)// STALL: The endpoint responds to all tx req's with STALL packet
  #define USB_EP_RX_NAK ((uint16_t) 0x2000U)  // NAK: The endpoint responds to all tx req's with NAK packet
  #define USB_EP_RX_VALID ((uint16_t) 0x3000U)// VALID: The endpoint can be used for tx
  #define USB_EPRX_DTOG1 ((uint16_t) 0x1000U) // Toggle Bit 1
  #define USB_EPRX_DTOG2 ((uint16_t) 0x2000U) // Toggle Bit 2
  #define USB_EPRX_DTOGMASK (USB_EPRX_STAT | USB_EPREG_MASK)

#else
  #error You are using an untested or unimplemented APM32 variant. Please update the driver.
#endif

/* -------------------------------------------------------------------------- */
/*                    Peripheral and Packet Memory Adresses                   */
/* -------------------------------------------------------------------------- */
#ifndef FSDEV_REG_BASE
  #if defined(USBD_BASE)
    #define FSDEV_REG_BASE USBD_BASE
  #else
    #error "FSDEV_REG_BASE not defined"
  #endif
#endif

#ifndef FSDEV_PMA_BASE
  #error "FSDEV_PMA_BASE not defined"
#endif

//TODO: Check this
// This checks if the device has "LPM"
#if defined(USB_ISTR_L1REQ)
  #define USB_ISTR_L1REQ_FORCED (USB_ISTR_L1REQ)
#else
  #define USB_ISTR_L1REQ_FORCED ((uint16_t) 0x0000U)
#endif

#define USB_ISTR_ALL_EVENTS (USB_ISTR_PMAOVR | USB_ISTR_ERR | USB_ISTR_WKUP | USB_ISTR_SUSP | \
                             USB_ISTR_RESET | USB_ISTR_SOF | USB_ISTR_ESOF | USB_ISTR_L1REQ_FORCED)

/* -------------------------------------------------------------------------- */


static const IRQn_Type fsdev_irq[] = {
#if TU_CHECK_MCU(OPT_MCU_APM32F0)
    USBD_IRQn,
#else
  #error Unknown arch in USB driver
#endif
};
enum { FSDEV_IRQ_NUM = TU_ARRAY_SIZE(fsdev_irq) };

void dcd_int_enable(uint8_t rhport) {
  (void) rhport;

  // forces write to RAM before allowing ISR to execute
  __DSB();
  __ISB();

  {
    for (uint8_t i = 0; i < FSDEV_IRQ_NUM; i++) {
      NVIC_EnableIRQ(fsdev_irq[i]);
    }
  }
}

void dcd_int_disable(uint8_t rhport) {
  (void) rhport;

  {
    for (uint8_t i = 0; i < FSDEV_IRQ_NUM; i++) {
      NVIC_DisableIRQ(fsdev_irq[i]);
    }
  }
}

// Internal Pullups
#if defined(APM32F072)
void dcd_disconnect(uint8_t rhport) {
  (void) rhport;
  USBD->BCD_B.DPPUCTRL = 0b0;
}

void dcd_connect(uint8_t rhport) {
  (void) rhport;
  USBD->BCD_B.DPPUCTRL = 0b1;
}
#endif


#endif
