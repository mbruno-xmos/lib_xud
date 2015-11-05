#ifndef _XUD_USB_DEFINES_H_
#define _XUD_USB_DEFINES_H_

// Defines relating to USB/ULPI/UTMI/Phy specs

#define SUSPEND_TIMEOUT_us          3000 
#define SUSPEND_TIMEOUT             (SUSPEND_TIMEOUT_us * REF_CLK_FREQ)

// Device attach timing defines
#define T_SIGATT_ULPI_us            5000     // 5ms
#define T_SIGATT_ULPI               (T_SIGATT_ULPI_us * REF_CLK_FREQ)
#define T_ATTDB_us                  1000000  // 1000ms
#define T_ATTDB                     (T_ATTDB_us * REF_CLK_FREQ)
#define T_UCHEND_T_UCH_us           1000000  // 1000ms
#define T_UCHEND_T_UCH              (T_UCHEND_T_UCH_us * REF_CLK_FREQ)
#define T_UCHEND_T_UCH_ULPI_us      2000     //    2ms
#define T_UCHEND_T_UCH_ULPI         (T_UCHEND_T_UCH_us * REF_CLK_FREQ)
#define T_FILT_us                   3       //   2.5us
#define T_FILT                      (T_FILT_us * REF_CLK_FREQ)

#define SUSPEND_T_WTWRSTHS_us       200     // 200us Time beforechecking for J after asserting XcvrSelect and Termselect
#define SUSPEND_T_WTWRSTHS          (SUSPEND_T_WTWRSTHS_us * REF_CLK_FREQ)

#define OUT_TIMEOUT_us              500     // How long we wait for data after OUT token
#define OUT_TIMEOUT                 (OUT_TIMEOUT_us * REF_CLK_FREQ)
#define TX_HANDSHAKE_TIMEOUT_us     5      // How long we wait for handshake after sending tx data
#define TX_HANDSHAKE_TIMEOUT        (TX_HANDSHAKE_TIMEOUT_us * REF_CLK_FREQ)

// Raw PIDs
#define PID_OUT                     0x01
#define PID_ACK                     0x02
#define PID_DATA0                   0x03
#define PID_PING                    0x04
#define PID_SOF                     0x05
#define PID_IN                      0x09
#define PID_DATA1                   0x0b
#define PID_SETUP                   0x0d

// PIDs with error check
#define PIDn_OUT                    0xe1
#define PIDn_ACK                    0xd2
#define PIDn_DATA0                  0xc3
#define PIDn_SOF                    0xa5
#define PIDn_IN                     0x69
#define PIDn_NAK                    0x5a
#define PIDn_DATA1                  0x4b
#define PIDn_SETUP                  0x2d
#define PIDn_STALL                  0x1e

// Test selector defines for Test mode
#define WINDEX_TEST_J               (0x1<<8)
#define WINDEX_TEST_K               (0x2<<8)
#define WINDEX_TEST_SE0_NAK         (0x3<<8)
#define WINDEX_TEST_PACKET          (0x4<<8)
#define WINDEX_TEST_FORCE_ENABLE    (0x5<<8)

#endif
