/** XUD_USBTile_Support.S
  * @brief     Support functions for USB Tile use
  * @author    Ross Owen, XMOS Limited
  */

#include <xs2a_registers.h>

.text
.globl XUD_SetUSBPortMux, "f{ui}(ui)"
.type  XUD_SetUSBPortMux, @function
.cc_top XUD_SetUSBPortMux.function
.align    4
// void XUD_SetUSBPortMux (unsigned mode);
XUD_SetUSBPortMux:
  ldc       r1, XS1_PS_XCORE_CTRL0
  set       ps[r1], r0

  //Sanity check, read back and check
  //get	    r1, ps[r1]
  //eq        r1, r0, r1
  //ecallf    r1
  retsp     0

.size XUD_SetUSBPortMux, .-XUD_SetUSBPortMux
.cc_bottom XUD_SetUSBPortMux.function
.globl XUD_SetUSBPortMux.nstackwords
.globl XUD_SetUSBPortMux.maxchanends
.globl XUD_SetUSBPortMux.maxtimers
.globl XUD_SetUSBPortMux.maxcores
.set XUD_SetUSBPortMux.nstackwords, 0
.set XUD_SetUSBPortMux.maxchanends, 0
.set XUD_SetUSBPortMux.maxtimers, 0
.set XUD_SetUSBPortMux.maxcores, 1
.set XUD_SetUSBPortMux.locnochandec, 1

