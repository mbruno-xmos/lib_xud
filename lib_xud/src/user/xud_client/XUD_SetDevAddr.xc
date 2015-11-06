
#include <xs2_su_registers.h>
#include "XUD_USB_Defines.h"
#include "XUD_USBTile_Support.h"
#include "xud.h"

/** @brief  Sets the device address in XUD
  * @parap  usbtile USB tile tileref
  * @param  addr the new address
  */
unsafe XUD_Result_t XUD_SetDevAddr(tileref usbtile, unsigned addr)
{
    unsigned data;
    
    /* Set device address in USB tile */
    write_periph_word(usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM, XS1_GLX_PER_UIFM_DEVICE_ADDRESS_NUM, addr);

    return XUD_RES_OKAY;
}
