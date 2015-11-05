
#ifndef _XUD_PWRSIG_H_
#define _XUD_PWRSIG_H_

int XUD_Init(
    out buffered port:32 p_usb_txd_, 
    in port flag0_port_, 
    in port flag1_port_, 
    in port flag2_port_
);


int XUD_Suspend(tileref usbtile, XUD_PwrConfig pwrConfig,
    out buffered port:32 p_usb_txd_, 
    in port flag0_port_, 
    in port flag1_port_, 
    in port flag2_port_
);

#endif
