
/** @brief Does high speed device attach
  * @return non-zero for error
  **/
int XUD_DeviceAttachHS(tileref usbtile, XUD_PwrConfig pwrConfig,
    out buffered port:32 p_usb_txd_, 
    in port flag0_port_, 
    in port flag1_port_, 
    in port flag2_port_
);
