
#ifdef LSERIES
extern out port p_usb_txd;
extern port p_usb_rxd;
extern out port p_rst;
extern in buffered port:32 p_usb_clk;
#else
extern out buffered port:32 p_usb_txd;
extern in  buffered port:32 p_usb_rxd;
#endif
