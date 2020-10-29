#include <xs1.h>
#include <platform.h>
#include <print.h>
#include "setup.h"

#include "xud.h"
#include "XUD_HAL.h"

void XUD_ConfigUsbPorts(); // Not part of lib_xud api

extern in  buffered port:32 p_usb_clk;

//int data[256];
int data_bounce[256];

#ifdef FAST
#warning "FAST"
extern int rcvInit();
extern void sendPacketVarFast(out buffered port:32 txd, int ptr[]);
extern int rcvPacketVarTimeout(in buffered port:32 rxd, int ptr[]);
#else
extern void sendPacketVar(out buffered port:32 txd, int ptr[]);
extern int rcvPacketVar(int length, in buffered port:32 rxd, int ptr[]);
#endif

#ifdef XKUSBAUDIO
out port USB_SEL = XS1_PORT_32A;
#endif

#ifdef XK_216_MC
on tile[0]: out port p_USB_SEL = XS1_PORT_8C;

void hw_setup_xk_216_mc(void){
  p_USB_SEL <: 0x18; //Enable VBUS and USB A socket
  while(1){ //Stop ports turning off!
  }
}
#endif


int mytimer;

int do_test(void) {
    timer tmr;
    unsigned t;

#ifdef XKUSBAUDIO
    USB_SEL <: 0x60;
#endif 

    XUD_ConfigUsbPorts();
    
    XUD_HAL_EnableUsb(XUD_PWR_BUS);
    
    XUD_HAL_EnterMode_PeripheralHighSpeed();
    
	clearbuf(p_usb_rxd);
  
	printstr("Bouncer started\n");

    int led_value = 1;    // All leds on

#ifdef FAST
    rcvInit();
    tmr :> t;
    tmr when timerafter(t+10000) :> int _;
#endif

    while(1) {
#ifndef XKUSBAUDIO
        //leds <: led_value | 8; // Led 2 off on receive
#endif
        clearbuf(p_usb_rxd);

#ifdef FAST
        while (rcvPacketVarTimeout(p_usb_rxd, data_bounce) != 0)
          {}
#else
        rcvPacketVar(0, p_usb_rxd, data_bounce); 
#endif
#ifndef XKUSBAUDIO
        //leds <: led_value | 2; // Led 0 off on send
#endif

        for(int i = 0; i < BOUNCECOUNT; i++)
        {
            tmr :> t;
            t += 10000;
            tmr when timerafter(t) :> void;       // wait 1 us before bouncing back
#ifdef FAST
            sendPacketVarFast(p_usb_txd, data_bounce);
#else
            sendPacketVar(p_usb_txd, data_bounce); 
#endif
            led_value ^= 1;  // modulate led 1 on/off on every packet
        }
    }
    return 0;
}


#ifdef BOUNCER
int main(void){
  par{
#ifdef XK_216_MC
    on tile[1]: do_test();
    on tile[0]: hw_setup_xk_216_mc();
#else
    do_test();
#endif
  }
  return 0;
}
#endif
