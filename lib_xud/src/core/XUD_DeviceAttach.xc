#include <xs1.h>
#include <xs2_su_registers.h>
#include <print.h>
#include <platform.h>
#include "XUD_USB_Defines.h"
#include "XUD_Support.h"
#include "xud.h"
#include "XUD_USBTile_Support.h"

#define TUCHEND_DELAY_us   1500 // 1.5ms
#define TUCHEND_DELAY      (TUCHEND_DELAY_us * REF_CLK_FREQ)
#define INVALID_DELAY_us   2500 // 2.5ms
#define INVALID_DELAY      (INVALID_DELAY_us * REF_CLK_FREQ)

extern int resetCount;

/* Assumptions:
 * - In full speed mode
 * - No flags sticky
 * - Flag 0 port inverted
 */
int XUD_DeviceAttachHS(tileref usbtile, XUD_PwrConfig pwrConfig,
    out buffered port:32 p_usb_txd_, 
    in port flag0_port_, 
    in port flag1_port_, 
    in port flag2_port_
)
{
    unsigned tmp;
    timer t;
    int start_time;
    int detecting_k = 1;
    int tx;
    int chirpCount = 0;

    clearbuf(p_usb_txd_);
   
    // On detecting the SE0:
    // De-assert XCVRSelect and set opmode=2
    // DEBUG - write to ulpi reg 0x54. This is:
    // opmode = 0b10, termsel = 1, xcvrsel = 0b00;
    write_periph_word(usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM, XS1_GLX_PER_UIFM_FUNC_CONTROL_NUM, 0b1010);
    
    XUD_Sup_Delay(10000);
   
    /* Output k-chirp for required time */
    for (int i = 0; i < 16000; i++) 
    {   
        /* 16000 words @ 480 MBit = 1.066 ms */
        p_usb_txd_<: 0;
    }
    
    // J, K, SE0 on flag ports 0, 1, 2 respectively
    // Wait for fs chirp k (i.e. HS chirp j)
    flag1_port_ when pinseq(0) :> tmp; // Wait for out k to go

    t :> start_time;
   
    while(1) 
    {
        select 
        {
            case t when timerafter(start_time + INVALID_DELAY) :> void:
                
                /* Go into full speed mode: XcvrSelect and Term Select (and suspend) high */
                write_periph_word(usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM,
                             XS1_GLX_PER_UIFM_FUNC_CONTROL_NUM,
                             (1<<XS1_UIFM_FUNC_CONTROL_XCVRSELECT_SHIFT)
                              | (1<<XS1_UIFM_FUNC_CONTROL_TERMSELECT_SHIFT));

                /* Wait for SE0 end */
                while(1) 
                {
                    /* TODO Use a timer to save some juice...*/
                    flag2_port_ :> tmp;

                    if(!tmp) 
                    {
                        return 0;                /* SE0 gone, return 0 to indicate FULL SPEED */
                    }

                    if(pwrConfig == XUD_PWR_SELF)  
                    {
                        unsigned x;
                         
                        read_periph_word(usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM,
                                    XS1_GLX_PER_UIFM_OTG_FLAGS_NUM, x);
                        if(!(x&(1<<XS1_UIFM_OTG_FLAGS_SESSVLDB_SHIFT)))
                        {
                            write_periph_word(usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM,
                                         XS1_GLX_PER_UIFM_FUNC_CONTROL_NUM, 4);
                            return -1;             // VBUS gone, handshake fails completely.
                        }
                    }
                }
                break;

            case detecting_k => flag1_port_ when pinseq(1):> void @ tx:          // K Chirp
                flag1_port_ @ tx + T_FILT :> tmp;
                if (tmp) 
                {
                    detecting_k = 0;
                }
                break;
       
            case !detecting_k => flag0_port_ when pinseq(0) :> void @ tx:     // J Chirp, inverted!
                flag0_port_ @ tx + T_FILT :> tmp;
                if (tmp == 0) 
                {                                              // inverted!
                    chirpCount ++;                                              // Seen an extra K-J pair
                    detecting_k = 1;
                    if (chirpCount == 3) 
                    {                                      // On 3 we have seen a HS

                        // Three pairs of KJ received... de-assert TermSelect...
                        // (and opmode = 0, suspendm = 1)
                        write_periph_word(usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM, XS1_GLX_PER_UIFM_FUNC_CONTROL_NUM, 0b0000);
                   
                        //wait for SE0 (TODO consume other chirps?)
                        flag2_port_ when pinseq(1) :> tmp;
                        return 1;                                               // Return 1 for HS
                    }
                }
                break;
        }
    }
}

