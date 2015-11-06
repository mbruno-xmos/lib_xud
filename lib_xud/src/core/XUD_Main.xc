
/** XUD_Manager.xc
  * @brief     XMOS USB Device(XUD) Layer
  * @author    Ross Owen
  **/

#include <xs1.h>
#include <print.h>
#include <xclib.h>
#include <platform.h>
#include <xs2_su_registers.h>
#include "xud.h"                 /* External user include file */
#include "XUD_USB_Defines.h"
#include "XUD_USBTile_Support.h"
#include "XUD_Support.h"
#include "XUD_DeviceAttach.h"
#include "XUD_PowerSig.h"

/* Location to store stack pointer (required for interupt handler) */
unsigned SavedSp;

#if (USB_MAX_NUM_EP_IN != 16)
#error USB_MAX_NUM_EP_IN must be 16!
#endif

#if (USB_MAX_NUM_EP_OUT != 16)
#error USB_MAX_NUM_EP_OUT must be 16!
#endif

/* User hooks */
void XUD_UserSuspend();
void XUD_UserResume();

/* Timeout differences due to using 60MHz vs 100MHz */
#if !defined(ARCH_S) && !defined(ARCH_X200)
#define HS_TX_HANDSHAKE_TIMEOUT 100
#define FS_TX_HANDSHAKE_TIMEOUT 3000
#else
#define HS_TX_HANDSHAKE_TIMEOUT (167)
#define FS_TX_HANDSHAKE_TIMEOUT (5000)
#endif

/* Global vars for current and desired USB speed */
unsigned g_curSpeed;
unsigned g_desSpeedVal;
unsafe
{
    unsigned * unsafe g_desSpeed = &g_desSpeedVal;
}
unsigned g_txHandshakeTimeout;

/* USB Port declarations */
extern out port tx_readyout; // aka txvalid
extern in port tx_readyin;
extern out buffered port:32 p_usb_txd;
extern in buffered port:32 p_usb_rxd;
extern in port rx_rdy;
extern in port flag0_port;
extern in port flag1_port;
extern in port flag2_port;
extern in buffered port:32 p_usb_clk;
extern clock tx_usb_clk;
extern clock rx_usb_clk;

unsafe
{
    out port * unsafe tx_readyout_ = &tx_readyout; // aka txvalid
    in port * unsafe tx_readyin_ = &tx_readyin;
    out buffered port:32 * unsafe p_usb_txd_ = &p_usb_txd;
    in buffered port:32 * unsafe p_usb_rxd_ = &p_usb_rxd;
    in port * unsafe rx_rdy_ = &rx_rdy;
    in port * unsafe flag0_port_ = &flag0_port;
    in port * unsafe flag1_port_ = &flag1_port;
    in port * unsafe flag2_port_ = &flag2_port;
    in buffered port:32 * unsafe p_usb_clk_ = &p_usb_clk;
    clock * unsafe rx_usb_clk_ = &rx_usb_clk;
    clock * unsafe tx_usb_clk_ = &tx_usb_clk;
}

/* TODO pack this to save mem
 * TODO size of this hardcoded in ResetRpStateByAddr_
 */
typedef struct XUD_ep_info
{
    unsigned int chan_array_ptr;       // 0
    unsigned int ep_xud_chanend;       // 1
    unsigned int ep_client_chanend;    // 2
    unsigned int scratch;              // 3 used for datalength in
    unsigned int pid;                  // 4 Expected out PID
    unsigned int epType;               // 5 Data
    unsigned int actualPid;            // 6 Actual OUT PID received for OUT, Length (words) for IN.
    unsigned int tailLength;           // 7 "tail" length for IN (bytes)
    unsigned int epAddress;            // 8 EP address assigned by XUD (Used for marking stall etc)
    unsigned int resetting;            // 9 Flag to indicate to EP a bus-reset occured.
} XUD_ep_info;

XUD_chan epChansArray[USB_MAX_NUM_EP];
XUD_chan epChans0Array[USB_MAX_NUM_EP];
XUD_ep_info ep_info[USB_MAX_NUM_EP];

unsafe
{
    XUD_ep_info * unsafe ep_infop = ep_info;
    XUD_chan * unsafe epChans = epChansArray;
    XUD_chan * unsafe epChans0 = epChans0Array;
}

/* Sets the UIFM flags into a mode suitable for power signalling */
unsafe void XUD_UIFM_PwrSigFlags(tileref * unsafe usbtile)
{
    write_periph_word(*usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM, XS1_GLX_PER_UIFM_MASK_NUM, ((1<<XS1_UIFM_IFM_FLAGS_SE0_SHIFT)<<16)
        | ((1<<XS1_UIFM_IFM_FLAGS_K_SHIFT)<<8) | (1 << XS1_UIFM_IFM_FLAGS_J_SHIFT));
}

/* Tables storing if EP's are signed up to bus state updates */
int epStatFlagTableInArray[USB_MAX_NUM_EP_IN];
int epStatFlagTableOutArray[USB_MAX_NUM_EP_OUT];

unsafe
{
    int * unsafe epStatFlagTableIn = epStatFlagTableInArray;
    int * unsafe epStatFlagTableOut = epStatFlagTableOutArray;
}

/* Used for terminating XUD loop */
int XUD_USB_Done = 0;

extern int XUD_LLD_IoLoop(
#if defined(ARCH_S) || defined(ARCH_X200)
                            in buffered port:32 rxd_port,
#else
                            in port rxd_port,
#endif
                            in port rxa_port,
#if defined(ARCH_S) || defined(ARCH_X200)
                            out buffered port:32 txd_port,
#else
                            out port txd_port,
#endif
                            in port rxe_port, in port flag0_port,
                            in port ?read, out port ?write, int x,
                            XUD_EpType epTypeTableOut[], XUD_EpType epTypeTableIn[], XUD_chan * unsafe epChans,
                            int epCount, chanend? c_sof) ;

unsigned handshakeTable_IN_a[USB_MAX_NUM_EP_IN];
unsigned handshakeTable_OUT_a[USB_MAX_NUM_EP_OUT];

unsigned sentResetVal = 0;

unsafe
{
    unsigned * unsafe sentReset = &sentResetVal;
    unsigned * unsafe handshakeTable_IN = handshakeTable_IN_a;
    unsigned * unsafe handshakeTable_OUT = handshakeTable_OUT_a;
}

unsigned chanArray;
static int oneval = 1;

unsafe
{
    int * unsafe one = &oneval;
}

#pragma unsafe arrays
static void SendResetToEps(XUD_chan * unsafe c, XUD_chan * unsafe epChans, XUD_EpType epTypeTableOut[], XUD_EpType epTypeTableIn[], int nOut, int nIn, int token)
{
    unsafe
    {
    for(int i = 0; i < nOut; i++)
    {
        unsigned val;
        asm volatile ("ldw %0, %1[%2]":"=r"(val):"r"(epStatFlagTableOut),"r"(i));
        //if(epTypeTableOut[i] != XUD_EPTYPE_DIS && epStatFlagTableOut[i])
        if(epTypeTableOut[i] != XUD_EPTYPE_DIS && val)
        {
            /* Set EP resetting flag. EP uses this to check if it missed a reset before setting ready */
            ep_infop[i].resetting = 1;

            /* Clear EP ready. Note. small race since EP might set ready after XUD sets resetting to 1
             * but this should be caught in time (EP gets CT) */
            epChans[i] = 0;
            XUD_Sup_outct(c[i], token);
        }
    }
    for(int i = 0; i < nIn; i++)
    {
        if(epTypeTableIn[i] != XUD_EPTYPE_DIS && epStatFlagTableIn[i])
        {
            ep_infop[i + USB_MAX_NUM_EP_OUT].resetting = 1;
            epChans[i + USB_MAX_NUM_EP_OUT] = 0;
            XUD_Sup_outct(c[i + USB_MAX_NUM_EP_OUT], token);
        }
    }

    }


    /* Not longer drain channels or recive CT from EP - this is because EPS's no longer use channels to indicate ready status */
#if 0
    for(int i = 0; i < nOut; i++)
    {
        if(epTypeTableOut[i] != XUD_EPTYPE_DIS && epStatFlagTableOut[i])
        {
            while(!XUD_Sup_testct(c[i]))
            {
                XUD_Sup_int(c[i]);
            }
            XUD_Sup_inct(c[i]);       // TODO chkct

            /* Clear EP ready. Note, done after inct to avoid race with EP */
            eps[i] = 0;
        }
    }
    for(int i = 0; i < nIn; i++)
    {
        if(epTypeTableIn[i] != XUD_EPTYPE_DIS && epStatFlagTableIn[i])
        {
          int tok=-1;
          while (tok != XS1_CT_END) {
            while(!XUD_Sup_testct(c[i + USB_MAX_NUM_EP_OUT]))
            {
                XUD_Sup_int(c[i + USB_MAX_NUM_EP_OUT]);
            }
            tok = XUD_Sup_inct(c[i + USB_MAX_NUM_EP_OUT]);       // TODO chkct

            /* Clear EP ready. Note, done after inct to avoid race with EP */
            eps[i + USB_MAX_NUM_EP_OUT] = 0;
          }
        }
    }
#endif
}

static void SendSpeed(XUD_chan * unsafe c, XUD_EpType epTypeTableOut[], XUD_EpType epTypeTableIn[], int nOut, int nIn, int speed)
{
    unsafe
    {
    for(int i = 0; i < nOut; i++)
    {
        if(epTypeTableOut[i] != XUD_EPTYPE_DIS && epStatFlagTableOut[i])
        {
            XUD_Sup_outuint(c[i], speed);
        }
    }
    for(int i = 0; i < nIn; i++)
    {
        if(epTypeTableIn[i] != XUD_EPTYPE_DIS && epStatFlagTableIn[i])
        {
            XUD_Sup_outuint(c[i + USB_MAX_NUM_EP_OUT], speed);
        }
    }
    }
}

void XUD_ULPIReg(out port p_usb_txd);

// Main XUD loop
unsafe int XUD_Manager_loop(tileref * unsafe usbtile, XUD_chan * unsafe epChans0, XUD_chan * unsafe epChans,  chanend ?c_sof, XUD_EpType epTypeTableOut[], XUD_EpType epTypeTableIn[], int noEpOut, int noEpIn, out port ?p_rst, unsigned rstMask, clock ?clk, XUD_PwrConfig pwrConfig)
{
    int reset = 1;            /* Flag for if device is returning from a reset */

    /* Make sure ports are on and reset port states */
    set_port_use_on(*p_usb_clk_);
    set_port_use_on(*p_usb_txd_);
    set_port_use_on(*p_usb_rxd_);
    set_port_use_on(*flag0_port_);
    set_port_use_on(*flag1_port_);
    set_port_use_on(*flag2_port_);

#define TX_RISE_DELAY 5
#warning check this for U series (was 2)
#define TX_FALL_DELAY 1
#define RX_RISE_DELAY 5
#define RX_FALL_DELAY 5

    // Handshaken ports need USB clock
    configure_clock_src (*tx_usb_clk_, *p_usb_clk_);
    configure_clock_src (*rx_usb_clk_, *p_usb_clk_);

    //this along with the following delays forces the clock
    //to the ports to be effectively controlled by the
    //previous usb clock edges
    set_port_inv(*p_usb_clk_);
    set_port_sample_delay(*p_usb_clk_);

    //This delay controls the capture of rdy
    set_clock_rise_delay(*tx_usb_clk_, TX_RISE_DELAY);

    //this delay controls the launch of data.
    set_clock_fall_delay(*tx_usb_clk_, TX_FALL_DELAY);

    //this delay the capture of the rdyIn and data.
    set_clock_rise_delay(*rx_usb_clk_, RX_RISE_DELAY);
    set_clock_fall_delay(*rx_usb_clk_, RX_FALL_DELAY);

  	set_port_inv(*flag0_port_);
  	
    start_clock(*tx_usb_clk_);
  	start_clock(*rx_usb_clk_);
 	
    configure_out_port_handshake(*p_usb_txd_, *tx_readyin_, *tx_readyout_, *tx_usb_clk_, 0);
  	configure_in_port_strobed_slave(*p_usb_rxd_, *rx_rdy_, *rx_usb_clk_);

    while(1)
    {
#if !defined(ARCH_S) && !defined(ARCH_X200)
        p_usb_rxd <: 0;         // Note, this is important else phy clocks in invalid data before UIFM is enabled causing
        clearbuf(p_usb_rxd);    // connection issues
#endif

#ifndef SIMULATION
        unsigned settings[] = {0};

        /* For xCORE-200 enable USB port muxing before enabling phy etc */
        XUD_EnableUsbPortMux(); //setps(XS1_PS_XCORE_CTRL0, UIFM_MODE);

        /* Enable the USB clock */
        write_sswitch_reg(get_tile_id(*usbtile), XS1_GLX_CFG_RST_MISC_NUM, ( 1 << XS1_GLX_CFG_USB_CLK_EN_SHIFT));

        /* Take phy out of reset */
        write_periph_word(*usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM, XS1_GLX_PER_UIFM_PHY_CONTROL_NUM,  0);/*(0<<XS1_UIFM_PHY_CONTROL_FORCERESET));*/
        
        /* Keep usb clock active, enter active mode */
        write_sswitch_reg(get_tile_id(*usbtile), XS1_GLX_CFG_RST_MISC_NUM, (1 << XS1_GLX_CFG_USB_CLK_EN_SHIFT) | (1<<XS1_GLX_CFG_USB_EN_SHIFT)  );

        /* Clear OTG control reg - incase we were running as host previously.. */
        write_periph_word(*usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM, XS1_GLX_PER_UIFM_OTG_CONTROL_NUM, 0);
#endif

        /* Wait for USB clock (typically 1ms after reset) */
        *p_usb_clk_ when pinseq(1) :> int _;
        *p_usb_clk_ when pinseq(0) :> int _;
        *p_usb_clk_ when pinseq(1) :> int _;
        *p_usb_clk_ when pinseq(0) :> int _;

        /* Enable linestate decode in USB tile */
        write_periph_word(*usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM, XS1_GLX_PER_UIFM_CONTROL_NUM, (1<<XS1_UIFM_IFM_CONTROL_DECODELINESTATE_SHIFT));

        while(1)
        {
            {
                /* Wait for VBUS before enabling pull-up. The USB Spec (page 150) allows 100ms
                 * between vbus valid and signalling attach */
                if(pwrConfig == XUD_PWR_SELF)
                {
                    while(1)
                    {
                        unsigned x;
                        read_periph_word(*usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM, XS1_GLX_PER_UIFM_OTG_FLAGS_NUM, x);
                        if(x & (1<<XS1_UIFM_OTG_FLAGS_SESSVLDB_SHIFT))
                        {
                            break;
                        }
                        XUD_Sup_Delay(200 * REF_CLK_FREQ); // 200us poll
                    }
                }
#ifndef SIMULATION
                /* Go into full speed mode: XcvrSelect and Term Select (and suspend) high */
                write_periph_word(*usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM, XS1_GLX_PER_UIFM_FUNC_CONTROL_NUM,
                      (1<<XS1_UIFM_FUNC_CONTROL_XCVRSELECT_SHIFT)
                    | (1<<XS1_UIFM_FUNC_CONTROL_TERMSELECT_SHIFT));
#endif  /* SIMULATION */

#ifdef SIMULATION
                reset = 1;
#else
                /* Setup flags for power signalling - J/K/SE0 line state*/
                XUD_UIFM_PwrSigFlags(usbtile);

                if (*one)
                {
                    /* Set flags up for pwr signalling */
                    reset = XUD_Init(*p_usb_txd_, *flag0_port_, *flag1_port_, *flag2_port_);
                    *one = 0;
                }
                else
                {
                    XUD_Sup_Delay(20000); // T_WTRSTHS: 100-875us

                    /* Sample line state and check for reset (or suspend) */
                    *flag2_port_ :> reset; /* SE0 Line */
                }

#endif
                /* Inspect for suspend or reset */
                if(!reset)
                {
                    /* Run user suspend code */
                    XUD_UserSuspend();

                    /* Run suspend code, returns 1 if reset from suspend, 0 for resume, -1 for invalid vbus */
                    reset = XUD_Suspend(*usbtile, pwrConfig, *p_usb_txd_, *flag0_port_, *flag1_port_, *flag2_port_);
                    
                    if((pwrConfig == XUD_PWR_SELF) && (reset==-1))
                    {
                        /* Lost VBUS */
                        continue;
                    }

                    /* Run user resume code */
                    XUD_UserResume();
                }

                /* Test if coming back from reset or suspend */
                if(reset==1)
                {
                    if(!*sentReset)
                    {
                        SendResetToEps(epChans0, epChans, epTypeTableOut, epTypeTableIn, noEpOut, noEpIn, USB_RESET_TOKEN);
                        *sentReset = 1;
                    }

                    /* Reset the OUT ep structures */
                    for(int i = 0; i< noEpOut; i++)
                    {
                        ep_infop[i].pid = USB_PID_DATA0;
                    }

                    /* Reset in the ep structures */
                    for(int i = 0; i< noEpIn; i++)
                    {
                        ep_infop[USB_MAX_NUM_EP_OUT+i].pid = USB_PIDn_DATA0;
                    }

                    /* Set default device address */
#ifndef SIMULATION
                    write_periph_word(*usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM, XS1_GLX_PER_UIFM_DEVICE_ADDRESS_NUM, 0);
#endif

#ifdef SIMULATION
                    if(*g_desSpeed == XUD_SPEED_HS)
                    {
                        //g_curSpeed = XUD_SPEED_HS;
                        //g_txHandshakeTimeout = HS_TX_HANDSHAKE_TIMEOUT;
                        asm volatile("stw %0, dp[g_curSpeed]"::"r"(XUD_SPEED_HS):"memory");
                        asm volatile("stw %0, dp[g_txHandshakeTimeout]"::"r"(HS_TX_HANDSHAKE_TIMEOUT):"memory");
                    }
                    else
                    {
                        //g_curSpeed = XUD_SPEED_FS;
                        //g_txHandshakeTimeout = FS_TX_HANDSHAKE_TIMEOUT;
                        asm volatile("stw %0, dp[g_curSpeed]"::"r"(XUD_SPEED_FS):"memory");
                        asm volatile("stw %0, dp[g_txHandshakeTimeout]"::"r"(FS_TX_HANDSHAKE_TIMEOUT):"memory");
                    }
#else
                    if(*g_desSpeed == XUD_SPEED_HS)
                    {
                        unsigned tmp;
                        tmp = XUD_DeviceAttachHS(*usbtile, pwrConfig, *p_usb_txd_, *flag0_port_, *flag1_port_, *flag2_port_);

                        if(tmp == -1)
                        {
                            XUD_UserSuspend();
                            continue;
                        }
                        else if (!tmp)
                        {
                            /* HS handshake fail, mark as running in FS */
                            //g_curSpeed = XUD_SPEED_FS;
                            //g_txHandshakeTimeout = FS_TX_HANDSHAKE_TIMEOUT;
                            asm volatile("stw %0, dp[g_curSpeed]"::"r"(XUD_SPEED_FS):"memory");
                            asm volatile("stw %0, dp[g_txHandshakeTimeout]"::"r"(FS_TX_HANDSHAKE_TIMEOUT):"memory");
                        }
                        else
                        {
                            //g_curSpeed = XUD_SPEED_HS;
                            //g_txHandshakeTimeout = HS_TX_HANDSHAKE_TIMEOUT;
                            asm volatile("stw %0, dp[g_curSpeed]"::"r"(XUD_SPEED_HS):"memory");
                            asm volatile("stw %0, dp[g_txHandshakeTimeout]"::"r"(HS_TX_HANDSHAKE_TIMEOUT):"memory");
                        }
                    }
                    else
                    {
                        //g_curSpeed = XUD_SPEED_FS;
                        //g_txHandshakeTimeout = FS_TX_HANDSHAKE_TIMEOUT;
                        asm volatile("stw %0, dp[g_curSpeed]"::"r"(XUD_SPEED_FS):"memory");
                        asm volatile("stw %0, dp[g_txHandshakeTimeout]"::"r"(FS_TX_HANDSHAKE_TIMEOUT):"memory");
                    }
#endif

                    /* Send speed to EPs */
                    SendSpeed(epChans0, epTypeTableOut, epTypeTableIn, noEpOut, noEpIn, g_curSpeed);
                    *sentReset=0;
                }
            }


            /* Set UIFM to CHECK TOKENS mode and enable LINESTATE_DECODE
            NOTE: Need to do this every iteration since CHKTOK would break power signaling */
#ifndef SIMULATION
            write_periph_word(*usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM, XS1_GLX_PER_UIFM_CONTROL_NUM, (1<<XS1_UIFM_IFM_CONTROL_DOTOKENS_SHIFT)
                | (1<< XS1_UIFM_IFM_CONTROL_CHECKTOKENS_SHIFT)
                | (1<< XS1_UIFM_IFM_CONTROL_DECODELINESTATE_SHIFT)
                | (1<< XS1_UIFM_IFM_CONTROL_SOFISTOKEN_SHIFT));

#ifndef SIMULATION
            write_periph_word(*usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM, XS1_GLX_PER_UIFM_MASK_NUM,
                ((1<<XS1_UIFM_IFM_FLAGS_NEWTOKEN_SHIFT)
                | ((1<<XS1_UIFM_IFM_FLAGS_RXACTIVE_SHIFT)<<8)
                | ((1<<XS1_UIFM_IFM_FLAGS_RXERROR_SHIFT)<<16)));
#endif

            set_thread_fast_mode_on();

            /* Run main IO loop
                flag0: Valid token flag
                flag1: Rx Active
                flag2: Rx Error */
#if defined(ARCH_L) && defined(ARCH_G)
            XUD_LLD_IoLoop(*p_usb_rxd_, *flag1_port_, *p_usb_txd_, *flag2_port_, *flag0_port_, *reg_read_port_,
                           *reg_write_port_, 0, epTypeTableOut, epTypeTableIn, epChans, noEpOut, c_sof);
#else
            XUD_LLD_IoLoop(*p_usb_rxd_, *flag1_port_, *p_usb_txd_, *flag2_port_, *flag0_port_, null,
                            null, 0, epTypeTableOut, epTypeTableIn, epChans, noEpOut, c_sof);
#endif

            set_thread_fast_mode_off();

            /* Put UIFM back to default state */
            // write_periph_word(usbtile, XS1_GLX_PER_UIFM_CHANEND_NUM, XS1_UIFM_IFM_CONTROL_REG,
                //(1<<XS1_UIFM_IFM_CONTROL_DOTOKENS) |
                //(1<< XS1_UIFM_IFM_CONTROL_CHECKTOKENS) |
                //(1<< XS1_UIFM_IFM_CONTROL_DECODELINESTATE));
                //(1<< XS1_UIFM_IFM_CONTROL_SOFISTOKEN));
#endif

        }
    }

    /* Turn ports off */
    set_port_use_off(*p_usb_txd_);
    set_port_use_off(*p_usb_rxd_);
    set_port_use_off(*flag0_port_);
    set_port_use_off(*flag1_port_);
    set_port_use_off(*flag2_port_);
    set_port_use_off(*p_usb_clk_);
    return 0;
}

void _userTrapHandleRegister(void);

#pragma unsafe arrays
static void drain(chanend chans[], int n, int op, XUD_EpType epTypeTable[]) {
    for(int i = 0; i < n; i++) {
        if(epTypeTable[i] != XUD_EPTYPE_DIS) {
            switch(op) {
            case 0:
                outct(chans[i], XS1_CT_END);
                break;
            case 1:
                while (!testct(chans[i]))
                    inuchar(chans[i]);
                break;
            case 2:
                chkct(chans[i], XS1_CT_END);
                break;
            }
        }
    }
}


//#pragma unsafe arrays
int XUD_Manager(tileref * unsafe usbtile, chanend c_ep_out[], int noEpOut,
                chanend c_ep_in[], int noEpIn,
                chanend ?c_sof,
                XUD_EpType epTypeTableOut[], XUD_EpType epTypeTableIn[],
                out port ?p_rst, clock ?clk, unsigned rstMask, XUD_BusSpeed_t speed, XUD_PwrConfig pwrConfig)
{
    /* Arrays for channels... */
    /* TODO use two arrays? */

    unsafe
    {

    *g_desSpeed = speed;

    for (int i=0; i < USB_MAX_NUM_EP;i++)
    {
        epChans[i] = 0;
    }

#warning !!!! ADD BACK IN HANDSHAKE
    for(int i = 0; i < USB_MAX_NUM_EP_OUT; i++)
    {
        handshakeTable_OUT[i] = USB_PIDn_NAK;
        ep_infop[i].epAddress = i;
        ep_infop[i].resetting = 0;
    }

    for(int i = 0; i < USB_MAX_NUM_EP_IN; i++)
    {
        handshakeTable_IN[i] = USB_PIDn_NAK;
        ep_infop[USB_MAX_NUM_EP_OUT+i].epAddress = (i | 0x80);
        ep_infop[USB_MAX_NUM_EP_OUT+i].resetting = 0;
    }

    /* Populate arrays of channels and status flag tabes */
    for(int i = 0; i < noEpOut; i++)
    {
      unsigned x;
      epChans0[i] = XUD_Sup_GetResourceId(c_ep_out[i]);

      asm("ldaw %0, %1[%2]":"=r"(x):"r"(epChans),"r"(i));

      ep_infop[i].chan_array_ptr = x;
      

      asm("mov %0, %1":"=r"(x):"r"(c_ep_out[i]));
      ep_infop[i].ep_xud_chanend = x;

	  asm("getd %0, res[%1]":"=r"(x):"r"(c_ep_out[i]));
      ep_infop[i].ep_client_chanend = x;

	  //asm("ldaw %0, %1[%2]":"=r"(x):"r"(ep_info),"r"(i*sizeof(XUD_ep_info)/sizeof(unsigned)));
      x = (unsigned) ep_infop + (i * sizeof(XUD_ep_info));
      outuint(c_ep_out[i], x);

      epStatFlagTableOut[i] = epTypeTableOut[i] & XUD_STATUS_ENABLE;
      //asm volatile("stw %0, %1[%2]"::"r"(epTypeTableOut[i] & XUD_STATUS_ENABLE),"r"(epTypeTableOut),"r"(i));

      epTypeTableOut[i] = epTypeTableOut[i] & 0x7FFFFFFF;

      ep_infop[i].epType = epTypeTableOut[i];

      ep_infop[i].pid = USB_PID_DATA0;
     // ep_infop[i].epAddress = i;

    }

    for(int i = 0; i< noEpIn; i++)
    {
        int x;
        epChans0[i+USB_MAX_NUM_EP_OUT] = XUD_Sup_GetResourceId(c_ep_in[i]);

        asm("ldaw %0, %1[%2]":"=r"(x):"r"(epChans),"r"(USB_MAX_NUM_EP_OUT+i));
        ep_infop[USB_MAX_NUM_EP_OUT+i].chan_array_ptr = x;

        asm("mov %0, %1":"=r"(x):"r"(c_ep_in[i]));
        ep_infop[USB_MAX_NUM_EP_OUT+i].ep_xud_chanend = x;

	    asm("getd %0, res[%1]":"=r"(x):"r"(c_ep_in[i]));
        ep_infop[USB_MAX_NUM_EP_OUT+i].ep_client_chanend = x;

        //asm("ldaw %0, %1[%2]":"=r"(x):"r"(ep_info),"r"((USB_MAX_NUM_EP_OUT+i)*sizeof(XUD_ep_info)/sizeof(unsigned)));
        x = (unsigned) ep_infop + ((i+USB_MAX_NUM_EP_OUT) * sizeof(XUD_ep_info));
        
        outuint(c_ep_in[i], x);

        ep_infop[USB_MAX_NUM_EP_OUT+i].pid = USB_PIDn_DATA0;

        epStatFlagTableIn[i] = epTypeTableIn[i] & XUD_STATUS_ENABLE;
        //asm volatile("stw %0, %1[%2]"::"r"(epTypeTableIn[i] & XUD_STATUS_ENABLE),"r"(epTypeTableIn),"r"(i));

        
        epTypeTableIn[i] = epTypeTableIn[i] & 0x7FFFFFFF;

        ep_infop[USB_MAX_NUM_EP_OUT+i].epType = epTypeTableIn[i];

        //ep_info[USB_MAX_NUM_EP_OUT+i].epAddress = 0x80; // OR in the IN bit

    }

    /* EpTypeTable Checks.  Note, currently this is not too crucial since we only really care if the EP is ISO or not */

    /* Check for control on IN/OUT 0 */
    if(epTypeTableOut[0] != XUD_EPTYPE_CTL || epTypeTableIn[0] != XUD_EPTYPE_CTL)
    {
        //XUD_Error("XUD_Manager: Ep 0 must be control for IN and OUT");
    }

#if 0

    /* Check that if the required channel has a destination if the EP is marked as in use */
    for( int i = 0; i < noEpOut + noEpIn; i++ )
    {
        if( XUD_Sup_getd( epChans[i] )  == 0 && epTypeTableOut[i] != XUD_EPTYPE_DIS )
            XUD_Error_hex("XUD_Manager: OUT Ep marked as in use but chanend has no dest: ", i);
    }

    for( int i = 0; i < noEpOut + noEpIn; i++ )
    {
        if( XUD_Sup_getd( epChans[i + XUD_EP_COUNT ] )  == 0 && epTypeTableIn[i] != XUD_EPTYPE_DIS )
            XUD_Error_hex("XUD_Manager: IN Ep marked as in use but chanend has no dest: ", i);
    }
#endif

#ifndef ARCH_S
    /* Clock reset port from reference clock (required as clkblk 0 running from USB clock) */
    if(!isnull(p_rst) && !isnull(clk))
    {
       set_port_clock(p_rst, clk);
    }

    if(!isnull(clk))
    {
       set_clock_on(clk);
       set_clock_ref(clk);
       start_clock(clk);
    }

   #endif

    /* Run the main XUD loop */
    XUD_Manager_loop(usbtile, epChans0, epChans, c_sof, epTypeTableOut, epTypeTableIn, noEpOut, noEpIn, p_rst, rstMask, clk, pwrConfig);

    // Need to close, drain, and check - three stages.
    for(int i = 0; i < 3; i++)
    {
        drain(c_ep_out, noEpOut, i, epTypeTableOut);  // On all inputs
        drain(c_ep_in, noEpIn, i, epTypeTableIn);     // On all output
    }

    } //unsafe
    /* Don't hit */
    return 0;
}


/* Various messages for error cases */
void ERR_BadToken()
{
#ifdef XUD_DEBUG_VERSION
  printstrln("BAD TOKEN RECEVED");
#endif
}

void ERR_BadCrc(unsigned a, unsigned b)
{
  while(1);
}

void ERR_SetupBuffFull()
{
#ifdef XUD_DEBUG_VERSION
  printstrln("SETUP BUFFER FULL");
#endif
}

void ERR_UnsupportedToken(unsigned x)
{
#ifdef XUD_DEBUG_VERSION
  printstr("UNSUPPORTED TOKEN: ");
  printhexln(x);
#endif
}

void ERR_BadTxHandshake(unsigned x)
{
#ifdef XUD_DEBUG_VERSION
  printstr("BAD TX HANDSHAKE: ");
  printhexln(x);
#endif
}

void ERR_GotSplit()
{
#ifdef XUD_DEBUG_VERSION
  printstrln("ERR: Got a split");
#endif
}

void ERR_TxHandshakeTimeout()
{
#ifdef XUD_DEBUG_VERSION
  printstrln("WARNING: TX HANDSHAKE TIMEOUT");
  while(1);
#endif
}

void ERR_OutDataTimeout()
{
#ifdef XUD_DEBUG_VERSION
  printstrln("ERR: Out data timeout");
#endif
}

void ERR_EndIn4()
{
#ifdef XUD_DEBUG_VERSION
  printstrln("ERR: Endin4");
  while(1);

#endif
}

void ERR_EndIn5(int x)
{
#ifdef XUD_DEBUG_VERSION
  printhex(x);
  printstrln(" ERR: Endin5");
  while(1);
#endif
}

void ResetDetected(int x)
{
#ifdef XUD_DEBUG_VERSION
    printint(x);
    printstr(" rrrreeeset\n");
#endif
}

void SuspendDetected()
{
#ifdef XUD_DEBUG_VERSION
    printstr("Suspend!\n");
#endif
}
