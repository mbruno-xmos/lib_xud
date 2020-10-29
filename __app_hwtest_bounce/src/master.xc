
//#define DEBUG

#define USB_TILE 0

#include <print.h>
#include <stdio.h>
#include <stdlib.h>
#include <xs1.h>
#include <platform.h>

#include "setup.h"

#ifdef X200
#include <xs2_su_registers.h>
#else
#include <xs1_su_registers.h>
#endif

#include "xud.h"
#include "XUD_HAL.h"
void XUD_ConfigUsbPorts(); // Not part of lib_xud api

#ifdef XK_216_MC
on tile[0]: out port p_USB_SEL = XS1_PORT_8C;

void hw_setup_xk_216_mc(void){
  p_USB_SEL <: 0x18; //Enable VBUS and USB A socket
  while(1){ //Stop ports turning off!
  }
}
#endif

on stdcore[0]: port USB_SEL=XS1_PORT_32A;
on stdcore[0]: port p4c = XS1_PORT_4C;
on stdcore[0]: port p4e = XS1_PORT_4E;

static const unsigned poly = 0xEDB88320;

extern int data[1258];
int data_received[1258];

#define PATTERN0 0xC3
#define PATTERN1 0xFFFFFFFE          // Experimental: a good one
#define PATTERN2 0xAAAA5555
#define PATTERN3 0xFFFFFFFE
#define PATT_END 0x0                 // This must be zero
#define PATT_END_BOUNCE 0x0F0F0F0F
#define PATT_END_LOCAL  0x00000E0E
#define PATT_ILLEGAL    0x1234CDEF

#pragma unsafe arraysx
static void initPacket(int len) {
    // Initialise data to be sent
    for(int i = 0 ; i < len; i+=4) {
        data[i+0] = PATTERN0;
        data[i+1] = PATTERN1;
        data[i+2] = PATTERN2;
        data[i+3] = PATTERN3;
    }
    data[len] = PATT_END;
    // Now make sure that data_received does not have a legal end on it
    data_received[len+1] = PATT_ILLEGAL;
    data_received[len+2] = PATT_ILLEGAL;
}

#pragma unsafe arrays
static int failedPacket(int len, int first) {
    // Check that the first four words are ok

    if (data_received[0] != first) return 1;
    if (data_received[1] != PATTERN1) return 1;
    if (data_received[2] != PATTERN2) return 1;
    if (data_received[3] != PATTERN3) return 1;

    // Now check that the rest of the body is as expected
    for(int i = 4 ; i < len; i+=4) {
        if (data_received[i+0] != PATTERN0) return 1;
        if (data_received[i+1] != PATTERN1) return 1;
        if (data_received[i+2] != PATTERN2) return 1;
        if (data_received[i+3] != PATTERN3) return 1;
    }

    // Now check that the tail comprises an END marker from the bouncer,
    // a local END marker, and a temrinating 0
    if (data_received[len  ] != PATT_END_BOUNCE) return 1;
    if (data_received[len+1] != PATT_END_LOCAL) return 1;
    if (data_received[len+2] != PATT_END) return 1;
    return 0;
}

#ifdef DEBUG
void printPacket(int data[]) {
    int lastVal = 0;
    int skipped = 0;
    for(int i = 0; i < 256; i+= 8) {
        int printMe = 0;
        for(int j = 0; j < 8; j++) {
            if (i == 0 || data[i+j] != data[i+j-8]) {
                printMe = 1;
                break;
            }
        }
        if (printMe) {
            if (skipped) {
                printf("      ........ \n");
                skipped = 0;
            }
            printf("%04x  ", i);
            for(int j = 0; j < 8; j++) {
                printf("%08x ", data[i+j]);
                if (data[i+j] == 0xdeadbeef) {
                    printf(" Timed out...\n");
                    return;
                }
            }
            lastVal = data[i+7];
            printf("\n");
        } else {
            skipped = 1;
        }
    }
    if (skipped) {
        printf("      ........ ");
        for(int j = 0; j < 6; j++) {
            printf("%08x ", lastVal);
        }
        printf("........\n");
    }
}
#endif

extern void exit(int);
extern void sendPacketVarFast(out buffered port:32 txd, int ptr[]);
extern int rcvPacketVarTimeout(in buffered port:32 rxd, int ptr[]);

extern int rcvInit();

// Pick this value carefully.
// The BIST lets through 1-2% test escapes
// These test escapes have 1-2% bad packets
// A choice of 1000 for GOOD_PACKETS_REQUIRED will catch 90% of the test escapes, and let through 0.1-0.2% bad ones.
// A choice of 10000 will catch 99% of the test escapesm and let through 0.01-0.02% bad ones.

#define GOOD_PACKETS_REQUIRED 19999

extern int mytimer;

// Length of the initial packet - used to test whether bouncer has been powered up.

#define INITIAL_LENGTH  12

// The tester sequences through TYPES and LENGTH modes
// The four TYPE modes indicate what the start word looks like
// The 32 LENGTH modes indicate how long the packet is
// The LENGTH modes use the lengths array below to define a packet length
// in words for each mode; note that most of them are short and fast. They catch
// almost all errors.

#define MODE_PACKET_TYPES_BITS 2
#define MODE_PACKET_TYPES_MASK ((1 << MODE_PACKET_TYPES_BITS)-1)
#define MODE_PACKET_LENGTH_BITS 5
#define MODE_PACKET_LENGTH_MASK ((1 << MODE_PACKET_LENGTH_BITS)-1)

static unsigned char lengths[MODE_PACKET_LENGTH_MASK+1] = {
    4,4,4,4,4,4,4,4,
    4,4,4,4,4,4,4,4,
    4,4,4,4,4,4,4,4,
    4,4,4,8,8,12,12,252,
};

/** Function that executes all code needed to test USB from tile 0. 
 */
int main_(void) {
    timer tmr;
    int t;
    int cnt = 0x80000000;
    int mode;
    int total = 0;
    int t2;
    int errors = 0;
#ifdef DEBUG
    int good = 0;
    int printCnt = 0;
#endif

#ifdef SLICEKIT
#ifdef X200
    p4e <: 1;
#else
    p4c <: 1;
#endif
#else
#ifndef XKAUDIO
    //USB_SEL <: 0x00; /* 0x20 for A and 0 for B */
#endif
#endif
    XUD_ConfigUsbPorts();

    XUD_HAL_EnableUsb(XUD_PWR_BUS);
   
    XUD_HAL_EnterMode_PeripheralHighSpeed();
  
    clearbuf(p_usb_rxd);

    unsigned time; 
    timer t1;
    t1 :> time;
    t1 when timerafter(time + 10000) :> int _;


    rcvInit();         // Sets up the timer vector

    printstrln("started");

    // Wait for bouncer to be ready
    cnt = 0;
    while(1) {

        tmr :> t;
        tmr when timerafter(t+100000) :> void;
        clearbuf(p_usb_rxd);

        initPacket(12);

        sendPacketVarFast(p_usb_txd, data);
        if (!rcvPacketVarTimeout(p_usb_rxd, data)) {
            /* receive the rest of the bounces */
            for(int i = 0; i < (BOUNCECOUNT-1); i++)
                rcvPacketVarTimeout(p_usb_rxd, data);
            break;
        }
        cnt++;
        if (cnt >= 10000) {
#ifdef PRINTRESULT
            printstrln("============ FAIL DURING SETUP ==============");
            while(1);
#endif
            exit(1);            
        }
    }
    cnt = 0x80000000;
    mode = 0;
    tmr :> t2;
    while(1) {
        int isWrong, len, first;
        switch(mode & MODE_PACKET_TYPES_MASK) {
        case 0:
            first = 0xa5;
            break;
        case 1:
            first = 0xa5; /* SOF */
            break;
        case 2:
            first = 0x69; /* IN */
            break;
        case 3:
            first = 0x2d; /* SETUP */
            break;
        }
        len = lengths[(mode >> MODE_PACKET_TYPES_BITS) & MODE_PACKET_LENGTH_MASK];
        initPacket(len);
        data[0] = first;
        clearbuf(p_usb_rxd);

        sendPacketVarFast(p_usb_txd, data);

        isWrong = 0;
        for(int i = 0; i < BOUNCECOUNT; i++)
        {
            rcvPacketVarTimeout(p_usb_rxd, data_received); 
            isWrong += failedPacket(len, first);


            //printPacket(data);
            //printPacket(data_received);
            //exit(1);

        }

#ifdef DEBUG
        printCnt++;
        if (printCnt == 20000 || isWrong) {
            tmr :> t;
            t -= t2;
            if (good != 0) {
                if (isWrong) {
                    printf("\n");
                }
                printf("%d: Run of %d good packets summary: %d errors %d us\n",
                       total, good, errors, t*10);
            }
            tmr :> t2;
            printCnt = 0;
        }
        if (isWrong) {
            errors++;
            printf("Error %d/%d on count %08x mode %02x\n", errors, isWrong, cnt, mode);
            printPacket(data_received);
            good = 0;
        } else {
            good ++;
        }
#else
#if 0
        if (isWrong) {
            errors++;
        }
        if (total >= 10 * GOOD_PACKETS_REQUIRED) {
#ifndef PASSFAIL
            printf("%d errors on %d packets\n", errors, total);
#endif
            if (errors > 0) {
#ifdef PASSFAIL
                printf("FAIL\n");
#endif
                exit(1);
            } else {
#ifdef PASSFAIL
                printf("PASS\n");
#endif
                exit(0);
            }
        }
#else
        if (isWrong) {
#ifdef PRINTRESULT
            printstrln("FAIL");
            printintln(total);
#endif
            printf("============ FAIL on packet %d ==============\n", total);
            exit(1);
        }
        if (total == GOOD_PACKETS_REQUIRED) {
#ifdef PRINTRESULT
            printstrln("GOOD");
#endif
            printf("DONE @@@@@@@@@@@@@@@@@@@@\n");
            exit(0);
        }
#endif
#endif
        total++;
        mode++;
        if (mode >> (MODE_PACKET_TYPES_BITS+MODE_PACKET_LENGTH_BITS)) {
            mode = 0;
            cnt++;
        }
    }
    return 0;
}

#ifndef BOUNCER
int main()
{
 par{
#ifdef XK_216_MC
    on tile[1]: main_();
    on tile[0]: hw_setup_xk_216_mc();
#else
    main_();
#endif
  }
  return 0;
}
#endif
