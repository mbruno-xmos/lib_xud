#include <xs1.h>
    
//int rcvPacketInit()
//int rcvPacketVarTimeout(port, data)

    .globl rcvPacketVarTimeout
    .globl rcvInit
    .align 8
rcvInit:
    ENTSP_lu6 0
    getr     r0, 1
    stw      r0, dp[mytimer]   
    ldap     r11, timeOut
    setv     res[r0], r11   // initialise the timer once to jump out to timeOut
    eeu      res[r0]        // And permanently enable
    retsp 0
    

    .align   8
rcvPacketVarTimeout:
    ENTSP_lu6 0
    ldw      r2, dp[mytimer]
    setc     res[r2], XS1_SETC_COND_NONE
    in       r11, res[r2]
    ldc      r3, 0xffff
    add      r11, r11, r3
    add      r11, r11, r3
    add      r11, r11, r3
    add      r11, r11, r3
    add      r11, r11, r3
    add      r11, r11, r3
    add      r11, r11, r3
    setc     res[r2], XS1_SETC_COND_AFTER
    setd     res[r2], r11
    setsr    1            // Enable events
    
rcvLoop:    
    in       r11, res[r0]
    stw      r11, r1[0]
    bf       r11, done0
    in       r11, res[r0]
    stw      r11, r1[1]
    ldaw     r1, r1[2]
    bt       r11, rcvLoop

    ldaw     r1, r1[-1]    // r1 points to the 0 just stored
    
done0:
    clrsr    1
    ldc      r0, 0x0E0E
    stw      r0, r1[0]     // overwrite the 0 with a success tail word
    ldc      r0, 0         // Return 0: not timed out
    stw      r0, r1[1]     // Put a zero at the end
    retsp 0
    
timeOut:
    in       r11, res[r2]
    clrsr    1
    endin    r11, res[r0]
    in       r0, res[r0]
    stw      r0, r1[0]
    stw      r11, r1[1]
    ldc      r11, 0xDEAD
    ldc      r3, 0xBEEF
    shl      r11, r11, 16
    or       r11, r11, r3
    stw      r11, r1[2]
    ldc      r0, 1
    retsp    0



    .align 8
//void sendPacketVarFast(port, ptr)
    .globl sendPacketVarFast

sendPacketVarFast:
    ENTSP_lu6 0

sendLoop:   
    ldw      r11, r1[0]
    out      res[r0], r11
    bf       r11, done
    ldw      r11, r1[1]
    out      res[r0], r11
    ldaw     r1, r1[2] 
    bt       r11, sendLoop
done:   
    retsp    0
