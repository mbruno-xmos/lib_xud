/*
     The code below receives and sends 0-terminated words.
    */


#include <xs1.h>
    
//int rcvPacketVar(0, port, ptr)
    .globl rcvPacketVar

    .align   4
rcvPacketVar:

rcvLoop:
    in       r11, res[r1]
    stw      r11, r2[r0]
    add      r0, r0, 1
    shr      r3, r0, 8
    bt       r3, escape
    bt       r11, rcvLoop
    ldc      r11, 0
    stw      r11, r2[r0]
    sub      r0, r0, 1
    ldc      r1, 0x0F0F
    shl      r11, r1, 16
    or       r11, r11, r1
    stw      r11, r2[r0]
    retsp 0
    
escape:
    ldc      r1, 0xF0F0
    shl      r11, r1, 16
    or       r11, r11, r1
    stw      r11, r2[r0]
    retsp 0


//void sendPacketVar(port, ptr)
    .globl sendPacketVar

    .align   4
sendPacketVar:
    ldc      r3, 0
sendLoop:
    ldw      r11, r1[r3]
    out      res[r0], r11
    add      r3, r3, 1
    bt       r11, sendLoop
    retsp 0

    
