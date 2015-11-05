// Received SOF token: Reset suspend/reset detection timer
.align 4
Pid_Sof:
    inpw        r10, res[RXD], 8                   // Read EP Number
    shr         r10, r10, 24;                      // Shift off junk

    in          r11, res[r1];
    bt          r11, InvalidToken;                 // If VALID_TOKEN not high, ignore token
    clrsr       0x3
    ldw         r11, sp[(STACK_EXTEND+9)]

    out         res[r11], r10
    ldw         r10, sp[14]                        // Load timer from stack
    setc        res[r10], XS1_SETC_COND_NONE       // Read current time
    ldw         r8, sp[13]                         // Add suspend timeout to current time
    in          r11, res[r10]
    add         r11, r11, r8
    setd        res[r10], r11
    setc        res[r10], XS1_SETC_COND_AFTER      // Re-enable thread interrupts
    setsr       0x3
	bu          Loop_BadPid

// Received SOF token: Reset suspend/reset detection timer
.align 4
Pid_Sof_NoChan:
    inpw        r10, res[RXD], 8;                  // Read EP Number */
    shr         r10, r10, 24;                      // Shift off junk */

    in          r11, res[r1];
    bt          r11, InvalidToken;                 // If VALID_TOKEN not high, ignore token 
    clrsr       0x3
    ldw         r10, sp[14]                        // Load timer from stack
    
    setc        res[r10], XS1_SETC_COND_NONE       // Read current time
    ldw         r8, sp[13]
    in          r11, res[r10]
    add         r11, r11, r8
    setd        res[r10], r11
    ldw         r8, sp[13]
    setc        res[r10], XS1_SETC_COND_AFTER      // Re-enable thread interrupts
    setsr       0x3
	bu          Loop_BadPid




