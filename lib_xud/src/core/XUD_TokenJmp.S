
    ldaw        r10, dp[PidJumpTable]                 // TODO Could load from sp here
                                                      // We receive: | 0000 4-bit EP | 0000 4-bit PID |
    inpw       r11, res[RXD], 8                       // Read 8 bit PID
    shr        r11, r11, 24                           // Shift off junk

    ldw        r10, r10[r11]                          // Load relevant branch address

.xtabranch  Pid_Out, Pid_Sof, Pid_In, Pid_Setup, Pid_Sof_NoChan
    bau        r10

