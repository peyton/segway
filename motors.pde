/*

Motors Module

torque range -255 to +255
motordrive range 1000 to 2000uS

*/

double motordrive = 1500;
double motordriveave = 1500;  // Pre-load averaging formula to avoid initialisation glitches/slowness
double motordir = 1500;
float k = 1;

int Drive_Motor(int torque)  {

  if (enablemotors == 1)  {             // Power up delay is finished, now test if safe to enable motors
     lcd_CursorPos (1,1); Serial.println("STARTUP BALANCE ");
	 lcd_CursorPos (2,7); Serial.print(torque); Serial.print("  ");
  
     if (torque > -10 && torque < 10) { torqueok = 1; }

     if (torqueok == 0) {               // Torque not near balance point so re-test
	     digitalWrite(BUZ_Pin9, HIGH);  // Buzzer on
		 delay(50);
		 digitalWrite(BUZ_Pin9, LOW);   // Buzzer off
		 startupdelay = 379;            // Reset, reactivate main loop
		 enablemotors = 0;
     } else {
	     enablemotors = 2;              // Safe to enable motors
	     digitalWrite(BUZ_Pin9, LOW);   // Buzzer off
	     startupdelay = 381;            // Park the startupdelay counter
		 LCDparams();                   // 1-shot the current params to the LCD but don't update
     }
  }

  if (SaveSettingsSW == 1 && RunStopSw == 1) {      // Save Settings P/B (n/c contact)
	 EEUpdate();
  }

  if (torque > 0) {      // Balance LED
     BalanceLED = 1;     // LED on
  } else {
     BalanceLED = 0;     // LED off
  }
  
  if (torque == 255 || torque == -255) {
     FullSpeed = 1;     // Motors flat out
  } else {
     FullSpeed = 0;
  }   

  if (RunStopRel == 0) {             // If Run/Stop Relay is open (security just incase main power relay sticks)
     enablemotors = 0;               // Disable motors
	 startupdelay = 381;             // Park the startupdelay counter if not already
	 motordrive = 1500;              // Stop motors
     motordir = 1500;
	 SegwayTHR.writeMicroseconds(motordrive);
	 SegwayDIR.writeMicroseconds(motordir);
	 digitalWrite(BUZ_Pin9, HIGH);
  }

  if (RunStopSw == 1 && enablemotors == 2) {          // Run/Stop switch in STOP position
     RunLED = 0;                                      // RUN LED off
     motordrive = 1500;                               // Stop motors
     motordir = 1500;
	 SegwayTHR.writeMicroseconds(motordrive);
	 SegwayDIR.writeMicroseconds(motordir);
	 PIDSettings();                                   // Update PID pots
	 BalanceSet();                                    // Update Balance pot
	 LCDparams();                                     // Update LCD
  }

  if (RunStopSw == 0 && enablemotors == 2) {          // Run Stop Toggle Switch in RUN position and motors fully enabled
     RunLED = 1;                                      // RUN LED on
	 motordrive = map(torque, -255, 255, 1000, 2000);
	 motordriveave = motordriveave + (1 / k * ((double)motordrive - motordriveave));  // Averaged using Yn=Yn+(1/k*(Xn-Yn)) where Xn = input, Yn = output, k = constant
     if (motordriveave >2000) { motordriveave = 2000;}    // Limits check
	 if (motordriveave <1000) { motordriveave = 1000;}
     SegwayTHR.write(motordriveave);                      // Output drive to motor control board
	 SegwayDIR.write(steering);                           // Output steering to motor control board
  }

}
