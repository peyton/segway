/*

Settings Adjust Mode

*/

void PIDSettings()  {  // Motors have been stopped so enable LCD and settings mode.
	Kp = analogRead(AI_Pin_Kp);                           // Read external pot 0-1023
	Kp = Kp / 100;                                        // Range = 0 - 10
	Ki = analogRead(AI_Pin_Ki);                           // Read external pot 0-1023
	Ki = Ki / 56;
	Ki = Ki - 9;                                          // Range = +/- 9
	Kd = analogRead(AI_Pin_Kd);                           // Read external pot 0-1023
	Kd = Kd / 56;
	Kd = Kd - 9;                                          // Range = +/- 9
}

void LCDparams()  {  // Display LCD
    lcd_CursorPos (1,1); Serial.println("Kp=");
    lcd_CursorPos (1,4); Serial.print(Kp); Serial.print("  ");
    lcd_CursorPos (1,9); Serial.println("Ki=");
    lcd_CursorPos (1,12); Serial.print(Ki); Serial.print("  ");
	lcd_CursorPos (2,1); Serial.println("Kd=");
    lcd_CursorPos (2,4); Serial.print(Kd); Serial.print("  ");
	lcd_CursorPos (2,9); Serial.println("Ze=");
    lcd_CursorPos (2,12); Serial.print(balancezero); Serial.print("  ");
	delay(40);
}

void BalanceSet()  {  //
  	 balancezero = analogRead(AI_Pin_Bal);
	 balancezero = map(balancezero, 0, 1023, -25, 25);  // Tight range for balance zero val
     //sensorZero[0] = balancezero;       // Update sensor zero
	 LCDparams();
}

void EEUpdate()  {  //
     digitalWrite(BUZ_Pin9, HIGH);  // Buzzer on
	 EEvalHI = balancezero / 256;
     EEvalLO = balancezero % 256;
	 EEPROM.write(0, EEvalHI);  // write to EEprom 1-word. Probably writing too often, but it's not as if it's frequently done!
	 EEPROM.write(1, EEvalLO);
	 delay(100);
	 digitalWrite(BUZ_Pin9, LOW);  // Buzzer off
  }
