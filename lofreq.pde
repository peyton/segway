/*

Low Frequency Sub

*/

void lofreq() {

  steering = analogRead(AI_Pin_Ste);
  steering = map(steering, 0, 1023, 2500, 500) + 40;  // Scale pot to RC uS range. 1500uS is centre. Tweaked for IanJ's pot
  if (steering >2000) { steering = 2000;}
  if (steering <1000) { steering = 1000;}
  
  RunStopSw = digitalRead(DI_Pin7);               // Run/Stop Toggle Switch
  RunStopRel = digitalRead(DI_Pin6);              // Run/Stop Relay/Footswitch Input
  SaveSettingsSW = digitalRead(DI_Pin8);          // Save Setings P/B (n/c contact)
  
  if (BalanceLED == 1) {
     digitalWrite(LED_Pin5, HIGH);                // Balance LED on
  } else {
     digitalWrite(LED_Pin5, LOW);                 // Balance LED off
  }
  
  if (RunLED == 1) {
     digitalWrite(LED_Pin4, HIGH);                // Run LED on
  } else {
     digitalWrite(LED_Pin4, LOW);                 // Run LED off
  }  
  
  if (FullSpeed == 1) {
     digitalWrite(BUZ_Pin9, HIGH);                // Motors flat out - Buzzer on
  } else {
     digitalWrite(BUZ_Pin9, LOW);                 // Buzzer off
  }  
  
}
