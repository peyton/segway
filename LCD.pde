/*

LCD Module

*/

void lcd_init() {
    delay(2500); // Let the LCD initialise

    Serial.write(byte(12)); // Clear the screen and display line 1&2 (no command bit required)

    //Setup the Cursor Style
    Serial.write(byte(254));  // Command
    Serial.write(byte(67));   // Set cursor style command
    Serial.write(byte(0));  // No cursor
}


void lcd_Clear() {
    lcd_CursorPos (1,1);                // LCD line,column
    Serial.write("                "); // Clear line
    lcd_CursorPos (2,1);                // LCD line,column
    Serial.write("                "); // Clear line
}


void lcd_WriteLine(int lineNum) {
    Serial.write(byte(254));       // Command
    Serial.write(byte(76));        // Goto line n command
    Serial.write(byte(lineNum));   // Write to line n
}


void lcd_CursorPos(int cursorLine, int cursorCol) {
    Serial.write(byte(254));          // Command
    Serial.write(byte('P'));          // Position cursor command
	Serial.write(byte(cursorLine));   // Set to line n
    Serial.write(byte(cursorCol));    // Set to column n
}


void lcd_ShowLine(int lineNum) {
    Serial.write(byte(254));       // Command
    Serial.write(byte(71));        // Display the following line number command
    Serial.write(byte(lineNum)); // Show line number n (1-16)
}

