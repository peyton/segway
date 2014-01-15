/*

Home Built Segway by Ian Johnston - V2.0

- RC Servo outputs (2off RC Servo signals - drive & direction)
- Digital IMU - Sparkfun SEN-10121
- Arduino AREF externally set to 3.3v
- Balance point saved to EEprom
- Incorporates some ideas from Kasbot & X_BOT_V2 code

*/

#include <math.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Wire.h>     // SDA=pin4, SCL=pin5
#include <FIMU_ADXL345.h>  // Accelerometer
#include <FIMU_ITG3200.h>  // Gyro

// ADXL345 Accelerometer
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library

// ITG3200 Gyro
ITG3200 gyro = ITG3200();
float dx,dy,dz;
int ix, iy, iz;

// Speed up AnalogRead. Defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

Servo SegwayTHR;  // Motor - Drive/throttle output
Servo SegwayDIR;  // Motor - Direction control output

// Define analogue pins
byte AI_Pin_Kp = 7;   // 'P' pot
byte AI_Pin_Ki = 6;   // 'I' pot
byte AI_Pin_Kd = 0;   // 'D' pot
byte AI_Pin_Bal = 1;  // Balance 'zero' point adj. pot
byte AI_Pin_Ste = 3;  // Steering pot

// Define digital pins
byte LED_Pin4 = 4;    // Led - RUN
byte LED_Pin5 = 5;    // Led - Balance
byte BUZ_Pin9 = 9;    // Buzzer
byte DI_Pin6 = 6;     // Run/Stop from relay/footswitch (external pull-up used)
byte DI_Pin7 = 7;     // Run/Stop toggle switch
byte DI_Pin8 = 8;     // Save Settings P/B switch (n/c contact)
//byte TEST_Pin10 = 10;    // Test pin

// Define Vars
byte RunStopSw = 0;
byte RunStopRel = 1;
byte SaveSettingsSW = 0;
byte BalanceLED = 0;
byte RunLED = 0;
double sensorValue[5]  = { 0, 0, 0, 0, 0 };
//double sensorZero[3]  = { 0, 0, 0 }; 
int actAngle;                  // angles in QUIDS (360° = 2PI = 1204 QUIDS
int ACC_angle;
int GYRO_rate;
int setPoint = 0;
int FullSpeed = 0;
int drive = 0;
int balancezero;
int steering = 1500;
int lowfreqloopcount = 0;
int startupdelay = 0;
byte enablemotors = 0;
byte STD_LOOP_TIME = 5;  // This sets the main loop time (original = 9mS)
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;
byte torqueok = 0;
byte EEvalHI;
byte EEvalLO;
void lcd_WriteLine(int lineNum);
void lcd_ShowLine(int lineNum);
void lcd_CursorPos(int cursorLine, int cursorCol);
void lcd_init();

/******************************************************************************
*                               VOID SETUP
*******************************************************************************/
void setup() {

  // Init. LCD
  Serial.begin(38400);           // sets the serial BAUD rate
  lcd_init();                    // Initialize the LCD
  lcd_ShowLine(1);               // Set the display to line 1&2
  lcd_Clear();                   // Clear LCD
  lcd_CursorPos (1,1); Serial.println("     PLEASE     ");
  lcd_CursorPos (2,1); Serial.print("      WAIT      ");

  Wire.begin();
  
  delay(1000);

  // GYRO
  gyro.init(ITG3200_ADDR_AD0_LOW);
  gyro.zeroCalibrate(2500, 2);
  
  // ACCELEROMETER  
  adxl.powerOn();

  // Init. Servo Motor outputs
  SegwayTHR.attach(11);          // Pin for motor throttle
  SegwayDIR.attach(12);          // Pin for motor direction
  SegwayTHR.write(1500);         // Zero throttle and steering asap on power up
  SegwayDIR.write(1500);  

  // Speed up AnalogRead. Set prescale to 16
  // This gives an ADC clock of 1 MHz and thus a sample rate of ~77KHz
  sbi(ADCSRA,ADPS2) ;
  cbi(ADCSRA,ADPS1) ;
  cbi(ADCSRA,ADPS0) ;

  analogReference(EXTERNAL); // 3.3v on AREF pin

  // Init. LEDs/Buzzer
  pinMode(LED_Pin4, OUTPUT);     // sets as output
  pinMode(LED_Pin5, OUTPUT);     // sets as output
  pinMode(BUZ_Pin9, OUTPUT);     // sets as output
  //pinMode(TEST_Pin10, OUTPUT);     // sets as output

  // Power up beep & RUN LED
  digitalWrite(BUZ_Pin9, HIGH);
  delay(200);
  digitalWrite(BUZ_Pin9, LOW);
  digitalWrite(LED_Pin4, LOW);
  
  // Init. Switches
  pinMode(DI_Pin8, INPUT);       // sets digital pin 0 as input
  pinMode(DI_Pin7, INPUT);       // sets digital pin 1 as input
  digitalWrite(DI_Pin8, HIGH);   // turn on pullup resistors
  digitalWrite(DI_Pin7, HIGH);

  PIDSettings();                 // Load initial PID pot settings
  
  //calibrateSensors();            // Set zero IMU sensor values
  
  // Read saved balance zero point from EEprom (1-word)
  EEvalHI = EEPROM.read(0);
  EEvalLO = EEPROM.read(1);
  balancezero = EEvalHI*256+EEvalLO;;
  //sensorZero[0] = balancezero;
  
  if (digitalRead(DI_Pin8) == HIGH)  {  // Force update EEprom (brand new boards) - Hold Balance Zero P/B down on startup
     EEvalHI = 10 / 256;
     EEvalLO = 10 % 256;
     EEPROM.write(0, EEvalHI);          // write to EEprom 1-word, nominal setting of 520
     EEPROM.write(1, EEvalLO);
	 digitalWrite(BUZ_Pin9, LOW);
	 delay(500);
	 digitalWrite(BUZ_Pin9, HIGH);
	 delay(500);
	 digitalWrite(BUZ_Pin9, LOW);
	 delay(500);
	 digitalWrite(BUZ_Pin9, HIGH);
  }
  
  //digitalWrite(TEST_Pin10, LOW);

}

/******************************************************************************
*                               VOID LOOP
*******************************************************************************/
void loop() {

  // Sensor aquisition & filtering
  //updateSensors();
  ACC_angle = getAccAngle();                                                 // in Quids
  GYRO_rate = getGyroRate();                                                 // in Quids/seconds
  actAngle = kalmanCalculate(ACC_angle, GYRO_rate, lastLoopTime);            // calculate Absolute Angle
  
  // PID and motor drive
  drive = updatePid(setPoint, actAngle);                                     // PID algorithm
  Drive_Motor(drive);
    
  // Loop timing control
  lastLoopUsefulTime = millis()-loopStartTime;
  if(lastLoopUsefulTime<STD_LOOP_TIME)         delay(STD_LOOP_TIME-lastLoopUsefulTime);
  lastLoopTime = millis() - loopStartTime;
  loopStartTime = millis();
  
  //digitalWrite(TEST_Pin10, HIGH);
  //digitalWrite(TEST_Pin10, LOW);
  
  // Generate low freq sub
  lowfreqloopcount++;
  if (lowfreqloopcount == 10) {  // Run the lo freq sub every 10 main loops
	 lowfreqloopcount = 0;
	 lofreq();
  }

  // Generate startup delay to Allow IMU sensors to settle and thus stop erroneous motor output
  if (startupdelay < 381 && enablemotors == 0) {
     startupdelay++;    // 380 seems to be long enough for the IMU etc to settle
  }
  if (startupdelay == 380 && enablemotors == 0) {
	 enablemotors = 1;  // Pre-enable motors
  }

}
