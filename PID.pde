/*

PID Module

*/

#define   GUARD_GAIN   10.0

double K  = 1.5 ;      // wheels
double Kp;             
double Ki;                   
double Kd;  
int last_error = 0;
int integrated_error = 0;
int pTerm = 0, iTerm = 0, dTerm = 0;

int updatePid(int targetPosition, int currentPosition)   {

  int error = targetPosition - currentPosition; 
  pTerm = Kp * error;
  integrated_error += error;                                       
  iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
  dTerm = Kd * (error - last_error);                            
  last_error = error;
  return -constrain(K*(pTerm + iTerm + dTerm), -255, 255);
  
}
