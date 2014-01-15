/*

Kalman Filter Module

*/

    double Q_angle  =  0.001;  //0.001
    double Q_gyro   =  0.050;  //0.003 0.020
    double R_angle  =  0.03;   //0.03

    double x_angle = 0;
    double x_bias = 0;
    double P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;	
    double dt, y, S;
    double K_0, K_1;

  double kalmanCalculate(double newAngle, double newRate, int looptime) {  // ACC_angle, GYRO_rate, lastLoopTime

  dt = double(looptime)/1000;             // XXXXXXX arevoir

    x_angle += dt * (newRate - x_bias);
    P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
    P_01 +=  - dt * P_11;
    P_10 +=  - dt * P_11;
    P_11 +=  + Q_gyro * dt;
    
    y = newAngle - x_angle;
    S = P_00 + R_angle;
    K_0 = P_00 / S;
    K_1 = P_10 / S;
    
    x_angle +=  K_0 * y;
    x_bias  +=  K_1 * y;
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;
	
	//x_angle = newAngle;
	
    return x_angle;
  }

