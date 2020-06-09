#include <Servo.h>
#include <Arduino_LSM6DS3.h>
#include <math.h>

// Blynk Setup 
#include <SPI.h>
#include <WiFiNINA.h>
#include <BlynkSimpleWiFiNINA.h>

WidgetLCD lcd(V3);
BlynkTimer timer;

char auth[] = "Vwkvy_LoHJb4dKkoGqXMkqrOgLETZGDZ";
char ssid[] = "ggloffice";
char pass[] = "lundyter14";

Servo ESC_RB, ESC_LB, ESC_LF, ESC_RF;

float acc_x, acc_y, acc_z;
float gyr_x, gyr_y, gyr_z;
float ang_x, ang_y, ang_z;
float S_aX, S_aY, S_aZ, S_gyrX, S_gyrY, S_gyrZ, S_angX, S_angY, S_angZ;
float alpha_acc = 0.9;
float alpha_gyr = 0.8;
float alpha_ang = 0.8;
bool first_acc = 1;
bool first_gyr = 1;
bool first_ang = 1;

float timePrev, timeCur, timeElapsed;
bool atRest = 1;
float acc_x_offset, acc_y_offset, acc_z_offset;
float gyr_x_offset, gyr_y_offset, gyr_z_offset;
int count1;
int count2;

float acc_ang_x, acc_ang_y;
float ang_x_true, ang_y_true;

int x, y; // Integers used to determine the desired angle/direction of the quadcopter
float x_des, y_des; // Desired x and y angles of the quadcopter
float x_err, y_err; // Error in the x- and y-angles
float x_err_prev, y_err_prev; // Previous errors in the x- and y- angles

int throttle; // Input throttle of the quadcopter

// PID Constants:
float Kp_x; 
float Kp_y;
float Ki_x = 0;
float Ki_y = 0;
float Kd_x;
float Kd_y;

float x_p, x_i, x_d, y_p, y_i, y_d; // Proportional, integral, and derivative errors for both the x- and y-axes
int x_pid1, x_pid2, y_pid1, y_pid2; // Raw PID signal values to be passed to the motors
int x_sig1, x_sig2, y_sig1, y_sig2; // Ideal signal values passed to each of the motors (before averaging with each other)
int sigRB, sigLB, sigRF, sigLF; // Actual signal values passed to the motors

void setup() 
{
  Serial.begin(115200);
  IMU.begin();

  pinMode(13, OUTPUT); // Sets the LED as an output
  Blynk.begin(auth, ssid, pass);
  lcd.clear();
  timer.setInterval(1000L, PIDTimer);
  
  ESC_RB.attach(9, 1000, 2000);
  ESC_LB.attach(10, 1000, 2000);
  ESC_LF.attach(11, 1000, 2000);
  ESC_RF.attach(12, 1000, 2000);

  calibrateESCs(); 
}

void PIDTimer()
{
  lcd.print(0, 0, x_sig1);
  lcd.print(8, 0, x_sig2);
  lcd.print(0, 1, y_sig1);
  lcd.print(8, 1, y_sig2);
}

BLYNK_WRITE(V1)
{
  throttle = param.asInt();
  ESC_RB.write(throttle);
  ESC_LB.write(throttle);
  ESC_LF.write(throttle);
  ESC_RF.write(throttle);    
}

BLYNK_WRITE(V2)
{
  x = param[0].asInt();
  y = param[1].asInt();
}

BLYNK_WRITE(V4)
{
  int stopButton = param.asInt();
  if(stopButton)
  {
    ESC_RB.write(1000);
    ESC_LB.write(1000);
    ESC_LF.write(1000);
    ESC_RF.write(1000);
    while(1);
  }
}

BLYNK_WRITE(V10)
{
  Kp_x = param.asInt();
}

BLYNK_WRITE(V11)
{
  Kp_y = param.asInt();
}

BLYNK_WRITE(V12)
{
  Kd_x = param.asInt();
}

BLYNK_WRITE(V13)
{
  Kd_y = param.asInt();
}

void loop() 
{
  Blynk.run();
  timer.run();
  
  timePrev = timeCur;
  timeCur = millis();
  timeElapsed = (timeCur - timePrev)/1000;
   
  if(atRest)
  {
    calibrateIMU();
    atRest = 0;
  }
  
  if(IMU.accelerationAvailable())
  {
    IMU.readAcceleration(acc_x, acc_y, acc_z);
    acc_x = (acc_x * 9.81) - acc_x_offset;
    acc_y = (acc_y * 9.81) - acc_y_offset;
    acc_z = (acc_z * 9.81) - acc_z_offset;

    updateEMAacc();

    /*
    Serial.print("acc_x: ");
    Serial.print(S_aX);
    Serial.print(";  acc_y: ");
    Serial.print(S_aY);
    Serial.print(";  acc_z: ");
    Serial.println(S_aZ);
    */
  }
  
  if(IMU.gyroscopeAvailable())
  {
    IMU.readGyroscope(gyr_x, gyr_y, gyr_z);
    gyr_x = gyr_x - gyr_x_offset;
    gyr_y = gyr_y - gyr_y_offset;
    gyr_z = gyr_z - gyr_z_offset;
    updateEMAgyr();
    
    ang_x = ang_x + (gyr_x * timeElapsed);
    ang_y = ang_y + (gyr_y * timeElapsed);
    ang_z = ang_z + (gyr_z * timeElapsed);
    updateEMAang();

    /*
    Serial.print("gyr_x: ");
    Serial.print(S_gyrX);
    Serial.print(";  gyr_y: ");
    Serial.print(S_gyrY);
    Serial.print(";  gyr_z: ");
    Serial.println(S_gyrZ);

    Serial.print("ang_x: ");
    Serial.print(S_angX);
    Serial.print(";  ang_y: ");
    Serial.print(S_angY);
    Serial.print(";  ang_z: ");
    Serial.println(S_angZ);
    Serial.println();
    */
  }
  
  acc_ang_x = (atan(S_aX/(sqrt(pow(S_aY, 2) + pow(S_aZ, 2))))) * (180/PI);
  acc_ang_y = (atan(S_aY/(sqrt(pow(S_aX, 2) + pow(S_aZ, 2))))) * (180/PI);

  ang_x_true = ((0.98 * S_angX) + (0.02 * acc_ang_x));
  ang_y_true = ((0.98 * S_angY) + (0.02 * acc_ang_y));

  /*
  Serial.print("ang_x: ");
  Serial.print(ang_x_true);
  Serial.print(";  ang_y: ");
  Serial.print(ang_y_true);
  Serial.println();
  */

  // PID Control for x-angle:
  x_des = map(x, 0, 254, -15, 15);
  Serial.println(x_des);
  x_err_prev = x_err;
  x_err = ang_x_true - x_des;

  // Implementing Anti-Windup Method (Clamping):
  if((((x_pid1 < 1100) || (x_pid1 > 2000)) || ((x_pid2 < 1100) || (x_pid2 > 2000))) && (x_i * x_p > 0))
  {
    x_i = x_i; // Clamps the integral error
  }
  else
  {
    x_i = x_i + (timeElapsed * x_err);  
  }
  
  x_d = ((x_err - x_err_prev)/(timeElapsed));
  
  //x_pid1 = (int) throttle - ((Kp_x * x_err) + (Ki_x * x_i) + (Kd_x * x_d));
  x_pid1 = (int) throttle - ((Kp_x * x_err) + (Kd_x * x_d));
  x_pid2 = (int) throttle + ((Kp_x * x_err) + (Kd_x * x_d));
  //x_pid2 = (int) throttle + ((Kp_x * x_err) + (Ki_x * x_i) + (Kd_x * x_d));

  
  // Implementing saturation:
  if(x_pid1 < 1100)
  {
    x_sig1 = 1100;
  }
  else if(x_pid1 > 2000)
  {
    x_sig1 = 2000;
  }
  else
  {
    x_sig1 = x_pid1;
  }

  if(x_pid2 < 1100)
  {
    x_sig2 = 1100;
  }
  else if(x_pid2 > 2000)
  {
    x_sig2 = 2000;
  }
  else
  {
    x_sig2 = x_pid2;
  }

  
  // PID Control for y-angle:
  y_des = map(y, 0, 254, -15, 15);
  y_err_prev = y_err;
  y_err = ang_y_true - y_des;

  // Implementing Anti-Windup Method (Clamping):
  if((((y_pid1 < 1100) || (y_pid1 > 2000)) || ((y_pid2 < 1100) || (y_pid2 > 2000))) && (y_i * y_p > 0))
  {
    y_i = y_i; // Clamps the integral error
  }
  else
  {
    y_i = y_i + (timeElapsed * y_err);  
  }
  
  y_d = ((y_err - y_err_prev)/(timeElapsed));
  
  //y_pid1 = (int) throttle - ((Kp_y * y_err) + (Ki_y * y_i) + (Kd_y * y_d));
  //y_pid2 = (int) throttle + ((Kp_y * y_err) + (Ki_y * y_i) + (Kd_y * y_d));

  y_pid1 = (int) throttle - ((Kp_y * y_err) + (Kd_y * y_d));
  y_pid2 = (int) throttle + ((Kp_y * y_err) + (Kd_y * y_d));
  
  // Implementing saturation:
  if(y_pid1 < 1100)
  {
    y_sig1 = 1100;
  }
  else if(y_pid1 > 2000)
  {
    y_sig1 = 2000;
  }
  else
  {
    y_sig1 = y_pid1;
  }

  if(y_pid2 < 1100)
  {
    y_sig2 = 1100;
  }
  else if(y_pid2 > 2000)
  {
    y_sig2 = 2000;
  }
  else
  {
    y_sig2 = y_pid2;
  }

  sigLF = 0.5 * (x_sig1 + y_sig2);
  sigLB = 0.5 * (x_sig1 + y_sig1);
  sigRB = 0.5 * (x_sig2 + y_sig1);
  sigRF = 0.5 * (x_sig2 + y_sig2);

  ESC_LF.write(sigLF);
  ESC_LB.write(sigLB);
  ESC_RB.write(sigRB);
  ESC_RF.write(sigRF);
}

void calibrateESCs()
{
  ESC_RB.write(2000);
  ESC_LB.write(2000);
  ESC_LF.write(2000);
  ESC_RF.write(2000);
  delay(1500);
  
  ESC_RB.write(1000);
  ESC_LB.write(1000);
  ESC_LF.write(1000);
  ESC_RF.write(1000);  
  delay(1500);
  
  ESC_RB.write(1100);
  ESC_LB.write(1100);
  ESC_LF.write(1100);
  ESC_RF.write(1100);
}

void calibrateIMU()
{
  float acc_x_cur, acc_y_cur, acc_z_cur;
  float gyr_x_cur, gyr_y_cur, gyr_z_cur;
  float acc_x_count, acc_y_count, acc_z_count;
  float gyr_x_count, gyr_y_count, gyr_z_count;
  
  for(int i = 0; i < 1000; i++)
  {
    if(IMU.accelerationAvailable())
    {
      count1++;
      IMU.readAcceleration(acc_x_cur, acc_y_cur, acc_z_cur);      
      acc_x_count = acc_x_count + acc_x_cur;
      acc_y_count = acc_y_count + acc_y_cur;
      acc_z_count = acc_z_count + acc_z_cur;
    }

    if(IMU.gyroscopeAvailable())
    {
      count2++;
      IMU.readGyroscope(gyr_x_cur, gyr_y_cur, gyr_z_cur);
      gyr_x_count = gyr_x_count + gyr_x_cur;
      gyr_y_count = gyr_y_count + gyr_y_cur;
      gyr_z_count = gyr_z_count + gyr_z_cur;
    }
  }

  acc_x_offset = (acc_x_count/count1) * 9.81;
  acc_y_offset = (acc_y_count/count1) * 9.81;
  acc_z_offset = ((acc_z_count/count1) * 9.81) - 9.81;

  gyr_x_offset = gyr_x_count/count2;
  gyr_y_offset = gyr_y_count/count2;
  gyr_z_offset = gyr_z_count/count2;
}

void updateEMAacc()
{
  if(first_acc)
  {
    S_aX = acc_x;
    S_aY = acc_y;
    S_aZ = acc_z;
    first_acc = 0;
  }
  else
  {
    S_aX = (alpha_acc * acc_x) + ((1 - alpha_acc) * S_aX);
    S_aY = (alpha_acc * acc_y) + ((1 - alpha_acc) * S_aY);
    S_aZ = (alpha_acc * acc_z) + ((1 - alpha_acc) * S_aZ);    
  }
}

void updateEMAgyr()
{
  if(first_gyr)
  {
    S_gyrX = gyr_x;
    S_gyrY = gyr_y;
    S_gyrZ = gyr_z;
    first_gyr = 0;
  }
  else
  {
    S_gyrX = (alpha_gyr * gyr_x) + ((1 - alpha_gyr) * S_gyrX);
    S_gyrY = (alpha_gyr * gyr_y) + ((1 - alpha_gyr) * S_gyrY);
    S_gyrZ = (alpha_gyr * gyr_z) + ((1 - alpha_gyr) * S_gyrZ);
  }
}

void updateEMAang()
{
  if(first_ang)
  {
    S_angX = ang_x;
    S_angY = ang_y;
    S_angZ = ang_z;
    first_ang = 0;
  }
  else
  {
    S_angX = (alpha_ang * ang_x) + ((1 - alpha_ang) * S_angX);
    S_angY = (alpha_ang * ang_y) + ((1 - alpha_ang) * S_angY);
    S_angZ = (alpha_ang * ang_z) + ((1 - alpha_ang) * S_angZ);
  }
}
