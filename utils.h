#include "esp_system.h"
#define A1 26 // Left
#define B1 27
#define P1 25
#define A2 14 // Right
#define B2 12
#define P2 13

#define pi 3.1416
#define DIAMETER 0.069

#define LEFT 0
#define RIGHT 1
#define ClkWise 0
#define AClkWise 1

#define outputA_R 39
#define outputB_R 36
#define outputA_L 35
#define outputB_L 34

double counter_R = 0;
double counter_L = 0;
hw_timer_t* timer = NULL;
double Rcounter_cur = 0;
double Lcounter_cur = 0;
double Rcounter_prev = 0;
double Lcounter_prev = 0;
double rps_R = 0;
double rps_L = 0;
double counts_per_rev = 206;
double count_diff_L = 0;
double count_diff_R = 0;
double distance_travelled = 0;

void Count_to_distance();


void IRAM_ATTR onTimer() {   // 100 ms timer

 
  Rcounter_cur = counter_R;    
  count_diff_R = Rcounter_cur - Rcounter_prev;
  double R_revs = count_diff_R/counts_per_rev;
  rps_R = R_revs*100;
  Rcounter_prev = Rcounter_cur;
  

  Lcounter_cur = counter_L;
  count_diff_L = Lcounter_cur - Lcounter_prev;
  double L_revs = count_diff_L/counts_per_rev;
  rps_L = L_revs*100;
  Lcounter_prev = Lcounter_cur;

}

void updateCounter_R(){
   if(digitalRead(outputB_R)==LOW)
  {
    counter_R++;
  }

  else
  {
    counter_R--;
  }

  Count_to_distance();

}

void updateCounter_L(){
   if(digitalRead(outputB_L)==LOW)
  {
    counter_L++;
  }

  else
  {
    counter_L--;
  }

  Count_to_distance();
  
}


void SetPins() {
  pinMode(A1, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(P1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(B2, OUTPUT);
  pinMode(P2, OUTPUT);

  pinMode(outputA_R, INPUT);
  pinMode(outputB_R, INPUT);
}

void SetTimer() {
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100000, true);
  timerAlarmEnable(timer);

  attachInterrupt(digitalPinToInterrupt(outputA_R), updateCounter_R, RISING);
  attachInterrupt(digitalPinToInterrupt(outputA_L), updateCounter_L, RISING);
}

void driveMotor(int Location, double speed, int direction) {
  
  if(Location){
    double pwm_R = map(speed, 0, 100, 0, 255);
    digitalWrite(A2, !direction);
    digitalWrite(B2, direction);
    analogWrite(P2,pwm_R);
  }


  else{
    double pwm_L = map(speed, 0, 100, 0, 255);
    digitalWrite(A1, !direction);
    digitalWrite(B1, direction);
    analogWrite(P1, pwm_L);
  }
}

void Count_to_distance()
{
  double revs_total_L = counter_L/counts_per_rev;
  double revs_total_R = counter_R/counts_per_rev;

  double dist_left = revs_total_L * pi * DIAMETER;
  double dist_right = revs_total_R * pi * DIAMETER;

  distance_travelled = (dist_left + dist_right)/2;

  
}
