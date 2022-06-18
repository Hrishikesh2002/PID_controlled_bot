#include "esp_system.h"
#define A1 26 // Left
#define B1 27
#define P1 25
#define A2 14 // Right
#define B2 12
#define P2 13


#define outputA_R 36
#define outputB_R 39
#define outputA_L 34
#define outputB_L 35

double counter_R = 0;
double counter_L = 0;
hw_timer_t* timer = NULL;
double Rcounter_cur = 0;
double Lcounter_cur = 0;
double Rcounter_prev = 0;
double Lcounter_prev = 0;
double rpm_R = 0;
double rpm_L = 0;
int counts_per_rev = 206;
double count_diff_R = 0;
double count_diff_L = 0;



void IRAM_ATTR onTimer() {
  Rcounter_cur = counter_R;
  count_diff_R = Rcounter_cur - Rcounter_prev;
  double R_revs = count_diff_R/counts_per_rev;
  rpm_R = R_revs*10;
  Rcounter_prev = Rcounter_cur;

  Lcounter_cur = counter_L;
  count_diff_L = Lcounter_cur - Lcounter_prev;
  double L_revs = count_diff_L/counts_per_rev;
  rpm_L = L_revs*10;
  Lcounter_prev = Lcounter_cur;

}

void updateCounter(){
    if(digitalRead(outputB_R)==LOW)
  {
    counter_R++;
  }

  else
  {
    counter_R--;
  }


    if(digitalRead(outputB_L)==LOW)
  {
    counter_L++;
  }

  else
  {
    counter_L--;
  }
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

  attachInterrupt(digitalPinToInterrupt(outputA_R), updateCounter, RISING);
  attachInterrupt(digitalPinToInterrupt(outputA_L), updateCounter, RISING);
}
