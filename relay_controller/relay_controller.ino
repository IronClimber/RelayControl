#include "TM1637.h"

/*MODES
 * 0 - tune
 * 1 - work
 */

/*
 * 
 * 0 - work
 * 1 - time
 * 2 - period
 * 3 - duration
 */

//--------PARAMS-------

void print_long(unsigned long long p) {
    char buf[50];
    sprintf(buf, "%lu", p);
    Serial.print( buf );
}



#define MODE_WORK 0
#define MODE_TIME 1
#define MODE_PERIOD 2
#define MODE_DURATION 3

int mode = MODE_WORK;
unsigned long long time_ = 10800000;//1000*60*60*3;
unsigned long long period_ = 600000; //1000*60*10;
unsigned long long duration_ = 1000*5;

unsigned long long last_display_time = millis();
unsigned long long last_control_time = millis();
unsigned long long last_tune_time = millis();
unsigned long long last_params_time = millis();
unsigned long long last_update_encoders = millis();

unsigned long long current_time = millis();

#define TIME_UPPER_LIMIT 36000000
#define TIME_LOWER_LIMIT 3600000
#define PERIOD_UPPER_LIMIT 900000
#define PERIOD_LOWER_LIMIT 60000
#define DURATION_UPPER_LIMIT 180000
#define DURATION_LOWER_LIMIT 1000

void next_mode() {
  mode++;
  if (mode>MODE_DURATION) {
    mode = MODE_WORK;
  }
  
}


//-------INDICATOR

#define CLK 4//pins definitions for TM1637 and can be changed to other ports
#define DIO 5

TM1637 tm1637(CLK,DIO);

void display_digits(unsigned int d) {
  int low = 0;
  int middle = 0;
  int high = 0;
  high = (d/100);
  middle = ((d - high*100)/10);
  low = d - high*100 - middle*10;
  if (high != 0) {  tm1637.display(1,high); }
  else {tm1637.display(1,124);}
  if (middle != 0) { tm1637.display(2,middle); }
  else {tm1637.display(2,124);}
  tm1637.display(3,low);  
  //Serial.print(high);
  //Serial.print(middle);
  //Serial.print(low);
}

void display_time(void) {
  unsigned int out = time_/1000/60/60;
  display_digits(out);
}

void display_period(void) {
  unsigned int out = period_/1000/60;
  display_digits(out);
}

void display_duration(void) {
  unsigned int out = duration_/1000;
  display_digits(out);
}

//-------ENCODERS--------

void check_rotatry();

#define BUFFER_SIZE 3

int encoder_pin_a = 2;
int encoder_pin_b = 3;

int previous_a;
int previous_b;

int buf = 0;
int pos = 0;

bool switch_state = true;
int switch_pin = 6;

unsigned long last_encoder = millis();

//----------------------

void setup() {

  tm1637.init();
  tm1637.set(BRIGHTEST);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
  
  // put your setup code here, to run once:
  pinMode(encoder_pin_a, INPUT_PULLUP);
  pinMode(encoder_pin_b, INPUT_PULLUP);
  pinMode(switch_pin, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encoder_pin_a), encoderRead, RISING);
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  current_time = millis();
  if(current_time - last_update_encoders > 10) {
    check_rotary();

    previous_a = digitalRead(encoder_pin_a);
    previous_b = digitalRead(encoder_pin_b);

    last_update_encoders = millis();
  }
  if(current_time - last_display_time > 1000) {
    tm1637.display(0,mode);
    if(mode == MODE_TIME) {
      display_time();
    }
    else if(mode == MODE_PERIOD) {
      display_period();
    }
    else if(mode == MODE_DURATION) {
      display_duration();
    }
    else if(mode == MODE_WORK) {
      display_digits(167);
    }

    
    last_display_time = millis();
  }
  if(current_time - last_params_time > 1000) {
    print_long(time_);
    Serial.print(", ");
    print_long(period_);
    Serial.print(", ");
    print_long(duration_);
    Serial.println();
    
    last_params_time = millis();
  }
  if(current_time - last_control_time > 50) {
    if(digitalRead(switch_pin) == LOW) { 
      if (switch_state) {
        next_mode();
        switch_state = false;
      }
      last_control_time = millis();
     
    }
    else {
      switch_state = true;
    }
  }
  if(current_time - last_tune_time > 50) {
    if(mode == MODE_TIME) {
      time_ += pos*TIME_LOWER_LIMIT;
      if (time_ < TIME_LOWER_LIMIT) {time_ = TIME_LOWER_LIMIT;}
      else if (time_ > TIME_UPPER_LIMIT) { time_ = TIME_UPPER_LIMIT;}
    }
    else if(mode == MODE_PERIOD) {
      period_ += pos*PERIOD_LOWER_LIMIT;
      if (period_ < PERIOD_LOWER_LIMIT) { period_ = PERIOD_LOWER_LIMIT; }
      else if (period_ > PERIOD_UPPER_LIMIT) { period_ = PERIOD_UPPER_LIMIT; }
    }
    else if(mode == MODE_DURATION) {
      duration_ += pos*DURATION_LOWER_LIMIT;
      if (duration_ < DURATION_LOWER_LIMIT) { duration_ = DURATION_LOWER_LIMIT; }
      else if (duration_ > DURATION_UPPER_LIMIT) { duration_ = DURATION_UPPER_LIMIT; }
      if (duration_ > period_) { duration_ = period_; }
    }
    pos = 0;
  }
    
}

void encoderRead(void) {

  if (digitalRead(encoder_pin_a) == HIGH) {
    if(digitalRead(encoder_pin_b) == HIGH) {
      
      buf--;
      if (buf<-BUFFER_SIZE) {
        pos--;
        
        buf = 0;
      }
      
    }
    else {
      
      buf++;
      if (buf>BUFFER_SIZE) {
        pos++;
        buf = 0;
      }
    }

  }
}

void check_rotary() {

  if((previous_a == 0) && (previous_b == 1)) {
    if ((digitalRead(encoder_pin_a) == 1) && (digitalRead(encoder_pin_b) == 0)) {
      pos++;
    }
    if ((digitalRead(encoder_pin_a) == 1) && (digitalRead(encoder_pin_b) == 1)) {
      pos--;
    }
  }

  if((previous_a == 1) && (previous_b == 0)) {
    if ((digitalRead(encoder_pin_a) == 0) && (digitalRead(encoder_pin_b) == 1)) {
      pos++;
    }
    if ((digitalRead(encoder_pin_a) == 0) && (digitalRead(encoder_pin_b) == 0)) {
      pos--;
    }
  }

  if((previous_a == 1) && (previous_b == 1)) {
    if ((digitalRead(encoder_pin_a) == 0)  && (digitalRead(encoder_pin_b) == 1)) {
      pos++;
    }
    if ((digitalRead(encoder_pin_a) == 0) && (digitalRead(encoder_pin_b) == 0)) {
      pos--;
    }
  }

  if((previous_a == 0) && (previous_b == 0)) {
    if ((digitalRead(encoder_pin_a) == 1) && (digitalRead(encoder_pin_b) == 0)) {
      pos++;
    }
    if ((digitalRead(encoder_pin_a) == 1) && (digitalRead(encoder_pin_b) == 1)) {
      pos--;
    }
  }
}
