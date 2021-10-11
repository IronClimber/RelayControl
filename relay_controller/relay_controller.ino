#include "TM1637.h"

/*MODES
 * 0 - tune
 * 1 - work
 */

 #define STATE_TUNE 0
 #define STATE_WORK 1
 
 int state = STATE_TUNE;
 int toogle_state_counter = 0;
 void toogle_state() {
    if (state == STATE_TUNE) {
      state = STATE_WORK;
    }
    else if (state == STATE_WORK) {
      state = STATE_TUNE;
    }  
 }


/*
 * 
 * 0 - work
 * 1 - time
 * 2 - period
 * 3 - duration
 */

//--------PARAMS-------

#define MODE_WORK 0
#define MODE_TIME 1
#define MODE_PERIOD 2
#define MODE_DURATION 3

int mode = MODE_WORK;
//unsigned long long time_ = 10800000;//1000*60*60*3;
//unsigned long long period_ = 600000; //1000*60*10;
//unsigned long long duration_ = 1000*5;

unsigned long long time_ = 60000;//1000*60*60*3;
unsigned long long period_ = 6000; //1000*60*10;
unsigned long long duration_ = 1000;


unsigned long long time_counter = 0;
unsigned long long period_counter = 0;
unsigned long long duration_counter = 0;


unsigned long long last_display_time = millis();
unsigned long long last_control_time = millis();
unsigned long long last_tune_time = millis();
unsigned long long last_params_time = millis();
unsigned long long last_update_encoders = millis();
unsigned long long last_timer_update_time = millis();

unsigned long long current_time = millis();

int push_state = 0;

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

int get_minutes_from_start() {
  return int(time_counter/1000/60);
}
int get_minutes_to_finish() {
  return int((time_-time_counter)/1000/60)+1;
}

//------------Print-------------------

void print_long(unsigned long long p) {
    char buf[50];
    sprintf(buf, "%lu", p);
    Serial.print( buf );
}

void print_status() {
    Serial.print("State: ");
    if(state==STATE_WORK) {
      Serial.print("WORK");
    }
    else if(state==STATE_TUNE) {
      Serial.print("TUNE");
    }
    else {
      Serial.print("?");
    }

    Serial.print(", Mode: ");
    if(mode==MODE_WORK) {
      Serial.print("WORK");
    }
    else if(mode==MODE_TIME) {
      Serial.print("TIME");
    }
    else if(mode==MODE_PERIOD) {
      Serial.print("PERIOD");
    }
    else if(mode==MODE_DURATION) {
      Serial.print("DURATION");
    }
    else {
      Serial.print("?");
    }

    Serial.print(". ");
}

void print_params() {
  Serial.print("TIME: ");
  print_long(time_);
  Serial.print(", PERIOD: ");
  print_long(period_);
  Serial.print(", DURATION: ");
  print_long(duration_);
  Serial.print(". ");

}

void print_timer() {
  Serial.print("TIME: ");
  print_long(time_counter);
  Serial.print(", PERIOD: ");
  print_long(period_counter);
  Serial.print(", DURATION: ");
  print_long(duration_counter);
  Serial.print(". ");

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

void check_rotary();

#define BUFFER_SIZE 3

int encoder_pin_a = 2;
int encoder_pin_b = 3;

int previous_a;
int previous_b;

int buf = 0;
int pos = 0;

bool switch_state = true;
int switch_pin = 6;
int push_pin = 8;

unsigned long last_encoder = millis();

//----------------------relay

int relay_pin = 7;


//---led----------
int led_pin = 13;

void setup() {

  tm1637.init();
  tm1637.set(BRIGHTEST);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;
  
  // put your setup code here, to run once:
  pinMode(encoder_pin_a, INPUT_PULLUP);
  pinMode(encoder_pin_b, INPUT_PULLUP);
  pinMode(switch_pin, INPUT_PULLUP);
  pinMode(push_pin, INPUT_PULLUP);
  pinMode(relay_pin, OUTPUT);
  pinMode(led_pin, OUTPUT);
  digitalWrite(relay_pin, LOW);

  
  attachInterrupt(digitalPinToInterrupt(encoder_pin_a), encoderRead, RISING);
  Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  current_time = millis();

  //---------------Update encoder------------------------
  /*
  if(current_time - last_update_encoders > 10) {
    check_rotary();

    previous_a = digitalRead(encoder_pin_a);
    previous_b = digitalRead(encoder_pin_b);

    last_update_encoders = millis();
  }
  */
  //---------------Display indicator-----------------------
  if(current_time - last_display_time > 1000) {
    if (state == STATE_WORK) { digitalWrite(led_pin, HIGH); }
    else { digitalWrite(led_pin, LOW); }
    //tm1637.display(0,mode);
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
      if (state == STATE_WORK) {
      display_digits(get_minutes_to_finish());
      }
      else if (state == STATE_TUNE) {
        display_digits(111);
      }
    }

    
    last_display_time = millis();
  }

  //-----Send params to serial-------
  if(current_time - last_params_time > 1000) {
    /*print_long(time_);
    Serial.print(", ");
    print_long(period_);
    Serial.print(", ");
    print_long(duration_);*/
    print_status();
    //print_params();
    print_timer();
    Serial.print(get_minutes_from_start());
    Serial.print(" ");
    Serial.print(get_minutes_to_finish());
    Serial.println();
    
    last_params_time = millis();
  }

  //---------------Control mode tuning-----------------------
  if(current_time - last_control_time > 50) {
    if(digitalRead(switch_pin) == LOW) { 
      //Should reset counter
      if (switch_state) {
        toogle_state_counter++;
      }
      if (toogle_state_counter > 50) { 
        toogle_state(); 
        toogle_state_counter = 0;
        switch_state =false;
      }
    }
    else {
      switch_state = true;
      if (toogle_state_counter > 1 && toogle_state_counter < 30) {
          //Serial.println(toogle_state_counter);
          //Serial.println("NEXT_MODE!!!!!!!");
          next_mode();
          //Serial.println(mode);
      }

      toogle_state_counter = 0;
    }
    last_control_time = millis();
  }

  //
  
  //----------------------Set params---------------------------------
  if(current_time - last_tune_time > 50) {
    if(state == STATE_TUNE) {
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
    }
    pos = 0;
  }
  
  // ----------------------Run Relay -----------------------------------------
   if(digitalRead(push_pin) == LOW || push_state) {
      digitalWrite(relay_pin, LOW);
  }
  else {
      digitalWrite(relay_pin, HIGH);
  }
  
  // --------------------TImer-----------------------------------------------
  unsigned long long delta_timer_update_time = current_time - last_timer_update_time; 
  if (state==STATE_WORK && delta_timer_update_time > 10) {    
    if(time_counter > time_) {
      //off
      state = STATE_TUNE;
      //reset
            
      time_counter = 0;
      period_counter = 0;
      duration_counter = 0;      
    }
    else {
      time_counter += delta_timer_update_time;   
    

    if(period_counter > period_) {
      push_state = 1;
      period_counter = 0;
    }
    else {
      period_counter += delta_timer_update_time;
    }

    if(push_state) {
      if(duration_counter > duration_) {
        push_state = 0;
        duration_counter = 0;
      }
      else {
        duration_counter += delta_timer_update_time;
      }
    }
    }
    last_timer_update_time = millis();
  }
  
}

void encoderRead(void) {

  if (digitalRead(encoder_pin_a) == HIGH) {
    if (digitalRead(encoder_pin_b) == HIGH) {
      
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
    //+
    if ((digitalRead(encoder_pin_a) == 1) && (digitalRead(encoder_pin_b) == 0)) {
      pos++;
    }
    //+
    if ((digitalRead(encoder_pin_a) == 1) && (digitalRead(encoder_pin_b) == 1)) {
      pos--;
    }
  }

  if((previous_a == 1) && (previous_b == 0)) {
    //+
    if ((digitalRead(encoder_pin_a) == 0) && (digitalRead(encoder_pin_b) == 1)) {
      pos++;
    }
    //+
    if ((digitalRead(encoder_pin_a) == 0) && (digitalRead(encoder_pin_b) == 0)) {
      pos--;
    }
  }

  //
  if((previous_a == 1) && (previous_b == 1)) {
    //+
    if ((digitalRead(encoder_pin_a) == 0)  && (digitalRead(encoder_pin_b) == 1)) {
      pos++;
    }
    //+
    if ((digitalRead(encoder_pin_a) == 0) && (digitalRead(encoder_pin_b) == 0)) {
      pos--;
    }
  }

  if((previous_a == 0) && (previous_b == 0)) {
    //+
    if ((digitalRead(encoder_pin_a) == 1) && (digitalRead(encoder_pin_b) == 0)) {
      pos++;
    }
    //+
    if ((digitalRead(encoder_pin_a) == 1) && (digitalRead(encoder_pin_b) == 1)) {
      pos--;
    }
  }
}
