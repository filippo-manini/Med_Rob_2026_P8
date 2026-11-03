

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#include <Stepper.h>

const int dirPin_AP = 4; //manages the rotation of the Delivery in AP plane
const int stepPin_AP = 5;

const int dirPin_ML = 2; //: manages the rotation of the Delivery in ML plane
const int stepPin_ML = 3;

const int dirPin_Linear_out = 8; //Linear Actuator: insertion of the Catheter
const int stepPin_Linear_out = 10;

const int dirPin_Linear_in = 12; //Linear actuator: manages the insertion of the Delivery inside the Catheter
const int stepPin_Linear_in = 11;

const int stepsPerRevolution = 400;
const int  rot_step_num = 50; //un giro sono 1600 step in totale

const int tot_step_lin = 200; //un giro sono 1600 step in totale
const int  lin_step_num = 10; 
int count_lin_step = 0;
char val;

long int t1,t2 ;

int m,l,a,p;

int finish = 0;

Stepper Stepper_ML(stepsPerRevolution, dirPin_ML, stepPin_ML );
Stepper Stepper_AP(stepsPerRevolution, dirPin_AP, stepPin_AP);
Stepper Linear_Catheter(stepsPerRevolution, dirPin_Linear_out, stepPin_Linear_out );
Stepper Linear_Delivery(stepsPerRevolution,dirPin_Linear_in, stepPin_Linear_in);

void setup() {

  digitalWrite(stepPin_AP, LOW);
  digitalWrite(dirPin_AP, LOW);
  digitalWrite(stepPin_ML, LOW);
  digitalWrite(dirPin_ML, LOW);
  digitalWrite(stepPin_Linear_out, LOW);
  digitalWrite(dirPin_Linear_out, LOW);
  digitalWrite(stepPin_Linear_in, LOW);
  digitalWrite(dirPin_Linear_in, LOW);
  pinMode(stepPin_AP, OUTPUT);
  pinMode(dirPin_AP, OUTPUT);
  pinMode(stepPin_ML, OUTPUT);
  pinMode(dirPin_ML, OUTPUT);
  pinMode(stepPin_Linear_out, OUTPUT);
  pinMode(dirPin_Linear_out, OUTPUT);
  pinMode(stepPin_Linear_in, OUTPUT);
  pinMode(dirPin_Linear_in, OUTPUT);

  // set motors speed Hz
   Stepper_ML.setSpeed(20);
   Stepper_AP.setSpeed(20);
   Linear_Catheter.setSpeed(150);
   Linear_Delivery.setSpeed(150);
   
  Serial.begin(9600);

}


void loop() {
 
  if(Serial.available()){
    char val = Serial.read();
   
    if(val == 'm'){
        Stepper_ML.step(-rot_step_num);
        Serial.print(" ML CW \r\n ");
    }
   if(val == 'l'){

        Stepper_ML.step(rot_step_num);
        Serial.print(" ML CCW \r\n ");
      
    }
   if(val == 'a'){
 
        Stepper_AP.step(rot_step_num);
        Serial.print(" AP CCW \r\n ");
    
    }
    if(val == 's'){
     
        Stepper_AP.step(-rot_step_num);
        Serial.print(" AP CW \r\n ");
     
    }
    if(val == 'i'){
        while(count_lin_step<tot_step_lin){
          Linear_Catheter.step(lin_step_num);
          Serial.print(" LINEAR IN \r\n ");
          //Linear_Delivery.step(lin_step_num);
          //Serial.print(" DELIVERY IN \r\n ");
          count_lin_step = count_lin_step + 10;
        }
      count_lin_step = 0;
    }
    if(val == 'o'){
      while(count_lin_step<tot_step_lin){
        Linear_Catheter.step(-lin_step_num);
        Serial.print(" LINEAR OUT \r\n ");
        //Linear_Delivery.step(-lin_step_num);
        //Serial.print(" DELIVERY OUT \r\n ");
        count_lin_step = count_lin_step+10;
        }
    count_lin_step = 0;
    }
  }
}
