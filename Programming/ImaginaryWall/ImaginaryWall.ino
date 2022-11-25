
class PID{
public:
float Kp,Ki,Kd,Ku,Ke;
float sample_time;
float errorsum;
float prev_error = 0.0;


PID(float KP, float KI, float KD, float KU,float KE, float sample_time){
  this->Kp = KP;
  this->Ki = KI;
  this->Kd = KD;
  this->Ku = KU;
  this->Ke = KE;
  this->sample_time = sample_time;
}

float PIDUpdate(float Error){
  Error = Error * this->Ke;
  float P = Error * this->Kp;
  this->errorsum += ((Error + this->prev_error)/2.0) * this->sample_time;
  if(this->errorsum > 0.15){
    this->errorsum = 0.15;
  }else if(this->errorsum < -0.15){
    this->errorsum = -0.15;
  }
  float I = this->errorsum * this->Ki;
  float D = (Error - this->prev_error)/this->sample_time * this->Kd;
  float result = (P+I+D) * this->Ku;
  if(result > this->Ku){
    result = this->Ku;
  }else if (result < -this->Ku){
    result = -this->Ku;
  }
  this->prev_error = Error;
  return result;
}

};

 #define outputA 8
 #define outputB 9
 #define PWM 5
 #define Dir1 4
 #define Dir2 6

 int counter = 0; 
 int aState;
 int aLastState;
 float angle = 0.0;
 float angle_limit = 90.0;
 float setpoint = 0.0;
 float error = 0.0;
 int PWMvalue = 0;
 int testflag = 1;
 PID MotorPID(1.0,0.05,0.0,120,1/150.0,0.001);
 unsigned long time;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(PWM,OUTPUT);
  pinMode(Dir1,OUTPUT);
  pinMode(Dir2,OUTPUT);

  pinMode (outputA,INPUT);
  pinMode (outputB,INPUT);
  time = millis();
}

void loop() {
  // put your main code here, to run repeatedly:

  aState = digitalRead(outputA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aState != aLastState && aState == 1){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB) != aState) { 
       counter ++;
     } else {
       counter --;
     }
    //  Serial.print("Position: ");
    //  Serial.println(counter);
      angle = (counter)/20.0 * 360.0;
      Serial.println(angle);
 
   } 
    aLastState = aState; // Updates the previous state of the outputA with the current state

    if(angle > angle_limit ){
      setpoint = angle_limit;
    }else if (angle < -angle_limit){
      setpoint = -angle_limit;
    }
 


    if(angle > angle_limit || angle < -angle_limit){
      if(millis() >= time + 1){
        error = 0.0 - angle;
        PWMvalue = MotorPID.PIDUpdate(error);
        if(PWMvalue > 0){
          analogWrite(5,135 + abs(PWMvalue));
          digitalWrite(4,HIGH);
          digitalWrite(6,LOW);
        }else if (PWMvalue < 0){
          analogWrite(5,135 + abs(PWMvalue));
          digitalWrite(4,LOW);
          digitalWrite(6,HIGH);
        }else if(PWMvalue == 0){
          analogWrite(5,0);
          digitalWrite(4,LOW);
          digitalWrite(6,LOW);
        }
        time = millis();
      }
  }else{
     analogWrite(5,0);
     digitalWrite(4,LOW);
     digitalWrite(6,LOW);
  }
  

}
