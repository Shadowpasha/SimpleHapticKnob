
class PID{
public:
float Kp,Ki,Kd,Ku,Ke;
float sample_time;
float errorsum = 0.0;
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
  if(this->errorsum > 2.0){
    this->errorsum = 2.0;
  }else if(this->errorsum < -2.0){
    this->errorsum = -2.0;
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
 float last_angle = 0.0;
 float error = 0.0;
 float speed = 0.0;
 int PWMvalue = 0;
 int testflag = 1;
 unsigned long last_speed;
 PID MotorPID(0.5,1.00,0.0,135,1/50.0,0.01);
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

      // Serial.println(angle);
      // Serial.print(String(angle) + " ");
      
   } 
    aLastState = aState; // Updates the previous state of the outputA with the current state
    
 
  if(millis() >= time + 10){
    speed = ((angle - last_angle)/0.01)/6.0;
    last_angle = angle;
    error = 50.0 - speed;
    PWMvalue = MotorPID.PIDUpdate(error);
    if(PWMvalue > 0){
      analogWrite(5,120 + abs(PWMvalue));
      digitalWrite(4,HIGH);
      digitalWrite(6,LOW);
    }else if (PWMvalue < 0){
      analogWrite(5,120 + abs(PWMvalue));
      digitalWrite(4,LOW);
      digitalWrite(6,HIGH);
    }
    time = millis();

  }

  

}
