
//Use 'Bluetooth Electronics' Android App for Control


//Array, used to store the data of the pattern, can be calculated by yourself or obtained from the modulus tool
unsigned char mouthClose[] = {0x00, 0x04, 0x06, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x06, 0x04, 0x00};
unsigned char mouthHalfOpen[] = {0x00, 0x04, 0x06, 0x18, 0x18, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x18, 0x18, 0x06, 0x04, 0x00};
unsigned char mouthOpen[] = {0x00, 0x00, 0x18, 0x24, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x24, 0x18, 0x00, 0x00};

unsigned char start01[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
unsigned char backward[] = {0x00,0x00,0x00,0x00,0x00,0x24,0x12,0x09,0x12,0x24,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char forward[] = {0x00,0x00,0x00,0x00,0x00,0x24,0x48,0x90,0x48,0x24,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char left[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x28,0x10,0x44,0x28,0x10,0x44,0x28,0x10,0x00};
unsigned char right[] = {0x00,0x10,0x28,0x44,0x10,0x28,0x44,0x10,0x28,0x44,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char STOP01[] = {0x2E,0x2A,0x3A,0x00,0x02,0x3E,0x02,0x00,0x3E,0x22,0x3E,0x00,0x3E,0x0A,0x0E,0x00};
unsigned char clear[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

unsigned char rightLessThanLeft[] = {0x7f, 0x09, 0x09, 0x19, 0x26, 0x40, 0x10, 0x28, 0x44, 0x00, 0x00, 0x7f, 0x40, 0x40, 0x40, 0x40};
unsigned char rightGreaterThanLeft[] = {0x7f, 0x09, 0x09, 0x19, 0x26, 0x40, 0x00, 0x44, 0x28, 0x10, 0x00, 0x7f, 0x40, 0x40, 0x40, 0x40};


#define SCL_Pin  A5  //Set clock pin to A5
#define SDA_Pin  A4  //Set data pin to A4

#define ML_Ctrl 13  //define direction control pin of left motor
#define ML_PWM 11   //define PWM control pin of left motor
#define MR_Ctrl 12  //define direction control pin of right motor
#define MR_PWM 3   //define PWM control pin of right motor

#define Trig 5  //ultrasonic trig Pin
#define Echo 4  //ultrasonic echo Pin
int distance; //save the distance value detected by ultrasonic, follow function
int random2; //save the variable of random numberssave the variable of random numbers
//save the distance value detected by ultrasonic, obstacle avoidance function
int frontDistance;  
int leftDistance;
int rightDistance;
int stallCheckDistance = 0;
int pauseAndCheckDist = 45;

#define servoPin 9  //servo Pin
int pulsewidth;

#define light_L_Pin A1   //define the pin of left photo resistor sensor
#define light_R_Pin A2   //define the pin of right photo resistor sensor
int left_light;
int right_light;

char bluetooth_val; //save the value of Bluetooth reception
int flag;  //flag variable, it is used to entry and exist function
void setup(){
  Serial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(ML_Ctrl, OUTPUT);
  pinMode(ML_PWM, OUTPUT);
  pinMode(MR_Ctrl, OUTPUT);
  pinMode(MR_PWM, OUTPUT);
  
  pinMode(servoPin, OUTPUT);
  MoveHead(90); //set servo to 90°
  pinMode(SCL_Pin,OUTPUT);
  pinMode(SDA_Pin,OUTPUT);
  matrix_display(clear);    //Clear the display
  pinMode(light_L_Pin, INPUT);
  pinMode(light_R_Pin, INPUT);
}

void loop(){

  //follow();  //do something (uncomment for testing without bluetooth)


  if (Serial.available())
  {
    bluetooth_val = Serial.read();
    Serial.println(bluetooth_val);
  }
  switch (bluetooth_val) 
  {
    case '1': 
      GoForward(255,255);
      break;
    case '3':  
      GoBackward(255,255);
      break;
    case '4': 
      GoLeft(255,255);
      break;
    case '2':  
      GoRight(255,255);
      break;
    case 'P': 
      GoPause(0,0);
      break;
   case 'Q':
      follow();
      break;
   case 'R':
      AvoidObstacles();
      break;
   case 'S':
      light_track();
      break;
  }}
/*****************Obstacle Avoidance Function**************/
void AvoidObstacles() 
{
  flag = 0;  //the design that enter obstacle avoidance function
  while (flag == 0) 
  {
    random2 = random(1, 100);

    
   frontDistance = CheckDistance();  //assign the front distance detected by ultrasonic sensor to variable a
    
    
    if (frontDistance < pauseAndCheckDist) //when the front distance detected is less than 20cm
    {
      GoPause(0,0);  //robot stops
      delay(200); //delay in 200ms
      
      MoveHead(160);  //Ultrasonic platform turns left
      for (int j = 1; j <= 10; j = j + (1)) { ///for statement, the data will be more accurate if ultrasonic sensor detect a few times. 
        leftDistance = CheckDistance();  //assign the left distance detected by ultrasonic sensor to variable a1
      }
      delay(200);
      MoveHead(20); //Ultrasonic platform turns right
      for (int k = 1; k <= 10; k = k + (1)) {
        rightDistance = CheckDistance(); //assign the right distance detected by ultrasonic sensor to variable a2
      }
      if (leftDistance < 50 || rightDistance < 50)  //robot will turn to the longer distance side when left or right distance is less than 50cm.if the left or right distance is less than 50cm, the robot will turn to the greater distance
      {
        if (leftDistance > rightDistance) //left distance is greater than right
        {
          matrix_display(rightLessThanLeft); 
          MoveHead(90);  //Ultrasonic platform turns back to right ahead ultrasonic platform turns front
          GoLeft(200,200);  //robot turns left
          delay(500);  //turn left 500ms
         GoForward(125,125); //go forward
        } 
        else 
        {
          matrix_display(rightGreaterThanLeft); 
          MoveHead(90);
          GoRight(200,200); //robot turns right
          delay(500);
         GoForward(200,200);  //go forward
        }
      } 
      else  //both distance on two side is greater than or equal to 50cm, turn randomly
      {
        if ((long) (random2) % (long) (2) == 0)  //when the random number is even
        {
          MoveHead(90);
          GoLeft(200,200); //robot turns left
          delay(500);
         GoForward(200,200); //go forward
        } 
        else 
        {
          MoveHead(90);
          GoRight(200,200); //robot turns right
          delay(500);
          GoForward(200,200); ///go forward
       } } } 
      else  //If the front distance is greater than or equal to 20cm, robot car will go front
      {
        GoForward(200,200); //go forward
        delay(500);
        
        stallCheckDistance = CheckDistance();

        if (stallCheckDistance == frontDistance)
        {
          GetUnStuck(200,200);
        }
        
      }


  
  // receive the Bluetooth value to end the obstacle avoidance function
  if (Serial.available())
  {
    bluetooth_val = Serial.read();
    if (bluetooth_val == 'P')  //receive P
    {
      flag = 1;  //when assign 1 to flag, end loop
    }}}}
/*******************Follow****************/
void follow() {
  flag = 0;
  while (flag == 0) {
    distance = CheckDistance();  //assign the distance detected by ultrasonic sensor to distance
    if (distance >= 20 && distance <= 60) //the range to go front
    {
     GoForward(125,125);
    }
    else if (distance > 10 && distance < 20)  //the range to stop
    {
      GoPause(0,0);
    }
    else if (distance <= 10)  // the range to go back
    {
      GoBackward(125,125);
    }
    else  //other situations, stop
    {
      GoPause(0,0);
    }
    if (Serial.available())
    {
      bluetooth_val = Serial.read();
      if (bluetooth_val == 'P') 
      {
        flag = 1;  //end loop
      }}}}
//The function to control ultrasonic sensor the function controlling ultrasonic sensor
float CheckDistance() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  float distance = pulseIn(Echo, HIGH) / 58.00;  //58.20 means 2*29.1=58.2
  delay(10);
  return distance;
}

//The function to control servo the function controlling servo
void MoveHead(int myangle) {
  for (int i = 0; i <= 50; i = i + (1)) {
    pulsewidth = myangle * 11 + 500;
    digitalWrite(servoPin,HIGH);
    delayMicroseconds(pulsewidth);
    digitalWrite(servoPin,LOW);
    delay((20 - pulsewidth / 1000));
  }}

/****************Light Follow******************/
void light_track() {
  flag = 0;
  while (flag == 0) {
    left_light = analogRead(light_L_Pin);
    right_light = analogRead(light_R_Pin);
    if (left_light > 450 && right_light > 450) //the value detected by photo resistor, go forward   //was 650 for all
    {  
     GoForward(125,125);
    } 
    else if (left_light > 450 && right_light <= 450)  //the value detected by photo resistor, turn left
    {
      GoLeft(125,125);
    } 
    else if (left_light <= 450 && right_light > 450) //the value detected by photo resistor, turn right
    {
      GoRight(125,125);
    } 
    else  //other situations, stop
    {
      GoPause(0,0);
    }
    if (Serial.available())
    {
      bluetooth_val = Serial.read();
      if (bluetooth_val == 'P') {
        flag = 1;
     }}}}
/***************Dot Matrix *****************/
// this function is used for dot matrix display 
void matrix_display(unsigned char matrix_value[])
{
  IIC_start();
  IIC_send(0xc0);  //Choose address
  
  for(int i = 0;i < 16;i++) //pattern data has 16 bits
  {
     IIC_send(matrix_value[i]); //convey the pattern data
  }
  IIC_end();   //end the transmission of pattern data
  IIC_start();
  IIC_send(0x8A);  //display control, set pulse width to 4/16
  IIC_end();
}
//The condition starting to transmit data
void IIC_start()
{
  digitalWrite(SCL_Pin,HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin,HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin,LOW);
  delayMicroseconds(3);
}
//convey data
void IIC_send(unsigned char send_data)
{
  for(char i = 0;i < 8;i++)  //each byte has 8 bits
  {
      digitalWrite(SCL_Pin,LOW);  //pull down clock pin SCL Pin to change the signals of SDA
      delayMicroseconds(3);
      if(send_data & 0x01)  //set high and low level of SDA_Pin according to 1 or 0 of every bit
      {
        digitalWrite(SDA_Pin,HIGH);
      }
      else
      {
        digitalWrite(SDA_Pin,LOW);
      }
      delayMicroseconds(3);
      digitalWrite(SCL_Pin,HIGH); //pull up clock pin SCL_Pin to stop transmitting data
      delayMicroseconds(3);
      send_data = send_data >> 1;  // detect bit by bit, so move the data right by one
  }}
//The sign that data transmission ends
void IIC_end()
{
  digitalWrite(SCL_Pin,LOW);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin,LOW);
  delayMicroseconds(3);
  digitalWrite(SCL_Pin,HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin,HIGH);
  delayMicroseconds(3);
}
/*************the function to run motor*************/
void GoForward(int rightSpeed, int leftSpeed)
{
  matrix_display(forward); 
  digitalWrite(MR_Ctrl,LOW);
  analogWrite(MR_PWM,rightSpeed);
  digitalWrite(ML_Ctrl,LOW);
  analogWrite(ML_PWM,leftSpeed);
}
void GoBackward(int rightSpeed, int leftSpeed)
{
  matrix_display(backward);
  digitalWrite(MR_Ctrl,HIGH);
  analogWrite(MR_PWM,rightSpeed);
  digitalWrite(ML_Ctrl,HIGH);
  analogWrite(ML_PWM,leftSpeed);
}
void GoLeft(int rightSpeed, int leftSpeed)
{
  matrix_display(left); 
  digitalWrite(MR_Ctrl,LOW);
  analogWrite(MR_PWM,rightSpeed);
  digitalWrite(ML_Ctrl,HIGH);
  analogWrite(ML_PWM,leftSpeed);
}
void GoRight(int rightSpeed, int leftSpeed)
{
  matrix_display(right);  
  digitalWrite(MR_Ctrl,HIGH);
  analogWrite(MR_PWM,rightSpeed);
  digitalWrite(ML_Ctrl,LOW);
  analogWrite(ML_PWM,leftSpeed);
}
void GoPause(int rightSpeed, int leftSpeed)
{
  matrix_display(STOP01); 
  digitalWrite(MR_Ctrl,LOW);
  analogWrite(MR_PWM,rightSpeed);
  digitalWrite(ML_Ctrl,LOW);
  analogWrite(ML_PWM,leftSpeed);
}

void GetUnStuck(int rightSpeed, int leftSpeed)
{
  GoBackward(rightSpeed,leftSpeed);
  delay(500);
  GoRight(rightSpeed,leftSpeed);
  delay(500);
}


void Mouth()
{
  int randomDelay = random(10, 20000); 
  delay(200);
  matrix_display(mouthClose); 
  delay(randomDelay);
  matrix_display(mouthHalfOpen); 
  delay(80);
  matrix_display(mouthOpen); 

}
