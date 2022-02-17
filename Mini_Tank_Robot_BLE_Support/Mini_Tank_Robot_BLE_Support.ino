//-----------------------------------------------------------------------------------------------------------------------------------
//Code written for for Ks0428 keyestudio Mini Tank Robot V2.  https://www.icedcoffeeist.com/keyestudio-mini-tank-robot-v2-assembly-code-and-test/

//Robot wired according to assembly guides at: https://wiki.keyestudio.com/Ks0428_keyestudio_Mini_Tank_Robot_V2
//If you've change the wiring, you'll need to review the pins defined within this code.

//Use Keuwlsoft's 'Bluetooth Electronics' Android App to create you own Bluetooth controller.


//-----------------------------------------------------------------------------------------------------------------------------------
//DOT MATRIX LED DISPLAY
//-----------------------------------------------------------------------------------------------------------------------------------
//Array, used to store the images for the 8 x 16 LED Display.  Used in all Modes.
unsigned char mouthClose[] = {0x00, 0x04, 0x06, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x06, 0x04, 0x00};
unsigned char mouthHalfOpen[] = {0x00, 0x04, 0x06, 0x18, 0x18, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x18, 0x18, 0x06, 0x04, 0x00};
unsigned char mouthOpen[] = {0x00, 0x00, 0x18, 0x24, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x24, 0x18, 0x00, 0x00};

unsigned char backward[] = {0x00,0x00,0x00,0x00,0x00,0x24,0x12,0x09,0x12,0x24,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char forward[] = {0x00,0x00,0x00,0x00,0x00,0x24,0x48,0x90,0x48,0x24,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char left[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x44,0x28,0x10,0x44,0x28,0x10,0x44,0x28,0x10,0x00};
unsigned char right[] = {0x00,0x10,0x28,0x44,0x10,0x28,0x44,0x10,0x28,0x44,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char brake[] = {0x2E,0x2A,0x3A,0x00,0x02,0x3E,0x02,0x00,0x3E,0x22,0x3E,0x00,0x3E,0x0A,0x0E,0x00};
unsigned char clear[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

unsigned char rightLessThanLeft[] = {0x7f, 0x09, 0x09, 0x19, 0x26, 0x40, 0x10, 0x28, 0x44, 0x00, 0x00, 0x7f, 0x40, 0x40, 0x40, 0x40};
unsigned char rightGreaterThanLeft[] = {0x7f, 0x09, 0x09, 0x19, 0x26, 0x40, 0x00, 0x44, 0x28, 0x10, 0x00, 0x7f, 0x40, 0x40, 0x40, 0x40};

#define SCL_Pin  A5        //Set clock pin to A5
#define SDA_Pin  A4        //Set data pin to A4

//-----------------------------------------------------------------------------------------------------------------------------------
//MOTOR CONTROL
//-----------------------------------------------------------------------------------------------------------------------------------
#define ML_Ctrl 13         //Left Motor Direction Control Pin 13
#define ML_PWM 11          //Left Motor PWM Control Pin 11
#define MR_Ctrl 12         //Right Motor Direction Control Pin 12
#define MR_PWM 3           //Right Motor PWM Control Pin 3

int rSpeedStop = 0;        //Right tank speeds (0 -255)
int lSpeedStop = 0;        //Left tank speeds (0 -255)

int rSpeedSlow = 125;
int lSpeedSlow = 125;

int rSpeedMed = 200;
int lSpeedMed = 200;

int rSpeedFast = 255;
int lSpeedFast = 255;

//-----------------------------------------------------------------------------------------------------------------------------------
//ULTRASONIC DISTANCE
//-----------------------------------------------------------------------------------------------------------------------------------

int setDistance = 45;     //Sets the front distance to check in Avoid Obstacles mode.
#define Trig 5            //Ultrasonic Trig Pin 5
#define Echo 4            //Ultrasonic Echo Pin 4

//-----------------------------------------------------------------------------------------------------------------------------------
//SERVO (HEAD) CONTROL
//-----------------------------------------------------------------------------------------------------------------------------------
#define servoPin 9        //servo Pin

//-----------------------------------------------------------------------------------------------------------------------------------
//LIGHT SENSORS
//-----------------------------------------------------------------------------------------------------------------------------------
#define light_L_Pin A1   //Left Light Sensor Pin A1
#define light_R_Pin A2   //Right Light Sensor Pin A2

//-----------------------------------------------------------------------------------------------------------------------------------
//BLUETOOTH
//-----------------------------------------------------------------------------------------------------------------------------------
char bluetoothRecValue; //Save the received Bluetooth value
int flag;               //To enter and exit modes.

//-----------------------------------------------------------------------------------------------------------------------------------
//setup()
//-----------------------------------------------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(9600);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(ML_Ctrl, OUTPUT);
  pinMode(ML_PWM, OUTPUT);
  pinMode(MR_Ctrl, OUTPUT);
  pinMode(MR_PWM, OUTPUT);
  
  pinMode(servoPin, OUTPUT);
  MoveHead(90); 			      //Servo head looks forward (servo at 90)
  pinMode(SCL_Pin,OUTPUT);
  pinMode(SDA_Pin,OUTPUT);
  matrix_display(clear);    //Clear the display
  pinMode(light_L_Pin, INPUT);
  pinMode(light_R_Pin, INPUT);
}

//-----------------------------------------------------------------------------------------------------------------------------------
//loop()
//-----------------------------------------------------------------------------------------------------------------------------------
void loop()
{
  // FollowDistance();  // Uncomment for testing without bluetooth
  // AvoidObstacles();  // Uncomment for testing without bluetooth
  // FollowLight();     // Uncomment for testing without bluetooth
  CheckBTChar();        //Keep looking for character(s) sent via Bluetooth
}
  
//-----------------------------------------------------------------------------------------------------------------------------------
//AvoidObstacles() - Avoid Obstacles Mode
//-----------------------------------------------------------------------------------------------------------------------------------

void AvoidObstacles()
{
  int frontAngle = 90;
  
  flag = 0; 
  while (flag == 0)                               //Enter the mode.
  {
    int fDist = CheckFrontDistance(frontAngle);   //Check the front distance
  
    if (fDist > setDistance*2 )                   //If the distance to stop and look is still a ways out, medium speed ahead.
    {
      GoForward(rSpeedMed,lSpeedMed);     
    }
  
    else
    {
      GoForward(rSpeedSlow,lSpeedSlow);          //Otherwise, slow speed.
    }
    
    if (fDist  <= setDistance)                   //Front distance is the set distance or less. 
    {
      StopLookDecide();                          //Stop , look, decide direction.
    }         

    if (Serial.available())
    {
      bluetoothRecValue = Serial.read();
      
      if (bluetoothRecValue == 'P')  //receive P
      {
        flag = 1;  //Exit the mode.
      }
    } 
  }
}

void StopLookDecide()
{
  Brake(rSpeedStop,lSpeedStop);                        //STOP
 
  int rightAngle = 20;  
  int frontAngle = 90;  
  int leftAngle = 160;
  int lDist = CheckLeftDistance(leftAngle);           //Look left and save the distance to lDist.
  int rDist = CheckRightDistance(rightAngle);         //Look right and save the distance to rDist.
  int random2; 


  if ((lDist < setDistance) || (rDist < setDistance)) 
  {
    if (lDist > rDist)                              //Left distance is greater than right
    {
      matrix_display(rightLessThanLeft); 
      MoveHead(frontAngle);                         //Head looks forward
      GoLeft(rSpeedMed,lSpeedMed);                  //Robot turns left for 500mS
      delay(500);          
      GoForward(rSpeedSlow,lSpeedSlow);             //Go forward
    } 
    else 
    {
      matrix_display(rightGreaterThanLeft); 
      MoveHead(frontAngle);                         //Head looks forward
      GoRight(rSpeedMed,lSpeedMed);                 //Robot turns right for 500mS
      delay(500);
      GoForward(rSpeedSlow,lSpeedSlow);             //Go forward
    }
  } 
  else                                              //If distance on both sides is greater than or equal to setDistance, turn randomly
  {
    if ((long) (random2) % (long) (2) == 0)         //When the random number is even
    {
      MoveHead(frontAngle);
      GoLeft(rSpeedMed,lSpeedMed);                  //Robot turns left for 500mS                             
      delay(500);
      GoForward(rSpeedSlow,lSpeedSlow);             //Go forward                         
    } 
    else 
    {
      MoveHead(frontAngle);
      GoRight(rSpeedMed,lSpeedMed);                 //Robot turns right for 500mS  
      delay(500);
      GoForward(rSpeedSlow,lSpeedSlow);             //Go forward
    }
  }
}


int CheckFrontDistance(int angle) 
{
  int frontDistance;
  MoveHead(angle); 
  frontDistance = CheckDistance();  //Assign the front distance detected by ultrasonic sensor.
  return frontDistance;
}


int CheckLeftDistance(int angle) 
{
  int leftDistance;
  int sum = 0;
  int avg = 0;
  int num = 4;

  MoveHead(angle); 
  
  for (int i=0; i <num; i++)
  {  
    leftDistance = CheckDistance();  //Assign the front distance detected by ultrasonic sensor.
    sum += leftDistance;
  }
  
  leftDistance = sum/num;            //Average a few measurements.
  return leftDistance;
}

int CheckRightDistance(int angle) 
{
  int rightDistance;
  int sum = 0;
  int avg = 0;
  int num = 4;

  MoveHead(angle); 
  
  for (int i=0; i <num; i++)
  {  
    rightDistance = CheckDistance();  //Assign the front distance detected by ultrasonic sensor.
    sum += rightDistance;
  }
  
  rightDistance = sum/num;
  return rightDistance;
}


//-----------------------------------------------------------------------------------------------------------------------------------
//FollowDistance() - Follow Distance Mode
//-----------------------------------------------------------------------------------------------------------------------------------

void FollowDistance() 
{
  int distance;
  
  Brake(rSpeedStop,lSpeedStop);                        //Make sure robot starts in stoped position
  
  flag = 0;
  while (flag == 0) 
  {
    distance = CheckDistance();                       //Assign the distance detected by ultrasonic sensor to distance
    if (distance >= 20 && distance <= 60)             //the range to go front
    {
     GoForward(rSpeedSlow,lSpeedSlow);                //Go forward
    }
    else if (distance > 10 && distance < 20)          //The range to stop
    {
      Brake(rSpeedStop,lSpeedStop);
    }
    else if (distance <= 10)                          // The range to go back
    {
      GoBackward(rSpeedSlow,lSpeedSlow);              //Go backward
    }
    else  //Stop in other situations
    {
      Brake(rSpeedStop,lSpeedStop);
    }
     if (Serial.available())
    {
      bluetoothRecValue = Serial.read();
      if (bluetoothRecValue == 'P') 
      {
        flag = 1;  //Exit the mode.
      }
    }  
  }  
}


//-----------------------------------------------------------------------------------------------------------------------------------
//FollowLight() - Follow light Mode
//-----------------------------------------------------------------------------------------------------------------------------------

void FollowLight()
{
  int leftLight;
  int rightLight;
  
  Brake(rSpeedStop,lSpeedStop);  //Make sure robot starts in stoped position
  
  flag = 0;
  while (flag == 0) 
  {
    leftLight = analogRead(light_L_Pin);
    rightLight = analogRead(light_R_Pin);
    if (leftLight > 450 && rightLight > 450) //the value detected by photo resistor, go forward   //was 650 for all
    {  
     GoForward(rSpeedSlow,lSpeedSlow);  
    } 
    else if (leftLight > 450 && rightLight <= 450)  //the value detected by photo resistor, turn left
    {
      GoLeft(rSpeedSlow,lSpeedSlow);  
    } 
    else if (leftLight <= 450 && rightLight > 450) //the value detected by photo resistor, turn right
    {
      GoRight(rSpeedSlow,lSpeedSlow);  
    } 
    else  //other situations, stop
    {
      Brake(rSpeedStop,lSpeedStop);
    }
    if (Serial.available())
    {
      bluetoothRecValue = Serial.read();
      if (bluetoothRecValue == 'P') 
      {
        flag = 1;
      }
    }  
  }  
}


//-----------------------------------------------------------------------------------------------------------------------------------
//Mouth() - Animated Mouth Mode
//-----------------------------------------------------------------------------------------------------------------------------------
void Mouth() 
{
  Brake(rSpeedStop,lSpeedStop);  //Make sure robot starts in stoped position
  
  flag = 0;      //the design that enter obstacle avoidance function
  while (flag == 0) 
  {
    
    int randomDelay = random(10, 2000); 
    delay(200);
    matrix_display(mouthClose); 
    delay(randomDelay);
    matrix_display(mouthHalfOpen); 
    delay(80);
    matrix_display(mouthOpen);

    if (Serial.available())
    {
      bluetoothRecValue = Serial.read();
      if (bluetoothRecValue == 'P')  //receive P
      {
        flag = 1;  //Exit the mode.
      }
    }
  }  
}

//-----------------------------------------------------------------------------------------------------------------------------------
//CheckDistance() - Ultrasonic Sensor for Distance Measuring
//-----------------------------------------------------------------------------------------------------------------------------------
float CheckDistance() 
{
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  float distance = pulseIn(Echo, HIGH) / 58.00;  //58.20 means 2*29.1=58.2
  delay(10);
  return distance;
}

//-----------------------------------------------------------------------------------------------------------------------------------
//MoveHead() - Move the robot's head (servo)
//-----------------------------------------------------------------------------------------------------------------------------------
void MoveHead(int myangle) 
{
  int pulsewidth;
  for (int i = 0; i <= 50; i = i + (1)) 
  {
    pulsewidth = myangle * 11 + 500;
    digitalWrite(servoPin,HIGH);
    delayMicroseconds(pulsewidth);
    digitalWrite(servoPin,LOW);
    delay((20 - pulsewidth / 1000));
  }
}

//-----------------------------------------------------------------------------------------------------------------------------------
//CheckBTChar() - Bluetooth Character Detect and Check
//-----------------------------------------------------------------------------------------------------------------------------------

void CheckBTChar()
{
  if (Serial.available())
  {
    bluetoothRecValue = Serial.read();
    Serial.println(bluetoothRecValue);
  }
  
  switch (bluetoothRecValue) 
  {
    case '1': 
      GoForward(rSpeedFast,lSpeedFast);
      break;
    case '3':  
      GoBackward(rSpeedFast,lSpeedFast);
      break;
    case '4': 
      GoLeft(rSpeedFast,lSpeedFast);
      break;
    case '2':  
      GoRight(rSpeedFast,lSpeedFast);
      break;
    case 'P': 
      Brake(rSpeedStop,lSpeedStop);
      break;
   case 'Q':
      FollowDistance();
      break;
   case 'R':
      AvoidObstacles();
      break;
   case 'S':
      FollowLight();
      break;
   case 'A':
      Mouth();
      break;
  }
}

//-----------------------------------------------------------------------------------------------------------------------------------
//MatrixDisplay() - 8 x 16 LED Display
//-----------------------------------------------------------------------------------------------------------------------------------
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
//Convey data
void IIC_send(unsigned char send_data)
{
  for(char i = 0;i < 8;i++)  //Each byte has 8 bits
  {
      digitalWrite(SCL_Pin,LOW);  //Pull down clock pin SCL Pin to change the signals of SDA
      delayMicroseconds(3);
      if(send_data & 0x01)        //Set high and low level of SDA_Pin according to 1 or 0 of every bit
      {
        digitalWrite(SDA_Pin,HIGH);
      }
      else
      {
        digitalWrite(SDA_Pin,LOW);
      }
      delayMicroseconds(3);
      digitalWrite(SCL_Pin,HIGH);  //Pull up clock pin SCL_Pin to stop transmitting data
      delayMicroseconds(3);
      send_data = send_data >> 1;  //Detect bit by bit, so move the data right by one
  }
}

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

//-----------------------------------------------------------------------------------------------------------------------------------
//Tank Drive Motor Funtions
//-----------------------------------------------------------------------------------------------------------------------------------
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
void Brake(int rightSpeed, int leftSpeed)
{
  matrix_display(brake); 
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
