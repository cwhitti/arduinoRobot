/*
keyestudio 4wd BT Car V2.0
Line Tracking Robot
This code controls a robot that can follow a black line using line tracking sensors
*/ 

// OPTIONAL: define an array to store the hexadecimal for the LED board light display, obtained from the online tool

// define the pins for LED board
#define SCL_Pin  A5 
#define SDA_Pin  A4 

// define pins for controlling the motors

  // define motor control pins for motors A and B 
  #define powerMotorA 2 // right Motor
  #define powerMotorB 4 // Left motor
  
  //define PWM pins for motor A and B
  #define speedMotorA 6 // controls speed
  #define speedMotorB 5 // controls speed 

// define pins for line tracking sensors
  #define leftSensor 11
  #define centerSensor 7
  #define rightSensor 8

  //default speed
  # define turningSpeed 200
  # define delayVal 0
  # define frontSpeed 100


unsigned char ciaArray[] =
{
0x00, 0x3c, 0x42, 0x24, 0x00, 0x00, 0x42, 0x7e, 0x42, 0x00, 0x00, 0x7c, 0x12, 0x7c, 0x00, 0x00
};

// define any other variables you may need

// This function is called when the Arduino board is powered on
void setup() {

  // Initialize serial communication with a baud rate of 9600
  Serial.begin(9600);

  // Set all the pins to input or output
  // Motors
  pinMode(powerMotorA, OUTPUT);
  pinMode(powerMotorB, OUTPUT);
  pinMode(speedMotorA, OUTPUT);
  pinMode(speedMotorB, OUTPUT);

  // Sensors
  pinMode(leftSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightSensor,INPUT);

  // LEDs
  pinMode(SCL_Pin, OUTPUT);
  pinMode(SDA_Pin, OUTPUT);
}


// This function is called repeatedly after the setup()
void loop() 
{
  // Run the main tracking program
  tracking();
  matrix_display(ciaArray);
}

//  The tracking() function reads the values of the line tracking sensors 
// and determines the robot's movement based on the sensor readings.
void tracking()
{
  // Get the values of line tracking sensors
    // function: analogRead(rightSensor)
   int centerSensorVal = digitalRead(centerSensor);
   int rightSensorVal = digitalRead(rightSensor);
   int leftSensorVal = digitalRead(leftSensor);

    // use functions front(), back(), left(), right(), and Stop() as needed
  if (centerSensorVal==1)
  {
    front();
  }
  else if (rightSensorVal == 1 && leftSensorVal ==0)
  {
    right();
  }

  else if ( leftSensorVal == 1 && rightSensorVal==0)
  {
    left();
  }
  else
  {
   Stop();
  }
}

void front()
{
  // make the robot go forward 
  digitalWrite( powerMotorA, HIGH );
  analogWrite( speedMotorA, frontSpeed );
  
  digitalWrite( powerMotorB, HIGH );
  analogWrite( speedMotorB, frontSpeed );
  
  delay(delayVal);   
} 
void back()
{
  // make the robot go back 
}
void left()
{
  // make the robot go left 
  digitalWrite( powerMotorA, HIGH );
  analogWrite( speedMotorA, turningSpeed );
  
  digitalWrite( powerMotorB, LOW );
  analogWrite( speedMotorB, turningSpeed );
  delay(delayVal);
    
}
void right()
{
  // make the robot go forward 
  digitalWrite( powerMotorA, LOW );
  analogWrite( speedMotorA, turningSpeed );
  
  digitalWrite( powerMotorB, HIGH );
  analogWrite( speedMotorB, turningSpeed );
  delay(delayVal);
}
void Stop()
{
  digitalWrite( powerMotorA, LOW);
  analogWrite( speedMotorA, 0);
  digitalWrite( powerMotorB, LOW);
  analogWrite( speedMotorB, 0);
}


// The matrix_display() function is used for LED dot matrix display (optional but encouraged). 
// It sends pattern data to the display using I2C communication.
void matrix_display(unsigned char matrix_value[])
{
  // Start the I2C data transmission
  IIC_start();
  
  // Select and send the I2C address (0xc0) for the LED matrix
  IIC_send( 0xc0 );
  
  // Loop through the pattern data array, the pattern data consists of 16 bytes
  for( int index = 0; index < 16; index++ )
    {
    // Send the current pattern data byte to convey the pattern
    IIC_send( matrix_value[ index ] );
    }
  // End the transmission of pattern data
  IIC_end();
  // Start a new I2C transmission to configure the display control
  IIC_start();
  // Send display control command to set pulse width to 4/16 (0x8A)
  IIC_send( 0x8A );
  // End the transmission
  IIC_end();
}



// The following functions are for I2C communication used for the LED dot matrix display.
// These functions handle data transmission and control signals.
// These functions are complete and should be used when coding the matrix display function.

// This function will start data transmission 
void IIC_start()
{
  // Pull SCL and SDA lines high
  digitalWrite(SCL_Pin,HIGH);
  delayMicroseconds(3);
  digitalWrite(SDA_Pin,HIGH);
  delayMicroseconds(3);

  // Pull SDA low while SCL is high to start communication
  digitalWrite(SDA_Pin,LOW);
  delayMicroseconds(3);
}
// This function will transmit data
void IIC_send(unsigned char send_data)
{
  // Loop through each bit of the data byte (8 bits in total)
  for(char i = 0;i < 8;i++)  //Every character has 8 bits
  {
    // Pull SCL low to start changing SDA signal
    digitalWrite(SCL_Pin,LOW);  //pull down the SCL_Pin to change the signal of SDA
    delayMicroseconds(3);

    // Check the current bit of send_data (rightmost bit first)
    if(send_data & 0x01)  //1 or 0 of byte  is used to set high and low level of SDA_Pin
    {
      // If the bit is 1, pull SDA high
      digitalWrite(SDA_Pin,HIGH);
    }
    else
    {
      // If the bit is 0, pull SDA low
      digitalWrite(SDA_Pin,LOW);
    }
    delayMicroseconds(3);

    // Pull SCL high to transmit the bit
    digitalWrite(SCL_Pin,HIGH); //Pull up SCL_Pin to stop data transmission
    delayMicroseconds(3);

    // Move to the next bit of send_data
    send_data = send_data >> 1;  //Detect bit by bit, so move the data right by one bit
  }
}
// This function will end the data transmission
void IIC_end()
{
  // Pull SCL low to start changing SDA signal
  digitalWrite(SCL_Pin,LOW);
  delayMicroseconds(3);

  // Pull SDA low to indicate the end of communication
  digitalWrite(SDA_Pin,LOW);
  delayMicroseconds(3);

  // Pull SCL high while SDA is low
  digitalWrite(SCL_Pin,HIGH);
  delayMicroseconds(3);

  // Pull SDA high to complete the end of communication
  digitalWrite(SDA_Pin,HIGH);
  delayMicroseconds(3);
}
