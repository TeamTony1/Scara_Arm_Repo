#include "mbed.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>

// Definitions of constansts
#define PI 3.141592
#define WAIT_1S 1000000
#define WAIT_2S 2000000
#define A2 12 // first link length
#define A4 17 // Second link length 
#define UART_BAUD 9600
#define WAIT_3S 3000000
#define WAIT_5S 5000000
#define WAIT_6S 6000000
#define WAIT_500MS 500000
#define I2C_FREQUENCY 100000
#define MOTOR_PERIOD 20

// initialisation of variables
int distance;
float cord1, cord2;
float angle1, angle2;
uint16_t readVal;
const int I2C_ADDRESS = 0x052 << 1;

// Declaration of functions
float setAngle(float angle) ;
void calAngle(float x, float y);
float rad2deg(float rad);
void ReadCord(void);
void home(void);
void pick(void);
int16_t RangeRead(void);
// void buzz(void);


// Instansiation of objects 
BufferedSerial uart(USBTX, USBRX);
I2C rangeSensor (PTC9, PTC8);
PwmOut joint1(PTD0);
PwmOut joint2(PTD5);
DigitalOut dcForward(PTB1);
DigitalOut dcReverse(PTB2);
DigitalOut magnent(PTA12);
DigitalOut redLed(PTD7);
DigitalOut yellowLed(PTD6);
DigitalOut greenLed(PTE31);
DigitalOut buzzer(PTA17);


enum state{
    off,
    on
};



int main()
{
    joint1.period_ms(MOTOR_PERIOD); // set the joint period to 20ms and frequency to 50Hz
    joint2.period_ms(MOTOR_PERIOD);

    uart.set_baud(UART_BAUD);  // set the uart baudrate to 9600
    rangeSensor.frequency(I2C_FREQUENCY); // set the frequency of the I2C to 100KHz

    redLed.write(off); // turn on red led and the rest is turned off 
    yellowLed.write(on);
    greenLed.write(on);


    while(1)
    { 


        if (uart.readable()) // check if uart is readable
        {

            DigitalOut led(LED1); // the red led on the kl25z board flashes when the is data available
            led.write(off);

            greenLed.write(off); // turn on the green led to indicate that the robot is in motoin
            yellowLed.write(on); 
            wait_us(WAIT_500MS);
            led.write(on);
            wait_us(WAIT_500MS);

            ReadCord(); // read the coordinates from the PC 

            calAngle(cord1, cord2); // calculate the angles to be moved by the various joints using inverse kinematics 

            if(angle1 < 0) // offset the first joint be 90degrees to compensate for the negative PWM value
            {
                angle1 = angle1 + 90;
            }else{
                angle1 = 90 + angle1;
            }

            joint1.write(setAngle(angle1)); // calculate the required PWM value and write to the joints
            wait_us(WAIT_2S);
            joint2.write(setAngle(angle2));
            wait_us(WAIT_2S);

            pick(); // pick object

            home(); // place object in the home coordinate
        }
        
    }

}


float setAngle(float angle) // calculate the require PWM value for the servo motor
{
    float y = 0.0;
    y = (0.00054*angle) + 0.0175;
    return y;
}

void calAngle(float x, float y)  // Inverse kinematics calculation for joint variables
{
    float r, phi1, phi2, phi3, theta1, theta2; 
    if( (x*x) + (y*y) >= 0 && ((x*x) + (y*y)) <= ((A2*A2) + (A4*A4) + (2*A2*A4)) )
    {
        r = sqrt((x*x) + (y*y));
        phi1 = acos(((r*r) + (A2*A2) - (A4*A4)) / (2*A2*r));
        phi2 = atan(y/x);
        theta1 = phi2 - phi1;
        phi3 = acos(((A2*A2) + (A4*A4) - (r*r)) / (2*A2*A4));
        theta2 = PI - phi3;

        angle1 = rad2deg(theta1);
        angle2 = rad2deg(theta2);
        
    }
}

float rad2deg(float rad) // comverts radian to degrees
{
    return(rad * (180 / PI));
}


void ReadCord(void) // read the x and y coordinates from the pc through UART
{
    char buffer[64];  // Create a character buffer to store incoming data
    int buffer_index = 0;

    while (uart.readable()) {
            char c;
            uart.read(&c, 1);  // Read one character at a time

            if (c == '\n') {
                // End of line, parse the received data as floats using atof
                buffer[buffer_index] = '\0';  // Null-terminate the string

                // Use atof to convert the string to a float
                cord1 = atof(buffer);

                // Find the end of the first float value in the buffer
                char *value2_str = strchr(buffer, ' ');
                if (value2_str != nullptr) {
                    // Move to the next character and convert to float
                    cord2 = atof(value2_str + 1);

                    // Reset the buffer index for the next data
                    buffer_index = 0;
                }
            } else {
                // Store the character in the buffer
                buffer[buffer_index++] = c;
            }
   }

}

void home(void) // move the joints(motors) to to home positions
{
    greenLed.write(on); // turn yellow led on to indicate the the robot is in the home position
    yellowLed.write(off);
    
    joint1.write(setAngle(90));
    wait_us(WAIT_2S);
    joint2.write(0.018);
    wait_us(WAIT_2S);
    magnent.write(off); // turn off magnet at the end-effector to dro the object

}

void pick(void) // pick object 
{
     while(readVal > 2) // check if the distance between the end-effector and the object is greater than 2mm then move the end-effector down
     {
         dcForward.write(on);
         dcReverse.write(off);
         readVal = RangeRead();

       // printf("distance is: %u\n", readVal);
     }
    
    // dcForward.write(on); // moves the end-effector down
    // dcReverse.write(off);
    // wait_us(WAIT_6S);

     dcForward.write(off); // activates magnet to pick object
     dcReverse.write(off);
     magnent.write(on); 
     wait_us(WAIT_2S);

    // dcForward.write(off); // moves the end-effector up
    // dcReverse.write(on);
    // wait_us(WAIT_5S);

     while(readVal < 150) // check if the distance between the end-effector and the object is less than 150mm then move the end-effector up
     {  
         dcForward.write(off);
         dcReverse.write(on);
         readVal = RangeRead();

         // printf("distance is: %u\n", readVal);
     }

    // stops the dc motor
    dcForward.write(off); 
    dcReverse.write(off);
    wait_us(WAIT_1S);

}

int16_t RangeRead(void) // reads the distance the end-effector has to travel from the range sensor
{
    char command= 0x00;
    rangeSensor.write(I2C_ADDRESS, &command, 1);
    wait_us(30000);
    char data[2]; 
    rangeSensor.read(I2C_ADDRESS, data, 2);
    int16_t distance = ((data[0] << 8) | data[1]);
    return distance;
}


// void buzz(void) // turns on the buzzer
// {
//     buzzer.write(on);
//     wait_us(WAIT_500MS);
//     buzzer.write(off);
//     wait_us(WAIT_500MS);

// }
