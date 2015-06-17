//# Automated-RC-plane by Shivank Kumar n Atul Saurav
//#ITSP project 2015. TEAM Vortex.
//It contains the final code for our attitude holding RC plane..
//library used


#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//MPU variables
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO
#define LED_PIN 13
#define PIN 3

bool blinkState = false;
Servo sr,sp,sy;

void setup() {
//I2C COMMUNICATION SETUP BETWEEN ARDUINO AND MPU 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
Fastwire::setup(400, true);
#endif
    //SERIAL COMMUNICATION BETWEEN ARDUINO AND COMP
Serial.begin(57600);
Serial.println("Initializing I2C devices...");
accelgyro.initialize();
Serial.println("Testing device connections...");
Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
pinMode(LED_PIN, OUTPUT);
pinMode(PIN,INPUT);

sr.attach(9);
sp.attach(10);
sy.attach(11);
}
float pitch,roll;
int cnt=0;
float CV,er,er0=0,sumr=0,pitch0=0,yaw0=0;
float ep,ep0,sump=0;
float ey,ey0,sumy=0;
float kp=1;
float ki=0.0005;
float kd=0.05;
float net=0;
int out;

void loop() {
//GETTING RAW READING
accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
#ifdef OUTPUT_READABLE_ACCELGYRO

//CONVERTING IN STANDARD UNIT
float axx=(ax*9.8)/16384.0;
float ayy=(ay*9.8)/16384.0;
float azz=(az*9.8)/16384.0;
float gxx=gx/131.0;
float gyy=gy/131.0;
float gzz=gz/131.0;


net=sqrt(axx*axx+ayy*ayy+azz*azz);//RESULTANT READING OF ACCELERO
if(net!=0){
pitch=(ayy*90)/net;
roll=(axx*90)/net;}
else{pitch=0;roll=0;}

//RECEIVER PWM READING
out=pulseIn(PIN,HIGH,20000);//using one of the channels to switch between automated controls and manual controls.

Serial.print("int value");
Serial.print(out);Serial.print("\t");
Serial.print("roll ");
Serial.print(roll);Serial.print("\t");
Serial.print("pitch ");
Serial.print(pitch);Serial.print("\t");
Serial.print("omega z ");
Serial.println(gzz);

//PID CONTROL PART
if(out>1500){
 cnt+=1;                   // Automated control begins when channel 5 on receiver is high
 if(cnt==1)
 {
    pitch0=pitch;     //sets the heading to follow and maintain pitch stability along this angle  
    yaw0=yaw;         //sets the heading to follow and maintain yaw stability along this angle  
 }
digitalWrite(8,LOW);  // Using the Receiver's PWM signal for controls and this output pin from arduino, we operate an AND 
sp.attach(9);         // gate to switch between manual n automatic controls, in automated mode receiver's signal can't pass.
sr.attach(10);
sr.attach(11);

//PID roll control
er=0-roll;
sumr=sumr+er;
CV=90-(kp*er+ki*sumr+kd*(er-er0));
sr.write(CV);
Serial.print(CV); Serial.print("\t");

//pitch control
ep=pitch0-pitch;
sump=sump+ep;
CV=90+(kp*ep+ki*sump+kd*(ep-ep0))*1.5;
sp.write(CV);
Serial.print(CV); Serial.print("\t");

//YAW control
ey=yaw0-gzz;
sumy=sumy+ey;
CV=90-(kp*ey+ki*sumy+kd*(ey-ey0));
sy.write(CV);
Serial.println(CV);


er0=er;
ep0=ep;
ey0=ey;
}

else
{ 
sp.detach();        //now this is the manual mode as the PWM reading of channel 5 is less than 1500
sr.detach();        //detaching the servo pin from arduino to use it as normal output pin for operating the OR gate
sy.detach();
digitalWrite(8,HIGH);
digitalWrite(9,LOW);
digitalWrite(10,LOW);
digitalWrite(11,LOW);



#endif
#ifdef OUTPUT_BINARY_ACCELGYRO

 
Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    
#endif
 
blinkState = !blinkState;
digitalWrite(LED_PIN, blinkState);

}
