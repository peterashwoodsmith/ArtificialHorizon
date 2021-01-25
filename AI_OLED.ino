
// This is a very simple first attempt at using an arduino UNO to drive a Waveshare 128x128 OLED 
// taking data from a BLO055 gyroscope. It draws a very simple artificial horizon and also the 
// heading which is calibrated against the magnetic flux detector in the BL055 to try to track 
// true north without having to be hand set.
//
// The horizon which is drawn as three lines, one the actual horizon and two perspective lines that 
// join it from below. Then a little aircraft figure is drawn like an inverted gull wing --V--. 
// The aircraft figure remains stationary in the display just as the wings would remain stationary to
// a pilot, however the horizon is rotated with roll and pitched with pitch to mimic an actual 
// external view. 
// 
// In addition a magnetic headig is display at the top from 0..360 and an attempt is made to slave
// it to the magnetic flux detector so that it finds true north.
//
// This is experimental version 1.0 for educational purposes only and should not be used for actual
// flight reference. A real artificial horizon requires many things that this does not have not least
// of which is hardware which can work a wide variety of temperatures and vibration. 
//
// The BNO055 is wired as per the default Adafruit documenation.
//
// The OLED however is wired as per the OLED_functions.h include file. There are some tricks with 
// the OLED because the UNO does not have neough memory to draw the image on the UNO before sending
// it to the OLED. As a result we draw the image as 128x64 and then repeat each line when we draw it.
// We also ignore the grey scale and only write 1 bit per pixel. THis allows us to 'just' squeeze this
// program and its data on the UNO.
//
// THis software is provided as is, and is freely usable by anybody with no warranty or liability by
// the author. BY all means copy this code but please maintain this disclaimer so that nobody uses this 
// in a real aircraft and gets hurt.
//
// Peter Ashwood-Smith - 2021 - Lockdown version 1.0

#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "OLED_Functions.h"

//
// Trivial Gyroscope/Accellerometer/Magnetic compass chip. We keep a separate state flag
// and will try to restart it if it does not respond while we are using it.
//
Adafruit_BNO055 bno        = Adafruit_BNO055(55);
bool            bnoState_g = false;                 

//
// Debug flag, allows tracing, if we turn it off compiler should remove unused code.
//
const bool      debug_g = false;

//
// Rather than use floating point we use integers multiplied by 100
//
const int       precis_g = 100;

// 
// To get started all we need to do is attach out interrupt handler to the rising edge on PIN 2
// adjust the display brighness and make sure interrupts are disabled before we start sampling.
//
void setup() 
{   
     if (debug_g == true) 
         Serial.begin(9600);
     oledBegin();
     bnoState_g = false;
}

//
// Sample the three access accelerometer and one access of the magnemometer for heading.
//
inline void sampleBNO(float *ox, float *oy, float *oz, float *ot)
{
     imu::Vector<3> imudata;
     //
     imudata = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
     *ox = imudata.x();
     *oy = imudata.y();
     *oz = imudata.z();
     //
     imudata = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
     *ot = imudata.x();
}

//
// Sample the sensors for ms ms. We return the actual time we used to sample.
// During the sample interval we either count interrupts, or using the analog 
// port we attempt to count transitions. If we get all zeros on every sample we 
// assume the sensor is dead, this will trigger an attempt to reset it.
//
inline bool sample(unsigned long ms, float *ox, float *oy, float *oz, float *ot)
{        
     unsigned long t_end, t_now, t_start, t_dur, count;
     t_dur = ms * 1000;
     t_start = micros();
     t_end = t_start + t_dur; 
     t_now = t_start;
     count = 0;
     *ox = *oy = *oz = *ot = 0;
     float up = 0.0;
     while(1) {
          count += 1;
          float ix,iy,iz,it; 
          sampleBNO(&ix,&iy,&iz,&it);
          *ox += ix;
          *oy += iy;
          *oz += iz;
          *ot += it;
          up += (ix + iy + iz +it);
          t_now = micros();
          if ((t_start < t_end)  &&(t_now >= t_end))   break;
          if ((t_end   < t_start)&&(t_now >= t_start)) break;
     }
     *ox /= count;
     *oy /= count;
     *oz /= count;
     *ot /= count;
     return(up != 0.0);
}

//
// Assuming the display is a normalized -1.0 --> 1.0 with 0,0 in the center
// We draw a line. We must scale it and shift it to from the normalized coords.
// We use a fixed point number / 1000.
//
inline void drawLine(int x0f, int y0f, int x1f, int y1f)
{
       int x0 = (x0f * OLED_WIDTH /2) /precis_g + (OLED_WIDTH /2);
       int x1 = (x1f * OLED_WIDTH /2) /precis_g + (OLED_WIDTH /2);
       int y0 = (y0f * OLED_HEIGHT)/2 /precis_g + OLED_HEIGHT /2;
       int y1 = (y1f * OLED_HEIGHT)/2 /precis_g + OLED_HEIGHT /2;
       oledSetLine(x0,y0,x1,y1,WHITE);
}

//
// Macro to rotate a point about a certain roll angle where the cos and sin of that angle
// are precomputed arguments as is the pitch in units.
//
inline void rotatePoint(int *x, int *y, int croll, int sroll, int pitch)  
{    
       int x2 = (*x * croll - *y * sroll) / precis_g;       
       int y2 = (*x * sroll + *y * croll) / precis_g;       
       *x = x2; 
       *y = y2 + pitch;             
}

//
// This will draw a line subject to a rotation and shift. To save on trig funcion calls we take the
// sin and cos of the angle to rotate as arguments however the pitch is just a simple addition. This is
// not a true 3d rendering of the horizon as that would exceed the capabilities of the UNO to compute in
// real time but it does a good job emulating what you'd see with a 2d rotation and shift.
//
inline void drawRotateShiftedLine(int x0, int y0, int x1, int y1, int croll, int sroll, int pitch)
{
       rotatePoint(&x0,&y0,croll,sroll,pitch);        
       rotatePoint(&x1,&y1,croll,sroll,pitch);
       drawLine(x0,y0,x1,y1);          
}
  
  
//
// The loop just performs 1/2 second of sampling after which is prints three G numbers. If for some reason
// the device is not active it keeps trying to start it.
//
void loop()     
{    //
     // We keep track of where the minimum and maximum magnetic field has been seen relative to the
     // actual gyro yaw value. We then use this to make a correction for magnetic heading in the display.
     // This means it will only be accurate if it sees strong ut North while relatively level.
     //
     static float max_ot = -1000; static int max_ot_deg = 180;
     // If we are down, try to go up but draw an X.
     if (bnoState_g == false) {
         oledClearDisplay(BLACK);
         drawLine(-100, -100, 100,  100);            // Draw an X on screen
         drawLine(-100,  100, 100, -100); 
         oledUpdateDisplay();
         delay(1000);
         if (debug_g) Serial.println("Re Init BNO");
         bnoState_g = bno.begin();
         if (bnoState_g == true) {
             if (debug_g == true) {
                 sensor_t sensor;
                 bno.getSensor(&sensor);   
                 Serial.print  ("Sensor:"); Serial.println(sensor.name);
             }
             bno.setExtCrystalUse(true);
             delay(1000);
         }
     } else {
         float ox,  oy,  oz, ot;
         char buf[32];
         int sampleMs = debug_g ? 500 : 10;          // longer samples in debug mode.
         if (sample(sampleMs, &ox, &oy, &oz, &ot)) { // Sample orientations etc. for 100ms.
             oledClearDisplay(BLACK);                // Start with a clean white display
             //
             drawLine(-50, -10, -15, -10);           // Draw the little plane. 
             drawLine(-15, -10,   0,   10);          // basically a --V--- symbol.
             drawLine( 0,   10,  15,  -10);      
             drawLine( 15, -10,  50,  -10);    
             //
             int   yaw   = (int) (ox + 0.5);          // round up yaw in degrees.
             int   tsla  = (int) (ot + 0.5);
             char  buf[10];                           // will print here as 090 etc.
             float roll  = (-oy * M_PI) / 180.0;      // convert roll to radians
             float pitch = ( oz * M_PI) / 180.0;      // convert pitch to radians.
             float cosroll = cos(roll);
             float sinroll = sin(roll); 
             bool  isUp  = (oz < 90) && (oz > -90);   // upright ?
             bool  perEvt = ((millis()/1000)%2)==0;   // Every second this is true
             bool  xOutHeading = false;               // will set if mag not good.
             char  flash = ' ';                       // normally spaces besides heading
             if (debug_g) {
                 Serial.println("------"); Serial.println(oy); Serial.println(oz);
             }
             //
             // If we are relatively level look at the magnetic field strength for min
             // max values and at what heading we saw them. Then correct the yaw value
             // so that 360 corresponds to strongest magnetic field strength.
             //
             if ((oy < 5) && (oy > -5) && (oz < 5) && (oz > -5) && (ot > max_ot)) {
                  max_ot = ot; 
                  max_ot_deg = ox; 
             }
             if (yaw > max_ot_deg)                   // Try to correct the yaw angle
                 yaw -= max_ot_deg;                  // so that North is the max mag
             else                                    // strength position seen so far.
                 yaw = 360 - (max_ot_deg - yaw);     // Will continuously try to optimize.
             //
             if (perEvt) {                        // Test the sensor every second.
                  uint8_t sys, gyro, accel, mag;
                  flash = '-';
                  bno.getCalibration(&sys,&gyro,&accel,&mag);
                  if (gyro == 0) {
                      drawLine(-100, -100, 100,  100);// Draw an X on screen if gyro not ready
                      drawLine(-100,  100, 100, -100);                  }
                  if (mag == 0)                       // Draw line through heading 
                      xOutHeading = true;
             }
             //
             sprintf(buf,"%c%03d%c", flash, yaw, flash );// Output yaw as 090 to top
             oledSetStr(buf, OLED_WIDTH/2-13, 1, WHITE);
             if (xOutHeading) {
                 drawLine(-30, -100 , 30, -70);
                 drawLine(-30, -70,   30, -100); 
             }
             //
             //  Draw the horizon and ground lines leading toward it rotated and shifted according to pitch and 
             //  roll. We treat the inverted situation differently to simplifly the math and avoid 3d projectsion
             //  etc.
             //
             if (isUp) {                      
                 // Upright ground image 
                 int croll = cosroll * precis_g;                                 // precompute some trig.
                 int sroll = sinroll * precis_g;  
                 int ipitch = pitch * precis_g;          
                 drawRotateShiftedLine(-250, 0,   250, 0, croll, sroll, ipitch);   // Draw Horizon                
                 drawRotateShiftedLine(-60,  250,-25,  0, croll, sroll, ipitch);   // Draw lines orthogonal(ish)
                 drawRotateShiftedLine( 60,  250, 25,  0, croll, sroll, ipitch);   // to horizon                        
              } else {
                 // Inverted ground image.
                 int croll =   cosroll * precis_g;         // display rolls is backwards when inverted.
                 int sroll = - sinroll * precis_g;
                 if (pitch > 0) 
                     pitch = -M_PI + pitch;
                 else
                     pitch =  M_PI + pitch; 
                 int ipitch = pitch * precis_g;      
                 drawRotateShiftedLine(-250,   0, 250, 0, croll, sroll, ipitch);   // Draw Horizon
                 drawRotateShiftedLine(-60, -250, -25, 0, croll, sroll, ipitch);   // Draw lines orthogonal(ish)
                 drawRotateShiftedLine( 60, -250,  25, 0, croll, sroll, ipitch);   // to horizo (but inverted view) 
              }
             //
             oledUpdateDisplay();
         } else {                            // Want to modify so that sampling detects failure and draws X
             bnoState_g = false;             // then tries to re-initialize. 
         }
     }
}
