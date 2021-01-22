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
//
inline void drawLine(float x0f, float y0f, float x1f, float y1f)
{
       int x0 = (x0f * OLED_WIDTH /2) + (OLED_WIDTH /2);
       int x1 = (x1f * OLED_WIDTH /2) + (OLED_WIDTH /2);
       int y0 = (y0f * OLED_HEIGHT/2) + (OLED_HEIGHT/2);
       int y1 = (y1f * OLED_HEIGHT/2) + (OLED_HEIGHT/2);
       oledSetLine(x0,y0,x1,y1,WHITE);
}

//
// Macro to rotate a point about a certain roll angle where the cos and sin of that angle
// are precomputed arguments as is the pitch in units.
//
inline void rotatePoint(float *x, float*y, float croll, float sroll, float pitch)  
{    
       float x2 = *x * croll - *y * sroll;       
       float y2 = *x * sroll + *y * croll;       
       *x = x2; 
       *y = y2 + pitch;             
}

inline void drawRotateShiftedLine(float x0, float y0, float x1, float y1, float croll, float sroll, float pitch)
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
         drawLine(-1.0, -1.0, 1.0,  1.0);        // Draw an X on screen
         drawLine(-1.0,  1.0, 1.0, -1.0); 
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
         int sampleMs = debug_g ? 500 : 20;          // longer samples in debug mode.
         if (sample(sampleMs, &ox, &oy, &oz, &ot)) { // Sample orientations etc. for 100ms.
             oledClearDisplay(BLACK);                // Start with a clean white display
             //
             drawLine(-0.5,  -0.1, -0.15, -0.1);     // Draw the little plane. 
             drawLine(-0.15, -0.1,  0.0,   0.1);     // basically a --V--- symbol.
             drawLine( 0.0,   0.1,  0.15, -0.1);      
             drawLine( 0.15, -0.1,  0.5,  -0.1);    
             //
             int   yaw   = (int) (ox + 0.5);          // round up yaw in degrees.
             int   tsla  = (int) (ot + 0.5);
             char  buf[10];                           // will print here as 090 etc.
             float roll  = (-oy * M_PI) / 180.0;      // convert roll to radians
             float pitch = ( oz * M_PI) / 180.0;      // convert pitch to radians.
             bool  isUp  = (oz < 90) && (oz > -90);   // upright ?
             bool  perSecEvt = ((millis()/1000)&1)==1;// Every second this is true
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
             if (perSecEvt) {                        // Test the sensor every second.
                  uint8_t sys, gyro, accel, mag;
                  flash = '-';
                  bno.getCalibration(&sys,&gyro,&accel,&mag);
                  if (gyro == 0) {
                      drawLine(-1.0, -1.0, 1.0,  1.0);// Draw an X on screen if gyro not ready
                      drawLine(-1.0,  1.0, 1.0, -1.0); 
                  }
                  if (mag == 0)                       // Draw line through heading 
                      xOutHeading = true;
             }
             //
             sprintf(buf,"%c%03d%c", flash, yaw, flash );// Output yaw as 090 to top
             oledSetStr(buf, OLED_WIDTH/2-13, 1, WHITE);
             if (xOutHeading) {
                 drawLine(-0.3, -1.0 , 0.3, -0.70);
                 drawLine(-0.3, -0.70, 0.3,  -1.0);
             }
             //
             //  Draw the horizon and ground lines leading toward it rotated and shifted according to pitch and 
             //  roll. We treat the inverted situation differently to simplifly the math and avoid 3d projectsion
             //  etc.
             //
             if (isUp) {                      
                 // Upright ground image 
                 float croll = cos(roll);                // precompute some trig.
                 float sroll = sin(roll);            
                 drawRotateShiftedLine(-2.5, 0.0, 2.5, 0.0, croll, sroll, pitch);   // Draw Horizon
                 drawRotateShiftedLine(-0.60,2.5,-0.25,0.0, croll, sroll, pitch);   // Draw lines orthogonal(ish)
                 drawRotateShiftedLine( 0.60,2.5, 0.25,0.0, croll, sroll, pitch);   // to horizon
             } else {
                 // Inverted ground image.
                 float croll = cos(-roll);                // display rolls is backwards when inverted.
                 float sroll = sin(-roll);
                 if (pitch > 0) 
                     pitch = -M_PI + pitch;
                 else
                     pitch =  M_PI + pitch;               
                 drawRotateShiftedLine(-2.5,  0.0, 2.5, 0.0, croll, sroll, pitch);  // Draw Horizon
                 drawRotateShiftedLine(-0.60,-2.5,-0.25,0.0, croll, sroll, pitch);  // Draw lines orthogonal(ish)
                 drawRotateShiftedLine( 0.60,-2.5, 0.25,0.0, croll, sroll, pitch);  // to horizo (but inverted view)
             }
             //
             oledUpdateDisplay();
         } else {
             bnoState_g = false;
         }
     }
}
