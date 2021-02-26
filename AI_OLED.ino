
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
// This is experimental version 2.0 for educational purposes only and should not be used for actual
// flight reference. A real artificial horizon requires many things that this does not have not least
// of which is hardware which can work a wide variety of temperatures and vibration. 
//
// The BNO055 is wired as per the default Adafruit documenation.
//
// The OLED however is wired as per the OLED_functions.h include file. There are some tricks with 
// the OLED because the UNO does not have neough memory to draw the image on the UNO before sending
// it to the OLED. As a result we draw the image as 128x128 but ignore the greyscale values.
//
// THis software is provided as is, and is freely usable by anybody with no warranty or liability by
// the author. BY all means copy this code but please maintain this disclaimer so that nobody uses this 
// in a real aircraft and gets hurt.
//
// Peter Ashwood-Smith - 2021 - Lockdown version 2.0

#include <SPI.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

//
// Code to control the 128x128 bit waveshare greyscale OLED.
//
#include "OLED_Functions.h"

//
// Trivial Gyroscope/Accellerometer/Magnetic compass chip. We keep a separate state flag
// and will try to restart it if it does not respond while we are using it. Also wire an
// External reset pin from pin 14 of Arduino to the RST on the BNO board.
//
Adafruit_BNO055            bno        = Adafruit_BNO055(55);
int                        bnoRstPin  = 12;
bool                       bnoState_g = false;  

//
// Debug flag, allows tracing, if we turn it off compiler should remove unused code.
//
const bool      debug_g    = false;
const bool      debugXYZ_g = false;   // show gyro raw x,y,z on screen
const bool      debugMag_g = true;

//
// Rather than use floating point we use integers multiplied by 100
//
const int       precis_g = 100;

#if     USE_WD
//
// If we have the TimerInterrupts available we set USE_WD to create the WD behavior.
// We use the TimerFunctions library to create a watch dog timer. I assume it hangs it off
// the existing interrupt handlers for timer2.
//
#define USE_TIMER_2 1
#include <TimerInterrupt.h>

//
// Watchdog timer - it ticks over making sure the loop() function is operating.
// If for some reason the loop() blocks then we will disable the display. This
// prevents the display from showing inaccurate IMU data which could be 
// catastrophic if its being relied on. 
//

volatile byte wdCounter_g;

void watchdog_isr()
{
     wdCounter_g += 1;
     if (wdCounter_g > 3) {                    // more than 2 seconds
         for(int i=0; i < 1000; i++) {         // force OLED off
              OLEDWrite(OLED_COMMAND,0xae);    // many times to be safe.
              digitalWrite(bnoRstPin, HIGH);   // signal the BNO to reset
         }                                     // Then either
         while(1) {                            // restart or hang.
            void (*f)(void)=0;
            (*f)();
         }   
     }
}

//
// This will set the internal timer to invoke the ISR above periodically. This allows
// us to detect a block in the loop() likely caused by the wire or spi library to a 
// device which has failed. In this case we can take action to warn the user, i.e.
// kill the display.
//
inline void watchdog_setup()
{    
     wdCounter_g = 0;
     ITimer2.init();
     if (! ITimer2.attachInterruptInterval(500, watchdog_isr))
          while(1);        
}

//
// Reset the watchdog counger, this is called in the main loop(s) to indicate that
// work is being done and to avoid being reset by the watchdog timeout.
//
inline void watchdog_reset()
{    
      wdCounter_g = 0;
}

#else

inline void watchdog_setup() { }    // No watchdog
inline void watchdog_reset() { }    // No watchdog

#endif

//
// THis function is called periodically during the loop to make sure that the chip is 
// responding properly. If for some reason the I2C dies or the chip stops talking we
// will get back garbage, or it will hang and the watchdog will catch it, but either
// way the entire CPU should restart. The first call will remember the info from the
// bno after it comes up successfully and subsequent calls compare against this.
//
inline bool bnoVerifyRevInfo()               
{                  
     static bool havelast = false;
     static Adafruit_BNO055::adafruit_bno055_rev_info_t last; 
      if (havelast) {        
          Adafruit_BNO055::adafruit_bno055_rev_info_t curr;
          bno.getRevInfo(&curr);
          return(memcmp(&curr, &last, sizeof(curr)) == 0);
      } else {
          bno.getRevInfo(&last);
          havelast = true;
          return(true);
      }
}

// 
// To get started all we need to do is attach out interrupt handler to the rising edge on PIN 2
// adjust the display brighness and make sure interrupts are disabled before we start sampling.
//
void setup() 
{   
     if (debug_g == true) Serial.begin(9600);
     oledBegin();                              // Bring up the OLED
     bnoState_g = false;                       // BNO is currently down
     digitalWrite(bnoRstPin, LOW);             // Allow BNO to come up.
     if (!debug_g) watchdog_setup();           // And start monitoring loop()   
}

//
// Sample the three access accelerometer and one access of the magnemometer for heading. If data seems correct 
// return true, otherwise false which will trigger a reset and X on the display.
//
inline bool sampleBNO(float *ox, float *oy, float *oz, float *os)
{
     imu::Vector<3> imudata;
     //
     imudata = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
     *ox = imudata.x();
     *oy = imudata.y();
     *oz = imudata.z();
     //
     // Slip/Skid is accelleration 
     imudata = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
     *os = imudata.x();
     //
     // Bounds check that angles are within range and G values are within
     // +/- 10G.
     //
#    define IF_NOT_IN_RANGE(v,a,b) if ((v < a)||(v > b))
     //
     IF_NOT_IN_RANGE(*ox          , -360,  +360) return(false);
     IF_NOT_IN_RANGE(*oy          , -360,  +360) return(false);
     IF_NOT_IN_RANGE(*oz          , -360,  +360) return(false);
     IF_NOT_IN_RANGE(*os          , -100,  +100) return(false);
     IF_NOT_IN_RANGE(imudata.y()  , -100,  +100) return(false);
     IF_NOT_IN_RANGE(imudata.z()  , -100,  +100) return(false);
     //
     return(true);
}

//
// Sample the sensors for ms ms. We return the actual time we used to sample.
// During the sample interval we either count interrupts, or using the analog 
// port we attempt to count transitions. If we get all zeros on every sample we 
// assume the sensor is dead, this will trigger an attempt to reset it.
//
inline bool sample(unsigned long ms, float *ox, float *oy, float *oz, float *os)
{        
     unsigned long t_end, t_now, t_start, t_dur, count;
     t_dur = ms * 1000;
     t_start = micros();
     t_end = t_start + t_dur; 
     t_now = t_start;
     count = 0;
     *ox = *oy = *oz = *os = 0;
     while(1) {
          count += 1;
          float ix,iy,iz,it,is; 
          if (!sampleBNO(&ix,&iy,&iz,&is)) return(false);
          *ox += ix;
          *oy += iy;
          *oz += iz;
          *os += is;
           t_now = micros();
           if ((t_start < t_end)  &&(t_now >= t_end))   break;
           if ((t_end   < t_start)&&(t_now >= t_start)) break;
     }
     *ox /= count;
     *oy /= count;
     *oz /= count;
     *os /= count;   
     return(true);
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
// are precomputed arguments. After rotation it appies and x,y shift to the point.
//
inline void rotateShiftPoint(int *x, int *y, int croll, int sroll, int xshift, int yshift)  
{      int x2 = (*x * croll - *y * sroll) / precis_g;       
       int y2 = (*x * sroll + *y * croll) / precis_g;       
       *x = x2 + xshift;
       *y = y2 + yshift;            
}

//
// This will draw a line subject to a rotation and shift. To save on trig funcion calls we take the
// sin and cos of the angle to rotate as arguments however the pitch is just a simple addition. This is
// not a true 3d rendering of the horizon as that would exceed the capabilities of the UNO to compute in
// real time but it does a good job emulating what you'd see with a 2d rotation and shift.
//
inline void drawRotateShiftedLine(int x0, int y0, int x1, int y1, int croll, int sroll, int xshift, int yshift)
{
       rotateShiftPoint(&x0,&y0,croll,sroll,xshift,yshift);        
       rotateShiftPoint(&x1,&y1,croll,sroll,xshift,yshift);
       drawLine(x0,y0,x1,y1);          
}

//
// The loop just performs 1/2 second of sampling after which is prints three G numbers. If for some reason
// the device is not active it keeps trying to start it.
//
void loop()     
{    
     // Reset the watchdog counter. Its a volatile byte incremented in the watchdog ISR.
     watchdog_reset();
     //
     // If we are down, try to go up but draw an X and initailize the display
     if (bnoState_g == false) {
         oledClearDisplay(BLACK);
         drawLine(-100, -100, 100,  100);           // Draw an X on screen
         drawLine(-100,  100, 100, -100); 
         watchdog_reset();                          // Avoid a WD timout on the delay
         oledUpdateDisplay();
         delay(1000);
         if (debug_g) Serial.println("Re Init BNO");
         bnoState_g = bno.begin();
         if (bnoState_g == true) {
             bnoState_g = bnoVerifyRevInfo();                     
             if (debug_g == true) {
                 sensor_t sensor;
                 bno.getSensor(&sensor);   
                 Serial.print  ("Sensor:"); Serial.println(sensor.name);
             }
             bno.setExtCrystalUse(true);
             watchdog_reset();
             delay(1000);                            // Avoid a WD timeout on the delay
         }
     } else {
         float ox,  oy, oz, os;
         char buf[32];
         int sampleMs =  20;                         // longer samples in debug mode.
         if (sample(sampleMs,&ox,&oy,&oz,&os)) {     // Sample Gyro/Accel
             oledClearDisplay(BLACK);                // Start with a clean white display
             //
             drawLine(-50, -10, -15,  -10);          // Draw the little plane. 
             drawLine(-15, -10,   0,   10);          // basically a --V--- symbol.
             drawLine( 0,   10,  15,  -10);      
             drawLine( 15, -10,  50,  -10); 
             //
             int   yawdeg= (int) (ox + 0.5);          // round up yaw in degrees
             int   skid  = (int) (os * 5);            // 9.8m/s is about 1/2 deflection
             char  buf[15];                           // will print here as 090 etc.
             float roll  = (-oy * M_PI) / 180.0;      // convert roll to radians
             float pitch = ( oz * M_PI) / 180.0;      // convert pitch to radians.
             float yaw   = ( ox * M_PI) / 180.0;      // convert yaw to radians
             float cosroll = cos(roll);
             float sinroll = sin(roll); 
             bool  isUp    = (oz < 90) && (oz > -90); // upright ?
             int   seconds = (millis()/1000);         // Every 2 seconds this is true
      static int   pseconds = 0;                      // previous seconds (static)
      static bool  xOutHeading = false;               // X out heading if compas bad
      static bool  xOutDisplay = false;               // X out display if gyro bad
      static char  flash = ' ';                       // alternates '-' ' '
             //
             if (debug_g) {
                 Serial.println("------"); Serial.println(oy); Serial.println(oz);
             }
             yawdeg = (yawdeg + 180) % 360;                // correct for chip placement.
             //
             if (seconds - pseconds >= 1) {                // Test the sensor every 2 seconds.
                  pseconds = seconds;                      // Remember last time we checked
                  uint8_t sys, gyro, accel, magn;
                  flash = flash == ' ' ? '-' : ' ';
                  bno.getCalibration(&sys,&gyro,&accel,&magn);
                  xOutHeading = (magn != 3);
                  xOutDisplay = (gyro != 3);
                  if (!bnoVerifyRevInfo()) {
                      bnoState_g = false;
                  }
             }
             //
             sprintf(buf,"%c%c %03d %c%c", flash, flash, yawdeg, flash, flash );// Output yaw as 090 to top
             oledSetStr(buf, OLED_WIDTH/2-23, 4, WHITE);
             if (xOutHeading) {
                 drawLine(-30, -100 , 30, -70);       // X out the heading if the compass not ready
                 drawLine(-30, -70,   30, -100); 
             }
             if (xOutDisplay) {
                 drawLine(-100, -100, 100,  100);     // Draw an X on screen if gyro not ready
                 drawLine(-100,  100, 100, -100); 
             }
             //
             //  Draw the horizon and ground lines leading toward it rotated and shifted according to pitch and 
             //  roll. We treat the inverted situation differently to simplifly the math and avoid 3d projectsion
             //  etc.
             //
             if (isUp) {                      
                 // Upright ground image 
                 int croll = cosroll * precis_g;                                          // precompute some trig.
                 int sroll = sinroll * precis_g;  
                 //
                 int ypitch =   (pitch * croll)/(M_PI/4.0);
                 int xpitch =  -(pitch * sroll)/(M_PI/2.0);
                 //        
                 drawRotateShiftedLine(-250, 0,   250, 0, croll, sroll, xpitch, ypitch);  // Draw Horizon                
                 drawRotateShiftedLine(-60,  250,-25,  0, croll, sroll, xpitch, ypitch);  // Draw lines orthogonal(ish)
                 drawRotateShiftedLine( 60,  250, 25,  0, croll, sroll, xpitch, ypitch);  // to horizon                        
              } else {
                 // Inverted ground image.
                 int croll =   cosroll * precis_g;         // display rolls is backwards when inverted.
                 int sroll = - sinroll * precis_g;
                 if (pitch > 0) 
                     pitch = -M_PI + pitch;
                 else
                     pitch =  M_PI + pitch; 
                 //
                 int ypitch =  (pitch * croll)/(M_PI/4.0);
                 int xpitch = -(pitch * sroll)/(M_PI/2.0);
                 //                                        
                 drawRotateShiftedLine(-250,   0, 250, 0, croll, sroll, xpitch, ypitch);   // Draw Horizon
                 drawRotateShiftedLine(-60, -250, -25, 0, croll, sroll, xpitch, ypitch);   // Draw lines orthogonal(ish)
                 drawRotateShiftedLine( 60, -250,  25, 0, croll, sroll, xpitch, ypitch);   // to horizo (but inverted view) 
              }
             //
             // Draw the skid diamond character (which we've overlaoded in the bit map for the ~ character)  
             //
             oledSetChar('(', OLED_WIDTH/2 - 8, OLED_HEIGHT - 20 , WHITE);
             oledSetChar(')', OLED_WIDTH/2 + 4, OLED_HEIGHT - 20 , WHITE);
             oledSetChar(OLED_TRIANGLE, skid + OLED_WIDTH/2 - 3, OLED_HEIGHT - 20 , WHITE); 
             //  
             oledUpdateDisplay();
         } else {                            // Want to modify so that sampling detects failure and draws X
             bnoState_g = false;             // then tries to re-initialize. 
         }
     }
}
