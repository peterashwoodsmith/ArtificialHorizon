# ArtificialHorizon

Using an Arduino UNO, a Waveshare 128x128 1.5" OLED and an Adafruit BN055 gyroscope we create a simple artificial horizon 
and directional indicator that works upright and inverted. Not for use in flight... for educational purposes only.

The code draws a horizon consisting of a horizontal line and two orthogonal lines to give perspective. It then draws a little
fixed aircraft diagram similar to what is actually used in many real devices consisting of 4 lines like this: ---V---. The horizon lines
pitch and roll according to input from the gyroscope which is averaged for 20ms or so. To make life simpler I use a normalized -1--+1 display
area and draw lines in that co-ordinate system and rotate them there too. Then they are mapped onto a 128x64 bit map which is then uploaded to
the 128x128x4 bit map in the waveshare by doubling the rows and mapping every single pixel to either full white or off. This is necessary because
the Arduino UNO does not have sufficient memory to store the entire image. On a higher performance processor you could do all kinds of cool stuff
like draw brown vs blue skye etc. 

There is a brief youtube description of this project here: https://youtu.be/mU1mT4dsyRU

