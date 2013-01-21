

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
#include <SD.h>

// TFT display and SD card will share the hardware SPI interface.
// Hardware SPI pins are specific to the Arduino board type and
// cannot be remapped to alternate pins.  For Arduino Uno,
// Duemilanove, etc., pin 11 = MOSI, pin 12 = MISO, pin 13 = SCK.
#define SD_CS    4  // Chip select line for SD card
#define TFT_CS  10  // Chip select line for TFT display
#define TFT_DC   9  // Data/command line for TFT
#define TFT_RST  8  // Reset line for TFT (or connect to +5V)


#include <EEPROM.h>
#include "Wire.h"

#define DIR_PIN 0
#define SPEED_PIN 1
#define MODE_PIN 12

int motorSpeed;
int motorDirection;
int stopCount = 0;
#define DIR_THRESHOLD 100
#define SPEED_THRESHOLD 5
#define STOP_THRESHOLD 35
#define REC_STOP_BYTE 0xff

int addr = 0;
#define MAX_ADDR 512
unsigned long startTime;

int recMode = true;


Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void setup(void) {
  
  Serial.begin(9600);
 
  pinMode(MODE_PIN, INPUT);
  Serial.begin(19200);
 
  recMode = digitalRead(MODE_PIN);   
  if (recMode) {
    recordStopByte();  
  }
   pinMode(6, OUTPUT);
  // Our supplier changed the 1.8" display slightly after Jan 10, 2012
  // so that the alignment of the TFT had to be shifted by a few pixels
  // this just means the init code is slightly different. Check the
  // color of the tab to see which init code to try. If the display is
  // cut off or has extra 'random' pixels on the top & left, try the
  // other option!
  



  // If your TFT's plastic wrap has a Red Tab, use the following:
  tft.initR(INITR_REDTAB);   // initialize a ST7735R chip, red tab
  // If your TFT's plastic wrap has a Green Tab, use the following:
  //tft.initR(INITR_GREENTAB); // initialize a ST7735R chip, green tab

  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("failed!");
    return;
  }
  Serial.println("OK!");


  
bmpDraw("parrot.bmp", 0, 0);


}

void loop() {
  
  startTime = millis();

  if (recMode) { // RECORD MODE
    // check to see if mode may have changed
    if (digitalRead(MODE_PIN) != recMode) {
      // only need to write stop byte if not at start of address
      if (addr != 0)
        recordStopByte();
      recMode = false;
      return;
    }
    else {
      recordMode();
    }
  }
  else { // PLAYBACK MODE
    // check to see if mode may have changed
    if (digitalRead(MODE_PIN) != recMode) {
      addr = 0;
     
      recMode = true;
      return;
    }
    else { 
      playbackMode();
    }
  }

  // enforce 50 Hz sampling
  unsigned long nowTime = millis();
 // while ((nowTime - startTime) < 20)
  while ((nowTime - startTime) < 100)
    nowTime = millis();
  for (int i =0; i <255; i+=5){
    int brightness = i;
    Serial.println(i);
   analogWrite(6, brightness); 
   delay(200);   //the delay will be a factor of speed? will the max birghtness also be affected
  }
  
  delay(1000);
 
  for (int i =255; i >0 ; i-=5){
    int brightness = i;
    Serial.println(i);
   analogWrite(6, brightness); 
   delay(200);
  }

}

// This function opens a Windows Bitmap (BMP) file and
// displays it at the given coordinates.  It's sped up
// by reading many pixels worth of data at a time
// (rather than pixel by pixel).  Increasing the buffer
// size takes more of the Arduino's precious RAM but
// makes loading a little faster.  20 pixels seems a
// good balance.

#define BUFFPIXEL 20

void bmpDraw(char *filename, uint8_t x, uint8_t y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  if((x >= tft.width()) || (y >= tft.height())) return;

  Serial.println();
  Serial.print("Loading image '");
  Serial.print(filename);
  Serial.println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.print("File not found");
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.print("File size: "); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print("Image Offset: "); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print("Header size: "); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print("Bit Depth: "); Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print("Image size: ");
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x+w-1, y+h-1);

        for (row=0; row<h; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col=0; col<w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            tft.pushColor(tft.Color565(r,g,b));
          } // end pixel
        } // end scanline
        Serial.print("Loaded in ");
        Serial.print(millis() - startTime);
        Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) Serial.println("BMP format not recognized.");
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

void recordMode() {

  //Serial.print("RECORD MODE: \t"); 

  // motor speed
  motorSpeed = analogRead(SPEED_PIN);

  // if no movment and at start of address space then ignore
  if ((motorSpeed < SPEED_THRESHOLD) && (addr == 0)) {
    //Serial.println("STOPPED / WAITING"); 
    return;
  }


  // if no speed then motor stopped so record STOP BYTE
  if (motorSpeed < SPEED_THRESHOLD) {
    if (stopCount > STOP_THRESHOLD) {
      stopCount = 0;

      if (addr != 0) {
        recordStopByte();        
      }
      //Serial.println("STOPPED ------------------------------------"); 
      return;
    }
    else {
      stopCount++;
    }
  }
  else {
    // must have some movement so reset stop count
    stopCount = 0;
  }

  // read the motor direction pin:
  motorDirection = analogRead(DIR_PIN);

  // 0V = motor going in one direction
  // ~2V = motor going in other direction (use thresholding)
  if (motorDirection > DIR_THRESHOLD ) {
    motorDirection = 1;
  //  digitalWrite(ledPin, HIGH);   // set the LED on  
    //Serial.println("<---");
  }
  else {
    motorDirection = 0;
    //Serial.println("--->");
  //  digitalWrite(ledPin, LOW);   // set the LED off
  }

  saveEnergy(motorDirection, motorSpeed);

}

void saveEnergy(int dir, int speed) {


  if (addr > MAX_ADDR - 2) {
    // need to just record stop bytes and end (out of memory)
    recordStopByte();
    Serial.println("MEMORY FULL");
    return;
  }

  if (dir > 0 ) {
  //  analogWrite(LED_OUTPUT0_PIN, 0);
 //   analogWrite(LED_OUTPUT1_PIN, speed);
  
  }
  else {
  //  analogWrite(LED_OUTPUT0_PIN, speed);
  //  analogWrite(LED_OUTPUT1_PIN, 0);
   
  }

  EEPROM.write(addr, dir);
  addr++;
  EEPROM.write(addr, speed);
  addr++;
}




void recordStopByte() 
{


  EEPROM.write(addr, REC_STOP_BYTE);
  addr++;
  EEPROM.write(addr, REC_STOP_BYTE);

  // get ready to record again
  addr = 0;

 

}



// PLAYBACK MODE -- Playback recorded energy input speed and direction

void playbackMode()
{

  //Serial.print("PLAYBACK MODE: \t"); 
  //Serial.print(addr);
  //Serial.print(" \t"); 

  motorDirection = EEPROM.read(addr);
  addr++;
  motorSpeed = EEPROM.read(addr);
  addr++;

  if ((motorDirection == REC_STOP_BYTE) && (motorSpeed == REC_STOP_BYTE)) {
    // stop condition end of data
 //   analogWrite(LED_OUTPUT0_PIN, 0);
 //   analogWrite(LED_OUTPUT1_PIN, 0);
  
    delay(2000);
    addr = 0;
    //Serial.println("END"); 
    return;
  }

  if (motorDirection > 0 ) {
  //  analogWrite(LED_OUTPUT0_PIN, 0);
  //  analogWrite(LED_OUTPUT1_PIN, motorSpeed);
   
  }
  else {
  //  analogWrite(LED_OUTPUT0_PIN, motorSpeed);
  //  analogWrite(LED_OUTPUT1_PIN, 0);
  
  }


  //Serial.print(motorSpeed);
  //Serial.print("\t ");
  //Serial.print(motorDirection);
  //Serial.println();

  if (addr > MAX_ADDR - 2) {
    addr = 0;
  }

}
