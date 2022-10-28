//--------------------------------------------------------------------------
// 2018 Modified by Laurent Moll for Uncanny Eyes costume
// https://www.hackster.io/projects/376a13/
// Based on Adafruit code  - Adafruit header below:
//
// Uncanny eyes for PJRC Teensy 3.1 with Adafruit 1.5" OLED (product #1431)
// or 1.44" TFT LCD (#2088).  This uses Teensy-3.1-specific features and
// WILL NOT work on normal Arduino or other boards!  Use 72 MHz (Optimized)
// board speed -- OLED does not work at 96 MHz.
//
// Adafruit invests time and resources providing this open source code,
// please support Adafruit and open-source hardware by purchasing products
// from Adafruit!
//
// Written by Phil Burgess / Paint Your Dragon for Adafruit Industries.
// MIT license.  SPI FIFO insight from Paul Stoffregen's ILI9341_t3 library.
// Inspired by David Boccabella's (Marcwolf) hybrid servo/OLED eye concept.
//--------------------------------------------------------------------------

#include <SPI.h>
#include <Adafruit_GFX.h>      // Core graphics lib for Adafruit displays
#include <HardwareSerial.h>    // Needed for 2nd serial port on ESP32

// Slightly modified headers in the eye include files to be able to include 2 and switch between them
// The ESP32 has a lot of memory, so this works fine
#include "defaultEye.h"        // Standard human-ish hazel eye
#include "newtEye.h"           // Eye of newt

const uint16_t (*sclera)[SCLERA_WIDTH] = scleraDefault;
const uint8_t (*upper)[SCREEN_WIDTH] = upperDefault;
const uint8_t (*lower)[SCREEN_WIDTH] = lowerDefault;
const uint16_t (*polar)[80] = polarDefault;
const uint16_t (*iris)[IRIS_MAP_WIDTH] = irisDefault;

// DISPLAY HARDWARE CONFIG -------------------------------------------------

#include <Adafruit_SSD1351.h>  // OLED display library -OR-

typedef Adafruit_SSD1351 displayType; // Using OLED display(s)

#define DISPLAY_DC      16 // Data/command pin for BOTH displays
#define DISPLAY_RESET   5 // Reset pin for BOTH displays
#define SELECT_L_PIN    17 // LEFT eye chip select pin
#define SELECT_R_PIN    04 // RIGHT eye chip select pin
#define UART_RX_PIN     13 // Pin to receive UART commands from controller

// INPUT CONFIG (for eye motion -- enable or comment out as needed) --------

#define TRACKING          // If enabled, eyelid tracks pupil
#define IRIS_SMOOTH       // If enabled, filter input from IRIS_PIN
#define IRIS_MIN      150 // Clip lower analogRead() range from IRIS_PIN (WAS: 120) - Reduced range so that it doesn't look to odd with multiple eye pairs
#define IRIS_MAX      400 // Clip upper "                                (WAS: 720) - Reduced range so that it doesn't look to odd with multiple eye pairs
#define AUTOBLINK        // If enabled, eyes blink autonomously

// Probably don't need to edit any config below this line, -----------------
// unless building a single-eye project (pendant, etc.), in which case one
// of the two elements in the eye[] array further down can be commented out.

// Eye blinks are a tiny 3-state machine.  Per-eye allows winks + blinks.
#define NOBLINK 0     // Not currently engaged in a blink
#define ENBLINK 1     // Eyelid is currently closing
#define DEBLINK 2     // Eyelid is currently opening
typedef struct {
  uint8_t  state;     // NOBLINK/ENBLINK/DEBLINK
  uint32_t duration;  // Duration of blink state (micros)
  uint32_t startTime; // Time (micros) of last state change
} eyeBlink;

#define MOSI_PIN  23
#define SCLK_PIN  18

SPISettings settings(16000000, MSBFIRST, SPI_MODE3); // 26.667MHz seems reliable on the ESP32.
struct {
  displayType display; // OLED/TFT object
  uint8_t     cs;      // Chip select pin
  eyeBlink    blink;   // Current blink state
} eye[] = { // OK to comment out one of these for single-eye display:
  // displayType(SELECT_L_PIN,DISPLAY_DC,0),SELECT_L_PIN,{NOBLINK},
  Adafruit_SSD1351(128, 128, &SPI, SELECT_L_PIN, DISPLAY_DC, DISPLAY_RESET), SELECT_L_PIN,{NOBLINK},

  // displayType(SELECT_R_PIN,DISPLAY_DC,0),SELECT_R_PIN,{NOBLINK},
};
#define NUM_EYES (sizeof(eye) / sizeof(eye[0]))

// INITIALIZATION -- runs once at startup ----------------------------------

HardwareSerial SerialIn(1);

void setup(void) {
  uint8_t e;

  Serial.begin(921600);
  // SerialIn.begin(9600, SERIAL_8N1, UART_RX_PIN); // disabled
  randomSeed(analogRead(A3)); // Seed random() from floating analog input

  // Both displays share a common reset line; 0 is passed to display
  // constructor (so no reset in begin()) -- must reset manually here:
  pinMode(DISPLAY_RESET, OUTPUT);
  digitalWrite(DISPLAY_RESET, LOW);  delay(1);
  digitalWrite(DISPLAY_RESET, HIGH); delay(50);

  for(e=0; e<NUM_EYES; e++) { // Deselect all
    pinMode(eye[e].cs, OUTPUT);
    digitalWrite(eye[e].cs, HIGH);
  }
  for(e=0; e<NUM_EYES; e++) {
    digitalWrite(eye[e].cs, LOW); // Select one eye for init
    eye[e].display.begin();
    digitalWrite(eye[e].cs, HIGH); // Deselect
  }
  
  // One of the displays is configured to mirror on the X axis.  Simplifies
  // eyelid handling in the drawEye() function -- no need for distinct
  // L-to-R or R-to-L inner loops.  Just the X coordinate of the iris is
  // then reversed when drawing this eye, so they move the same.  Magic!
  // eye[0].display.writeCommand(SSD1351_CMD_SETREMAP);
  // eye[0].display.write16(0x76);
}


// EYE-RENDERING FUNCTION --------------------------------------------------9
void drawEye( // Renders one eye.  Inputs must be pre-clipped & valid.
  uint8_t  e,       // Eye array index; 0 or 1 for left/right
  uint32_t iScale,  // Scale factor for iris
  uint8_t  scleraX, // First pixel X offset into sclera image
  uint8_t  scleraY, // First pixel Y offset into sclera image
  uint8_t  uT,      // Upper eyelid threshold value
  uint8_t  lT) {    // Lower eyelid threshold value

  uint8_t  screenX, screenY, scleraXsave;
  int16_t  irisX, irisY;
  uint16_t p, a, burstIdx;
  uint32_t d;
  static uint16_t pBurst[SCREEN_WIDTH*SCREEN_HEIGHT]; // Full frame buffer possible on ESP32
    
  // Set up raw pixel dump to entire screen.  Although such writes can wrap
  // around automatically from end of rect back to beginning, the region is
  // reset on each frame here in case of an SPI glitch.

  scleraXsave = scleraX; // Save initial X value to reset on each line
  irisY       = scleraY - (SCLERA_HEIGHT - IRIS_HEIGHT) / 2;
  burstIdx = 0;
  for(screenY=0; screenY<SCREEN_HEIGHT; screenY++, scleraY++, irisY++) {
    scleraX = scleraXsave;
    irisX   = scleraXsave - (SCLERA_WIDTH - IRIS_WIDTH) / 2;
    for(screenX=0; screenX<SCREEN_WIDTH; screenX++, scleraX++, irisX++) {
      if((lower[screenY][screenX] <= lT) ||
         (upper[screenY][screenX] <= uT)) {             // Covered by eyelid
        p = 0;
      } else if((irisY < 0) || (irisY >= IRIS_HEIGHT) ||
                (irisX < 0) || (irisX >= IRIS_WIDTH)) { // In sclera
        p = sclera[scleraY][scleraX];
      } else {                                          // Maybe iris...
        p = polar[irisY][irisX];                        // Polar angle/dist
        d = (iScale * (p & 0x7F)) / 128;                // Distance (Y)
        if(d < IRIS_MAP_HEIGHT) {                       // Within iris area
          a = (IRIS_MAP_WIDTH * (p >> 7)) / 512;        // Angle (X)
          p = iris[d][a];                               // Pixel = iris
        } else {                                        // Not in iris
          p = sclera[scleraY][scleraX];                 // Pixel = sclera
        }
      }
      pBurst[burstIdx++] = p;
    }
  }
  
  eye[e].display.startWrite();
  eye[e].display.writeCommand(SSD1351_CMD_SETROW);    // Y range
  eye[e].display.write16(0x0);
  eye[e].display.write16(SCREEN_HEIGHT - 1);
  eye[e].display.writeCommand(SSD1351_CMD_SETCOLUMN); // X range
  eye[e].display.write16(0x0);
  eye[e].display.write16(SCREEN_WIDTH  - 1);
  eye[e].display.writeCommand(SSD1351_CMD_WRITERAM);  // Begin write

  // For ESP32, use writePixels function to transfer the whole framebuffer in one large burst
  SPI.writePixels((uint8_t*)pBurst, sizeof(pBurst));
  eye[e].display.endWrite();
}


// EYE ANIMATION -----------------------------------------------------------

const uint8_t ease[] = { // Ease in/out curve for eye movements 3*t^2-2*t^3
    0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,  3,   // T
    3,  3,  4,  4,  4,  5,  5,  6,  6,  7,  7,  8,  9,  9, 10, 10,   // h
   11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 22, 23,   // x
   24, 25, 26, 27, 27, 28, 29, 30, 31, 33, 34, 35, 36, 37, 38, 39,   // 2
   40, 41, 42, 44, 45, 46, 47, 48, 50, 51, 52, 53, 54, 56, 57, 58,   // A
   60, 61, 62, 63, 65, 66, 67, 69, 70, 72, 73, 74, 76, 77, 78, 80,   // l
   81, 83, 84, 85, 87, 88, 90, 91, 93, 94, 96, 97, 98,100,101,103,   // e
  104,106,107,109,110,112,113,115,116,118,119,121,122,124,125,127,   // c
  128,130,131,133,134,136,137,139,140,142,143,145,146,148,149,151,   // J
  152,154,155,157,158,159,161,162,164,165,167,168,170,171,172,174,   // a
  175,177,178,179,181,182,183,185,186,188,189,190,192,193,194,195,   // c
  197,198,199,201,202,203,204,205,207,208,209,210,211,213,214,215,   // o
  216,217,218,219,220,221,222,224,225,226,227,228,228,229,230,231,   // b
  232,233,234,235,236,237,237,238,239,240,240,241,242,243,243,244,   // s
  245,245,246,246,247,248,248,249,249,250,250,251,251,251,252,252,   // o
  252,253,253,253,254,254,254,254,254,255,255,255,255,255,255,255 }; // n

#ifdef AUTOBLINK
uint32_t timeOfLastBlink = 0L, timeToNextBlink = 0L;
#endif

#define BCAST_ADDR 0
#define SER_CMD_SIZE 9
uint16_t serAddr = 1;

void frame( // Process motion for a single frame of left or right eye
  uint16_t        iScale) {     // Iris scale (0-1023) passed in
  static uint32_t frames   = 0; // Used in frame rate calculation
  static uint8_t  eyeIndex = 0; // eye[] array counter
  int16_t         eyeX, eyeY;
  uint32_t        t; // Time at start of function
  static char     serCmd[SER_CMD_SIZE+1];
  static uint16_t serCmdIdx = 0;
  static uint16_t serNewEyeCtrl = 0;
  static uint16_t serEyeCtrl = 0;

  // Process serial input
  // Command format: "#abxxxyyy_"
  // #: Address (0: bcast, other 1-6)
  // a: Eye position control
  //    O: Auto mode
  //    I: Controlled by xxxyyy  
  // b: Eye type
  //    O: Default
  //    I: Newt
  // xxx: x position in zero-extended decimal   000-255
  // yyy: y position in zero-extended decimal 000-255
  // _: space character

  /*
   while (SerialIn.available()) {      // If anything comes in Serial,
    serCmd[serCmdIdx] = SerialIn.read();
    if (serCmd[serCmdIdx] == ' ') {
      serCmd[serCmdIdx] = '\0';
      //Serial.println(serCmd);
      if (serCmdIdx != SER_CMD_SIZE) {
        Serial.println("Error: Serial command too short.");
      } else {
        if (serCmd[0] == '0' || serCmd[0] == '0' + serAddr) {
          if (serCmd[1] == 'I') {
            if (!isdigit(serCmd[3]) || !isdigit(serCmd[4]) || !isdigit(serCmd[5]) || !isdigit(serCmd[6]) || !isdigit(serCmd[7]) || !isdigit(serCmd[8])) {
              Serial.println("Error: Serial position not a number");
            } else { // If button is pressed, stop random eye movements and move (gracefully) to desired position
              serNewEyeCtrl = serEyeCtrl = 1;
              eyeX = 1020 - ((serCmd[3]-'0')*100 + (serCmd[4]-'0')*10 + (serCmd[5]-'0')) * 4;
              eyeY = ((serCmd[6]-'0')*100 + (serCmd[7]-'0')*10 + (serCmd[8]-'0')) * 4;
            }
          } else if (serCmd[1] == 'O') {
            serNewEyeCtrl = serEyeCtrl = 0;
          } else {
            Serial.println("Error: Serial command[1] unknown.");
          }
          if (serCmd[2] == 'O') { // Switch eye type right away based on button
            sclera = scleraDefault;
            upper = upperDefault;
            lower = lowerDefault;
            polar = polarDefault;
            iris = irisDefault;
          } else if (serCmd[2] == 'I') {
            sclera = scleraNewt;
            upper = upperNewt;
            lower = lowerNewt;
            polar = polarNewt;
            iris = irisNewt;
          } else {
            Serial.println("Error: Serial command[2] unknown.");
          }
        }
      }
      serCmdIdx = 0;
    } else {
      serCmdIdx++;
      if (serCmdIdx > SER_CMD_SIZE) {
        Serial.println("Error: Serial command too long.");
        serCmdIdx = 0;
      }
    }
  }
  */

  if(++eyeIndex >= NUM_EYES) eyeIndex = 0; // Cycle through eyes, 1 per call

  // X/Y movement

  t = micros();

  // Autonomous X/Y eye motion
  // Periodically initiates motion to a new random point, random speed,
  // holds there for random period until next motion.

  static boolean  eyeInMotion      = false;
  static int16_t  eyeOldX=512, eyeOldY=512, eyeCurX=512, eyeCurY=512, eyeNewX=512, eyeNewY=512;
  static uint32_t eyeMoveStartTime = 0L;
  static int32_t  eyeMoveDuration  = 0L;

  // Added to original code
  // If the controller sends a desired position, get out of random mode and move naturally to desired position
  if (serNewEyeCtrl) {
    eyeNewX = eyeX;
    eyeNewY = eyeY;
    eyeOldX = eyeCurX;                  // start from where we currently are
    eyeOldY = eyeCurY;
    eyeMoveDuration  = 100000;          // 100ms sec
    eyeMoveStartTime = t;               // Save initial time of move
    eyeInMotion      = true;            // Start move on next frame
    serNewEyeCtrl = 0;
  }
  
  int32_t dt = t - eyeMoveStartTime;      // uS elapsed since last eye event

  if(eyeInMotion) {                       // Currently moving?
    if(dt >= eyeMoveDuration) {           // Time up?  Destination reached.
      if (serEyeCtrl) { // If serial controlled, we're done moving, but stay in motion
        eyeX = eyeOldX = eyeNewX;           // Save position
        eyeY = eyeOldY = eyeNewY;
      } else {
        eyeInMotion      = false;           // Stop moving
        eyeMoveDuration  = random(3000000); // 0-3 sec stop
        eyeMoveStartTime = t;               // Save initial time of stop
        eyeX = eyeOldX = eyeNewX;           // Save position
        eyeY = eyeOldY = eyeNewY;
      }
    } else { // Move time's not yet fully elapsed -- interpolate position
      int16_t e = ease[255 * dt / eyeMoveDuration] + 1;   // Ease curve
      eyeX = eyeOldX + (((eyeNewX - eyeOldX) * e) / 256); // Interp X
      eyeY = eyeOldY + (((eyeNewY - eyeOldY) * e) / 256); // and Y
    }
  } else {                                // Eye stopped
    eyeX = eyeOldX;
    eyeY = eyeOldY;
    if(dt > eyeMoveDuration) {            // Time up?  Begin new move.
      int16_t  dx, dy;
      uint32_t d;
      do {                                // Pick new dest in circle
        eyeNewX = random(1024);
        eyeNewY = random(1024);
        dx      = (eyeNewX * 2) - 1023;
        dy      = (eyeNewY * 2) - 1023;
      } while((d = (dx * dx + dy * dy)) > (1023 * 1023)); // Keep trying
      eyeMoveDuration  = random(72000, 144000); // ~1/14 - ~1/7 sec
      eyeMoveStartTime = t;               // Save initial time of move
      eyeInMotion      = true;            // Start move on next frame
    }
  }
  eyeCurX = eyeX;
  eyeCurY = eyeY;

  
  // Blinking

#ifdef AUTOBLINK
  // Similar to the autonomous eye movement above -- blink start times
  // and durations are random (within ranges).
  if((t - timeOfLastBlink) >= timeToNextBlink) { // Start new blink?
    timeOfLastBlink = t;
    uint32_t blinkDuration = random(36000, 72000); // ~1/28 - ~1/14 sec
    // Set up durations for both eyes (if not already winking)
    for(uint8_t e=0; e<NUM_EYES; e++) {
      if(eye[e].blink.state == NOBLINK) {
        eye[e].blink.state     = ENBLINK;
        eye[e].blink.startTime = t;
        eye[e].blink.duration  = blinkDuration;
      }
    }
    timeToNextBlink = blinkDuration * 3 + random(4000000);
  }
#endif

  if(eye[eyeIndex].blink.state) { // Eye currently blinking?
    // Check if current blink state time has elapsed
    if((t - eye[eyeIndex].blink.startTime) >= eye[eyeIndex].blink.duration) {
      // No buttons, or other state...
        if(++eye[eyeIndex].blink.state > DEBLINK) { // Deblinking finished?
          eye[eyeIndex].blink.state = NOBLINK;      // No longer blinking
        } else { // Advancing from ENBLINK to DEBLINK mode
          eye[eyeIndex].blink.duration *= 2; // DEBLINK is 1/2 ENBLINK speed
          eye[eyeIndex].blink.startTime = t;
        }
     
    }
  }

  // Process motion, blinking and iris scale into renderable values

  // Iris scaling: remap from 0-1023 input to iris map height pixel units
  iScale = ((IRIS_MAP_HEIGHT + 1) * 1024) /
           (1024 - (iScale * (IRIS_MAP_HEIGHT - 1) / IRIS_MAP_HEIGHT));

  // Scale eye X/Y positions (0-1023) to pixel units used by drawEye()
  eyeX = map(eyeX, 0, 1023, 0, SCLERA_WIDTH  - 128);
  eyeY = map(eyeY, 0, 1023, 0, SCLERA_HEIGHT - 128);
  if(eyeIndex == 1) eyeX = (SCLERA_WIDTH - 128) - eyeX; // Mirrored display

  // Horizontal position is offset so that eyes are very slightly crossed
  // to appear fixated (converged) at a conversational distance.  Number
  // here was extracted from my posterior and not mathematically based.
  // I suppose one could get all clever with a range sensor, but for now...
  eyeX += 4;
  if(eyeX > (SCLERA_WIDTH - 128)) eyeX = (SCLERA_WIDTH - 128);

  // Eyelids are rendered using a brightness threshold image.  This same
  // map can be used to simplify another problem: making the upper eyelid
  // track the pupil (eyes tend to open only as much as needed -- e.g. look
  // down and the upper eyelid drops).  Just sample a point in the upper
  // lid map slightly above the pupil to determine the rendering threshold.
  static uint8_t uThreshold = 128;
  uint8_t        lThreshold, n;
#ifdef TRACKING
  int16_t sampleX = SCLERA_WIDTH  / 2 - (eyeX / 2), // Reduce X influence
          sampleY = SCLERA_HEIGHT / 2 - (eyeY + IRIS_HEIGHT / 4);
  // Eyelid is slightly asymmetrical, so two readings are taken, averaged
  if(sampleY < 0) n = 0;
  else            n = (upper[sampleY][sampleX] +
                       upper[sampleY][SCREEN_WIDTH - 1 - sampleX]) / 2;
  uThreshold = (uThreshold * 3 + n) / 4; // Filter/soften motion
  // Lower eyelid doesn't track the same way, but seems to be pulled upward
  // by tension from the upper lid.
  lThreshold = 254 - uThreshold;
#else // No tracking -- eyelids full open unless blink modifies them
  uThreshold = lThreshold = 0;
#endif

  // The upper/lower thresholds are then scaled relative to the current
  // blink position so that blinks work together with pupil tracking.
  if(eye[eyeIndex].blink.state) { // Eye currently blinking?
    uint32_t s = (t - eye[eyeIndex].blink.startTime);
    if(s >= eye[eyeIndex].blink.duration) s = 255;   // At or past blink end
    else s = 255 * s / eye[eyeIndex].blink.duration; // Mid-blink
    s          = (eye[eyeIndex].blink.state == DEBLINK) ? 1 + s : 256 - s;
    n          = (uThreshold * s + 254 * (257 - s)) / 256;
    lThreshold = (lThreshold * s + 254 * (257 - s)) / 256;
  } else {
    n          = uThreshold;
  }

  // Pass all the derived values to the eye-rendering function:
  drawEye(eyeIndex, iScale, eyeX, eyeY, n, lThreshold);
}


// AUTONOMOUS IRIS SCALING (if no photocell or dial) -----------------------

// Autonomous iris motion uses a fractal behavior to similate both the major
// reaction of the eye plus the continuous smaller adjustments that occur.

uint16_t oldIris = (IRIS_MIN + IRIS_MAX) / 2, newIris;

void split( // Subdivides motion path into two sub-paths w/randomization
  int16_t  startValue, // Iris scale value (IRIS_MIN to IRIS_MAX) at start
  int16_t  endValue,   // Iris scale value at end
  uint32_t startTime,  // micros() at start
  int32_t  duration,   // Start-to-end time, in microseconds
  int16_t  range) {    // Allowable scale value variance when subdividing

  if(range >= 8) {     // Limit subdvision count, because recursion
    range    /= 2;     // Split range & time in half for subdivision,
    duration /= 2;     // then pick random center point within range:
    int16_t  midValue = (startValue + endValue - range) / 2 + random(range);
    uint32_t midTime  = startTime + duration;
    split(startValue, midValue, startTime, duration, range); // First half
    split(midValue  , endValue, midTime  , duration, range); // Second half
  } else {             // No more subdivisons, do iris motion...
    int32_t dt;        // Time (micros) since start of motion
    int16_t v;         // Interim value
    while((dt = (micros() - startTime)) < duration) {
      v = startValue + (((endValue - startValue) * dt) / duration);
      if(v < IRIS_MIN)      v = IRIS_MIN; // Clip just in case
      else if(v > IRIS_MAX) v = IRIS_MAX;
      frame(v);        // Draw frame w/interim iris scale value
    }
  }
}

// MAIN LOOP -- runs continuously after setup() ----------------------------

void loop() {

// Autonomous iris scaling -- invoke recursive function

  newIris = random(IRIS_MIN, IRIS_MAX);
  
  split(oldIris, newIris, micros(), 10000000L, IRIS_MAX - IRIS_MIN);
  oldIris = newIris;
}