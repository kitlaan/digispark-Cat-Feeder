//
//
// PIN-0 is connected to an active-low button
// (5V -- resistor -- GND -- button -- PIN0)
//
// PIN-1 is connected to an LED
// (GND -- (|<) -- PIN1)
//
// PIN-2 is connected to a servo
// (GND -- GND; 5V -- 5V; signal -- PIN2)
//
// digiSpark is driven via Vin, with a direction-limit diode, to 12V
//

#include <avr/pgmspace.h>

#define PIN_BTN 0  // manual trigger button
#define PIN_LED 1  // when-to-feed indicator
#define PIN_SRV 2  // servo

// human-linear scaled brightness, inverted (0-31)
const byte brightness[] PROGMEM = {
  255, 235, 216, 197, 180, 163, 148, 134,
  120, 108,  96,  86,  76,  67,  58,  51,
   44,  38,  32,  27,  22,  18,  15,  12,
    9,   7,   5,   4,   3,   2,   1,   0
};

// duration helpers
#define DURATION_30MIN  1800000
#define DURATION_1HR    3600000

// servo pulse control
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000
#define SERVO_REFRESH_MS 20

#define SERVO_DELTA_US (SERVO_MAX_US - SERVO_MIN_US)

// feed rates
#define ROT_NEUTRAL  90
#define ROT_FORWARD  (ROT_NEUTRAL + 20)
#define ROT_REVERSE  (ROT_NEUTRAL - 20)

// feed control
#define FEED_INTERVAL_MS  (DURATION_1HR * 4)
#define FEED_COUNT        2
#define FEED_FORWARD_MS   1800
#define FEED_REVERSE_MS   (FEED_FORWARD_MS / 3)

// debouncer
#define DEBOUNCE_WAIT_MS 5



boolean firstTime = true;
int lastButtonState = HIGH;

unsigned long lastFeedTime = 0;
unsigned long lastDebounceTime = 0;

void runServo(unsigned int angle, unsigned int duration)
{
  unsigned long timeOn = SERVO_MIN_US + ((unsigned long)SERVO_DELTA_US * angle) / 180;
  unsigned long timeOff = (SERVO_REFRESH_MS * 1000) - timeOn;
  unsigned int offMs = timeOff / 1000;
  unsigned int offUs = timeOff % 1000;
  
  for (unsigned int ct = 0; ct < duration; ct += SERVO_REFRESH_MS)
  {
    digitalWrite(PIN_SRV, HIGH);
    delayMicroseconds(timeOn);
    digitalWrite(PIN_SRV, LOW);
    delayMicroseconds(offUs);
    delay(offMs);
  }
}

void setup()
{
  pinMode(PIN_BTN, INPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SRV, OUTPUT);
  
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_SRV, LOW);
  
  lastFeedTime = millis();
}

void loop()
{
  boolean triggerFeed = false;
  unsigned long timeNow = millis();

  //
  // detect if button is pressed (with debounce), where we trigger
  //

  // read the current state for manual debounce
  int currentButtonState = digitalRead(PIN_BTN);
  if (currentButtonState != lastButtonState)
  {
    lastDebounceTime = timeNow;
    lastButtonState = currentButtonState;
  }
  // and figure out if it's been long enough
  if ((timeNow - lastDebounceTime) > DEBOUNCE_WAIT_MS)
  {
    // feed with a LOW signal
    if (currentButtonState == LOW)
    {
      triggerFeed = true;
    }
  }

  //
  // check if we met the feed interval
  //

  if ((timeNow - lastFeedTime) >= FEED_INTERVAL_MS)
  {
    lastFeedTime = timeNow;
    triggerFeed = true;
  }

  //
  // if we've just powered on, do a feeding
  //

  if (firstTime)
  {
    firstTime = false;
    // don't feed immediately, as the kitty plays with
    // the power connection...
    //triggerFeed = true;
  }

  //
  // dim the LED from bright (just fed) to off (feed soon)
  //

  // convert to a 5-bit value for the brightness table
  unsigned long scaling = FEED_INTERVAL_MS / 31;
  int ledValue = (timeNow - lastFeedTime) / scaling;
  // write the LED signal
  analogWrite(PIN_LED, pgm_read_byte(&brightness[ledValue])); 

  //
  // trigger a feeding
  //

  if (triggerFeed)
  {
    lastFeedTime = timeNow;

    digitalWrite(PIN_LED, HIGH);
    for (int count = 0; count < FEED_COUNT; count++)
    {
      runServo(ROT_FORWARD, FEED_FORWARD_MS);
      digitalWrite(PIN_LED, LOW);
      runServo(ROT_REVERSE, FEED_REVERSE_MS);
      digitalWrite(PIN_LED, HIGH);
      runServo(ROT_FORWARD, FEED_REVERSE_MS);
    }
    digitalWrite(PIN_LED, LOW);
  }
}

