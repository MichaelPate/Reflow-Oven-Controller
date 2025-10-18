// Include all libraries //////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <GyverMAX6675.h>


// Define all hardware ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LCD Hardware, Nokia 5110 48x84
#define LCD_CLK 7
#define LCD_DIN 6
#define LCD_DC 5
#define LCD_CS 4
#define LCD_RST 3
// Buttons for selecting profiles and start/stop
#define BTN1 A0
#define BTN2 A1
// Indicator for heat in the oven
#define HEAT_PILOT A2
// Heating elements (relays controlled by MOSFETs)
#define TOP_ELEMENT 9   // An LED for prototyping, an LED and relay coil in production
#define BOT_ELEMENT 10  // Same for this one
// Thermocouple SPI interface
#define TC_CS 11
#define TC_DAT 12
#define TC_CLK 8
// For indicating cycle complete
#define BUZZER 2


// Instantiate all objects ////////////////////////////////////////////////////////////////////////////////////////////////////////////
Adafruit_PCD8544 lcd = Adafruit_PCD8544(LCD_CLK,LCD_DIN,LCD_DC,LCD_CS,LCD_RST);
GyverMAX6675<TC_CLK, TC_DAT, TC_CS> tc;


// Other constants and globals ////////////////////////////////////////////////////////////////////////////////////////////////////////
// The pre states are for getting to the set temp. Cycle timer does not increment
enum CycleState {IDLE, PRE_PREHEAT, PREHEAT, PRE_SOAK, SOAK, PRE_REFLOW, REFLOW, COOLDOWN};


// Above this temp, cycle is considered incomplete
// Whenever above this, the high heat pilot is on
#define HIGH_HEAT_THRESHOLD_C 80


// process variables
double cv;   //cv
double pv;    //pv
double slope;  // maximum allowed rate of change of temp for a given part of the cycle
double output;  // this will be converted to a PWM duty cycle for the heaters
int stage_seconds;
int cycle_seconds;

// System states
CycleState systemState = IDLE;
CycleState lastState = IDLE;
bool atTemp, isBeeping;
int beepCount = 0;

// Timing
unsigned long lastStageMillis, lastCycleMillis, lastGetTempMillis, lastHoldButtonMillis, lastBeepMillis, lastBeepPauseMillis;
unsigned long lastAtTempMillis = 0;



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BuzzerChime()
{
  for(byte i = 0; i < 2; i++)
  {
    tone(BUZZER, 3900, 900);
    delay(900);
    tone(BUZZER, 3000, 900);
    if (i < 2) delay(2000);
  }
}


void DisplayUI(Adafruit_PCD8544 display, CycleState state, float ppv, float cv, int stageSeconds, int cycleSeconds)
{
  display.clearDisplay();
  // Display cycle stage
  display.setTextSize(2);
  display.setCursor(0,0);
  switch(state)
  {
    case IDLE: display.print("Idle"); break;
    case PRE_PREHEAT:
    case PREHEAT: display.print("Preheat"); break;
    case PRE_SOAK:
    case SOAK: display.print("Soak"); break;
    case PRE_REFLOW:
    case REFLOW: display.print("Reflow"); break;
    case COOLDOWN: display.print("Cooling"); break;
  }
  //display.print("Idle"); //Preheat, Soak, Reflow, Cool, Idle
  display.drawLine(0, 17, 84, 17, BLACK);
  
  // Stage time, cycle time, set temp, PCB temp
  // maybe some symbols for what heaters are activated
  // maybe curve name or number
  display.setTextSize(1);
  display.setCursor(0, 19+1);
  float pv = ppv;
  display.print(pv, 1);
  display.setCursor(33, 19+1);
  display.print("C");
  display.setCursor(39, 19+1);
  display.print("/");
  display.setCursor(46, 19+1);
  display.print(cv, 1);
  display.setCursor(78, 19+1);
  display.print("C");
  display.setCursor(0, 19+8+4);
  int stage_min = stageSeconds / 60;
  int stage_sec = stageSeconds % 60;
  
  display.print("Stage: " + (stage_min < 10 ? "0" + String(stage_min) : String(stage_min)) + ":" + (stage_sec < 10 ? "0" + String(stage_sec) : String(stage_sec)));
  display.setCursor(0, 19+8+8+5);
  int cycle_min = cycleSeconds / 60;
  int cycle_sec = cycleSeconds % 60;
  display.print("Cycle: " + (cycle_min < 10 ? "0" + String(cycle_min) : String(cycle_min)) + ":" + (cycle_sec < 10 ? "0" + String(cycle_sec) : String(cycle_sec)));
  display.display();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  lcd.begin();
  lcd.clearDisplay();
  delay(100);
  lcd.setContrast(60);
  
  // put your setup code here, to run once:
  pinMode(BUZZER, OUTPUT);
  pinMode(TOP_ELEMENT, OUTPUT);
  pinMode(BOT_ELEMENT, OUTPUT);
  pinMode(HEAT_PILOT, OUTPUT);
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);

  systemState = IDLE;

  Serial.begin(115200);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 * Loop structure:
 * 1. Read temperatures
 * 2. Determine what PWM to put to both heaters
 * 3. Check if state change required
 * 4. Handle counters
 * 
 * No delays allowed! Everything must be nonblocking!
 */

void loop() {

  // Get the process var and display the UI
  unsigned long currentMillis = millis();
  if (currentMillis - lastGetTempMillis > 500)
  {
    pv = tc.getTemp();
    lastGetTempMillis = currentMillis;
  }
  DisplayUI(lcd, systemState, pv, cv, stage_seconds, cycle_seconds);


  // Run the high heat indicator LED
  if (pv > HIGH_HEAT_THRESHOLD_C)
  {
    digitalWrite(HEAT_PILOT, HIGH);
  }
  else
  {
    digitalWrite(HEAT_PILOT, LOW);
  }


  // Update the stage and cycle clocks.
  currentMillis = millis();
  if (systemState != IDLE)
  {
    if (currentMillis - lastCycleMillis > 1000)
    {
      cycle_seconds++;
      lastCycleMillis = currentMillis;
    }

    if (systemState != PRE_PREHEAT && systemState != PRE_SOAK && systemState != PRE_REFLOW)
    {
      if (currentMillis - lastStageMillis > 1000)
      {
        stage_seconds++;
        lastStageMillis = currentMillis;
      }
      
    }
    
  }


  // If you hold BTN2 for 5 seconds it will jump to cooling
  currentMillis = millis();
  if (digitalRead(BTN2) == LOW)
  {
    if (lastHoldButtonMillis == 0) lastHoldButtonMillis = millis();
    if (millis() - lastHoldButtonMillis > 5000 && systemState != COOLDOWN && systemState != IDLE)
    {
      systemState = COOLDOWN;
      lastAtTempMillis = 0;
      stage_seconds = 0;
    }
  }
  else
  {
    lastHoldButtonMillis = 0;
  }


  // If we need to beep then beep
  currentMillis = millis();
  if (isBeeping)
  {
    // we are currently making tone so see if its time to stop
    if (currentMillis - lastBeepMillis > 500) // note duration of 500 ms
    {
      lastBeepMillis = currentMillis;
      noTone(BUZZER);
      isBeeping = false;
    }
  }
  else
  {
    if (beepCount > 0)
    {
      // we are not toning but we need to since beepcount is still > 0
      if (currentMillis - lastBeepMillis > 1000) // pause of 1000ms between notes
      {
        lastBeepMillis = currentMillis;
        tone(BUZZER, 6500);
        isBeeping = true;
        beepCount--;
      }      
    }
  }


  // Compute the PID to determine the output


  // Set the output as a PWM duty cycle to the relays

  
  // State machine
  switch(systemState)
  {
    case IDLE:
      // update setpoints and parameters
      cv = 0.0;
      slope = 0.0;
      stage_seconds = 0;

      // transition to next state, if start button is pushed
      if(digitalRead(BTN2) == LOW)
      {
        lastState = IDLE;
        systemState = PRE_PREHEAT;
        cycle_seconds = 0;
        stage_seconds = 0;
        lastAtTempMillis = 0;
        Serial.println("Leaving IDLE for PRE_PREHEAT");
      }
      
      break;
    case PRE_PREHEAT:
      // update setpoints and parameters
      cv = 150.0;
      slope = 3.0;

      // We need to make sure we are within 5 deg C of cv for at least 5 seconds, before changing states
      if (abs(cv - pv) < 5 || digitalRead(BTN1) == LOW)
      {
        if (lastAtTempMillis == 0) lastAtTempMillis = millis();
        // transition to next state, if temperature is reached for 5 seconds
        if (millis() - lastAtTempMillis > 5000 || digitalRead(BTN1) == LOW)
        {
          lastState = PRE_PREHEAT;
          systemState = PREHEAT;
          lastAtTempMillis = 0;
          stage_seconds = 0;
          Serial.println("Leaving PRE_PREHEAT for PREHEAT");
        }
      }
      else if (abs(cv - pv) > 5)
      {
        lastAtTempMillis = 0;
      }
      break;
    case PREHEAT:
      // update setpoints and parameters
      cv = 150.0;
      slope = 0.0;

      // We need to make sure we are within 5 deg C of cv for at least 60 seconds, before changing states
      if (abs(cv - pv) < 5 || digitalRead(BTN2) == LOW)
      {
        if (lastAtTempMillis == 0) lastAtTempMillis = millis();
        // transition to next state, if temperature is reached for 60 seconds
        if (millis() - lastAtTempMillis > 60000 || digitalRead(BTN2) == LOW)
        {
          lastState = PREHEAT;
          systemState = PRE_SOAK;
          lastAtTempMillis = 0;
          stage_seconds = 0;
          Serial.println("Leaving PREHEAT for PRE_SOAK");
        }
      }
      else if (abs(cv - pv) > 5)
      {
        lastAtTempMillis = 0;
      }
      break;
    case PRE_SOAK:
      // update setpoints and parameters
      cv = 165.0;
      slope = 3.0;

      // We need to make sure we are within 5 deg C of cv for at least 5 seconds, before changing states
      if (abs(cv - pv) < 5 || digitalRead(BTN1) == LOW)
      {
        if (lastAtTempMillis == 0) lastAtTempMillis = millis();
        // transition to next state, if temperature is reached for 5 seconds
        if (millis() - lastAtTempMillis > 5000 || digitalRead(BTN1) == LOW)
        {
          lastState = PRE_SOAK;
          systemState = SOAK;
          lastAtTempMillis = 0;
          stage_seconds = 0;
        }
      }
      else if (abs(cv - pv) > 5)
      {
        lastAtTempMillis = 0;
      }
      break;
    case SOAK:
      // update setpoints and parameters
      cv = 165.0;
      slope = 0.0;

      // We need to make sure we are within 5 deg C of cv for at least 60 seconds, before changing states
      if (abs(cv - pv) < 5 || digitalRead(BTN2) == LOW)
      {
        if (lastAtTempMillis == 0) lastAtTempMillis = millis();
        // transition to next state, if temperature is reached for 60 seconds
        if (millis() - lastAtTempMillis > 60000 || digitalRead(BTN2) == LOW)
        {
          lastState = SOAK;
          systemState = PRE_REFLOW;
          lastAtTempMillis = 0;
          stage_seconds = 0;
        }
      }
      else if (abs(cv - pv) > 5)
      {
        lastAtTempMillis = 0;
      }
      break;
    case PRE_REFLOW:
      // update setpoints and parameters
      cv = 235.0;
      slope = 3.0;

      // We need to make sure we are within 5 deg C of cv for at least 5 seconds, before changing states
      if (abs(cv - pv) < 5 || digitalRead(BTN1) == LOW)
      {
        if (lastAtTempMillis == 0) lastAtTempMillis = millis();
        // transition to next state, if temperature is reached for 5 seconds
        if (millis() - lastAtTempMillis > 5000 || digitalRead(BTN1) == LOW)
        {
          lastState = PRE_REFLOW;
          systemState = REFLOW;
          lastAtTempMillis = 0;
          stage_seconds = 0;
        }
      }
      else if (abs(cv - pv) > 5)
      {
        lastAtTempMillis = 0;
      }
      break;
    case REFLOW:
      // update setpoints and parameters
      cv = 235.0;
      slope = 0.0;

      // We need to make sure we are within 5 deg C of cv for at least 75 seconds, before changing states
      if (abs(cv - pv) < 5 || digitalRead(BTN2) == LOW)
      {
        if (lastAtTempMillis == 0) lastAtTempMillis = millis();
        // transition to next state, if temperature is reached for 75 seconds
        if (millis() - lastAtTempMillis > 75000 || digitalRead(BTN2) == LOW)
        {
          lastState = REFLOW;
          systemState = COOLDOWN;
          lastAtTempMillis = 0;
          stage_seconds = 0;
          beepCount = 2;
        }
      }
      else if (abs(cv - pv) > 5)
      {
        lastAtTempMillis = 0;
      }
      break;
    case COOLDOWN:
      // update setpoints and parameters
      cv = 0.0;
      slope = 0.0;

      // We need to make sure we are within 5 deg C of HIGH_HEAT_THRESHOLD for at least 5 seconds, before changing states
      if (abs(pv - HIGH_HEAT_THRESHOLD_C) < 5 || digitalRead(BTN1) == LOW)
      {
        if (lastAtTempMillis == 0) lastAtTempMillis = millis();
        // transition to next state, if temperature is reached for 5 seconds
        if (millis() - lastAtTempMillis > 5000 || digitalRead(BTN1) == LOW)
        {
          lastState = COOLDOWN;
          systemState = IDLE;
          lastAtTempMillis = 0;
          beepCount = 3;
        }
      }
      else if (abs(pv - HIGH_HEAT_THRESHOLD_C) > 5)
      {
        lastAtTempMillis = 0;
      }
      break;
  }
}
