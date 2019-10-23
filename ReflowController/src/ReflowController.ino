// ----------------------------------------------------------------------------
// Reflow Oven Controller
// (c) 2017 Debugged and restructured by David Sanz Kirbis
// (c) 2014 Karl Pitrich <karl@pitrich.com>
// (c) 2012-2013 Ed Simmons
// ----------------------------------------------------------------------------

#include <avr/eeprom.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <SPI.h>
#include <PDQ_GFX.h>             // PDQ: Core graphics library
#include <PDQ_ST7735_config.h>   // PDQ: ST7735 pins and other setup for this sketch
#include <PDQ_ST7735.h>          // PDQ: Hardware-specific driver library
#include <Menu.h>
#include <ClickEncoder.h>
#include <TimerOne.h>
#include <max6675.h>
#include <pidautotuner.h>

#include "portMacros.h"
#include "temperature.h"
#include "helpers.h"
#include "UI.h"
#include "globalDefs.h"

// ----------------------------------------------------------------------------
volatile uint32_t    timerTicks       = 0;
volatile uint8_t     phaseCounter     = 0;
static const uint8_t TIMER1_PERIOD_US = 100;
// ----------------------------------------------------------------------------
uint32_t lastUpdate        = 0;
uint32_t lastDisplayUpdate = 0;
State    previousState     = Idle;
bool     stateChanged      = false;
uint32_t stateChangedTicks = 0;
// ----------------------------------------------------------------------------
// PID
PID myPID(&heaterInput, &heaterOutput, &heaterSetpoint, heaterPID.soakKp, heaterPID.soakKi, heaterPID.soakKd, DIRECT);

PIDAutotuner tuner = PIDAutotuner();
PIDAutotuner tuner2 = PIDAutotuner();

/*************************************/
/*************************************/

typedef struct {
  double temp;
  uint16_t ticks;
} Temp_t;

Temp_t airTemp[NUM_TEMP_READINGS];

double readingsT1[NUM_TEMP_READINGS]; // the readings used to make a stable temp rolling average
double runningTotalRampRate;
double rateOfRise = 0;          // the result that is displayed
double totalT1 = 0;             // the running total
double averageT1 = 0;           // the average
uint8_t index = 0;              // the index of the current reading
uint8_t thermocoupleErrorCount;

// ----------------------------------------------------------------------------
// Ensure that Solid State Relais are off when starting
//
void setupPins(void) {
  pinAsOutput(PIN_HEATER);
  digitalLow(PIN_HEATER); // off
  pinAsInputPullUp(PIN_ZX);
  pinAsOutput(PIN_TC_CS);
  pinAsOutput(PIN_LCD_CS);
  pinAsOutput(PIN_TC_CS);
  #ifdef WITH_BEEPER
    pinAsOutput(PIN_BEEPER);
  #endif // WITH_BEEPER
}
// ----------------------------------------------------------------------------
void killRelayPins(void) {
  Timer1.stop();
  detachInterrupt(INT_ZX);
  digitalHigh(PIN_HEATER);
}

// ----------------------------------------------------------------------------
// wave packet control: only turn the solid state relais on for a percentage 
// of complete sinusoids (i.e. 1x 360°)

#define CHANNELS       1
#define CHANNEL_HEATER 0

typedef struct Channel_s {
  volatile uint8_t target; // percentage of on-time
  uint8_t state;           // current state counter
  uint32_t next;            // when the next change in output shall occur  
  bool action;             // hi/lo active
  uint8_t pin;             // io pin of solid state relais
} Channel_t;

Channel_t Channels[CHANNELS] = {
  // heater
  { 0, 0, 0, false, PIN_HEATER }
};

// delay to align relay activation with the actual zero crossing
uint16_t zxLoopDelay = 0;

#ifdef WITH_CALIBRATION
  // calibrate zero crossing: how many timerIsr happen within one zero crossing
  #define zxCalibrationLoops 128
  struct {
    volatile int8_t iterations;
    volatile uint8_t measure[zxCalibrationLoops];
  } zxLoopCalibration = {
    0, {}
  };
#endif // WITH_CALIBRATION

// ----------------------------------------------------------------------------
//                             ZERO CROSSING ISR
// ----------------------------------------------------------------------------
// Zero Crossing ISR; per ZX, process one channel per interrupt only
// NB: use native port IO instead of digitalWrite for better performance
void zeroCrossingIsr(void) {
  static uint8_t ch = 0;

  // reset phase control timer
  phaseCounter = 0;
  TCNT1 = 0;

  zeroCrossTicks++;

  // calculate wave packet parameters
  Channels[ch].state += Channels[ch].target;
  if (Channels[ch].state >= 100) {
    Channels[ch].state -= 100;
    Channels[ch].action = false;
  }
  else {
    Channels[ch].action = true;
  }
  Channels[ch].next = timerTicks + zxLoopDelay; // delay added to reach the next zx

  ch = ((ch + 1) % CHANNELS); // next channel

  #ifdef WITH_CALIBRATION
    if (zxLoopCalibration.iterations < zxCalibrationLoops) {
      zxLoopCalibration.iterations++;
    }
  #endif
}

// ----------------------------------------------------------------------------
//                                    TIMER ISR
// ----------------------------------------------------------------------------
// timer interrupt handling

void timerIsr(void) { // ticks with 100µS
  static uint32_t lastTicks = 0;

  // wave packet control for heater
  if (Channels[CHANNEL_HEATER].next > lastTicks // FIXME: this looses ticks when overflowing
      && timerTicks > Channels[CHANNEL_HEATER].next) 
  {
    if (Channels[CHANNEL_HEATER].action) digitalLow(Channels[CHANNEL_HEATER].pin); //digitalWriteFast(Channels[CHANNEL_HEATER].pin, HIGH);
    else digitalHigh(Channels[CHANNEL_HEATER].pin);//digitalWriteFast(Channels[CHANNEL_HEATER].pin, LOW);
    lastTicks = timerTicks;
  }

  // handle encoder + button
  if (!(timerTicks % 10)) {
    Encoder.service();
  }

  timerTicks++;

  #ifdef WITH_CALIBRATION
    if (zxLoopCalibration.iterations < zxCalibrationLoops) {
      zxLoopCalibration.measure[zxLoopCalibration.iterations]++;
    }
  #endif // WITH_CALIBRATION
}
// ----------------------------------------------------------------------------
void abortWithError(int error) {
  killRelayPins();
  displayError(error);
}
// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.print("nano Reflow controller v"),
  Serial.print(ver);
  Serial.println(" started");
  Serial.println("-----------------------------------");
  #if DEBUG >= 1
    Serial.print("Mains frequency = ");
    Serial.print( MAINS_FREQ );
    Serial.println("hz");
  #endif // DEBUG
  
  setupPins(); 
  setupTFT();

  if (firstRun()) {
    factoryReset();
    loadParameters(0);
  } 
  else {
    loadLastUsedProfile();
  }
 
  do {
   delay(500); // wait for MAX chip to stabilize
   readThermocouple();
  }
  while ((tcStat > 0) && (thermocoupleErrorCount++ < TC_ERROR_TOLERANCE));

  if ((tcStat != 0) || (thermocoupleErrorCount  >= TC_ERROR_TOLERANCE)) {
    abortWithError(tcStat);
  }

  // initialize moving average filter
  runningTotalRampRate = actualTemperature * NUM_TEMP_READINGS;
  for(int i = 0; i < NUM_TEMP_READINGS; i++) {
    airTemp[i].temp = actualTemperature;
  }

  loadPID();

  myPID.SetOutputLimits(0, 100); // max output 100%
  myPID.SetSampleTime(PID_SAMPLE_TIME);
  myPID.SetMode(AUTOMATIC);

  delay(1000);

  #ifdef WITH_BEEPER
    tone(PIN_BEEPER,BEEP_FREQ,100);
  #endif

#ifdef WITH_SPLASH
  displaySplash();
#endif // WITH_SPLASH

#ifdef WITH_CALIBRATION
  tft.setCursor(7, 99);  
  tft.print("Calibrating... ");
  delay(400);

  // FIXME: does not work reliably
  while (zxLoopDelay == 0) {
    if (zxLoopCalibration.iterations == zxCalibrationLoops) { // average tick measurements, dump 1st value
      for (int8_t l = 0; l < zxCalibrationLoops; l++) {
        zxLoopDelay += zxLoopCalibration.measure[l];
      }
      zxLoopDelay /= zxCalibrationLoops;
      zxLoopDelay -= 10; // compensating loop runtime
    }
  }
  tft.print(zxLoopDelay);
#else
  zxLoopDelay = DEFAULT_LOOP_DELAY;
#endif // WITH_CALIBRATION

//  setupMenu();
  menuExit(Menu::actionDisplay); // reset to initial state
  MenuEngine.navigate(&miCycleStart);
  currentState = Settings;
  menuUpdateRequest = true;

  Timer1.initialize(TIMER1_PERIOD_US);
  Timer1.attachInterrupt(timerIsr);

  attachInterrupt(INT_ZX, zeroCrossingIsr, RISING);
  delay(100);
}

uint32_t lastRampTicks;
uint32_t lastSoakTicks;

void updateRampSetpoint(bool down = false) {
  if (zeroCrossTicks > lastRampTicks + TICKS_PER_UPDATE) {
    double rate = (down) ? activeProfile.rampDownRate : activeProfile.rampUpRate;
    heaterSetpoint += (rate / (float)TICKS_PER_SEC * (zeroCrossTicks - lastRampTicks)) * ((down) ? -1 : 1);
    lastRampTicks = zeroCrossTicks;
  }
}

void updateSoakSetpoint(bool down = false) {
  if (zeroCrossTicks > lastSoakTicks + TICKS_PER_UPDATE) {
    double rate = (activeProfile.soakTempB-activeProfile.soakTempA)/(float)activeProfile.soakDuration;
    heaterSetpoint += (rate / (float)TICKS_PER_SEC * (zeroCrossTicks - lastSoakTicks)) * ((down) ? -1 : 1);
    lastSoakTicks = zeroCrossTicks;
  }
}

void loop(void) 
{
  // --------------------------------------------------------------------------
  // handle encoder
  //
  encMovement = Encoder.getValue();
  if (encMovement) {
    encAbsolute += encMovement;
    if (currentState == Settings) {
      MenuEngine.navigate((encMovement > 0) ? MenuEngine.getNext() : MenuEngine.getPrev());
      menuUpdateRequest = true;
    }
  }

  // --------------------------------------------------------------------------
  // handle button
  //
  switch (Encoder.getButton()) {
    case ClickEncoder::Clicked:
      if (currentState == Complete) { // at end of cycle; reset at click
        menuExit(Menu::actionDisplay); // reset to initial state
        MenuEngine.navigate(&miCycleStart);
        currentState = Settings;
        menuUpdateRequest = true;
      }
      else if (currentState < UIMenuEnd) {
        menuUpdateRequest = true;
        MenuEngine.invoke();
      }
      else if (currentState > UIMenuEnd) {
        currentState = CoolDown;
      }
      break;

    case ClickEncoder::DoubleClicked:
      if (currentState < UIMenuEnd) {
        if (MenuEngine.getParent() != &miExit) {
          MenuEngine.navigate(MenuEngine.getParent());
          menuUpdateRequest = true;
        }
      }
      break;

    default:
      break;
  }

  // --------------------------------------------------------------------------
  // update current menu item while in edit mode
  //
  if (currentState == Edit) {
    if (MenuEngine.currentItem != &Menu::NullItem) {
      MenuEngine.executeCallbackAction(Menu::actionDisplay);      
    }
  }

  // --------------------------------------------------------------------------
  // handle menu update
  //
  if (menuUpdateRequest) {
    menuUpdateRequest = false;
    if (currentState < UIMenuEnd && !encMovement && currentState != Edit && previousState != Edit) { // clear menu on child/parent navigation
      tft.fillScreen(ST7735_WHITE);
    }  
    MenuEngine.render(renderMenuItem, menuItemsVisible);
  }

  // --------------------------------------------------------------------------
  // track state changes
  //
  if (currentState != previousState) {
    stateChangedTicks = zeroCrossTicks;
    stateChanged = true;
    previousState = currentState;
  }

  // --------------------------------------------------------------------------

  if (zeroCrossTicks - lastUpdate >= TICKS_PER_UPDATE) {
    uint32_t deltaT = zeroCrossTicks - lastUpdate;
    lastUpdate = zeroCrossTicks;

    readThermocouple(); // should be sufficient to read it every 250ms or 500ms   

    if (tcStat > 0) {
      thermocoupleErrorCount++;
       if ((thermocoupleErrorCount > TC_ERROR_TOLERANCE) && (currentState != Edit)) {
        abortWithError(tcStat);
      } else thermocoupleErrorCount = 0;
    }
    else {
      thermocoupleErrorCount = 0;
      #if 0 // verbose thermocouple error bits
        tft.setCursor(10, 40);
        for (uint8_t mask = B111; mask; mask >>= 1) {
          tft.print(mask & tSensor.stat ? '1' : '0');
        }
      #endif
      // rolling average of the temp T1 and T2
      totalT1 -= readingsT1[index];       // subtract the last reading
      readingsT1[index] = actualTemperature;
      totalT1 += readingsT1[index];       // add the reading to the total
      index = (index + 1) % NUM_TEMP_READINGS;  // next position
      averageT1 = totalT1 / (float)NUM_TEMP_READINGS;  // calculate the average temp
    
      // need to keep track of a few past readings in order to work out rate of rise
      for (int i = 1; i < NUM_TEMP_READINGS; i++) { // iterate over all previous entries, moving them backwards one index
        airTemp[i - 1].temp = airTemp[i].temp;
        airTemp[i - 1].ticks = airTemp[i].ticks;     
      }
    
      airTemp[NUM_TEMP_READINGS - 1].temp = averageT1; // update the last index with the newest average
      airTemp[NUM_TEMP_READINGS - 1].ticks = (uint16_t)deltaT;
    
      // calculate rate of temperature change
      uint32_t collectTicks = 0;
      for (int i = 0; i < NUM_TEMP_READINGS; i++) {
        collectTicks += airTemp[i].ticks;
      }
      float tempDiff = (airTemp[NUM_TEMP_READINGS - 1].temp - airTemp[0].temp);
      float timeDiff = collectTicks / (float)(TICKS_PER_SEC);
        
      rampRate = tempDiff / timeDiff;
     
      heaterInput = airTemp[NUM_TEMP_READINGS - 1].temp; // update the variable the PID reads
    }
    // display update
    if (zeroCrossTicks - lastDisplayUpdate >= TICKS_TO_REDRAW) {
      lastDisplayUpdate = zeroCrossTicks;
      if (currentState > UIMenuEnd) {
        updateProcessDisplay();
      }
      else displayThermocoupleData(1, tft.height()-16);
    }

    switch (currentState) {
      case RampToSoak:
        if (stateChanged) {
          lastRampTicks = zeroCrossTicks;
          stateChanged = false;
          heaterOutput = 50;
          myPID.SetMode(AUTOMATIC);
          myPID.SetControllerDirection(DIRECT);
          myPID.SetTunings(heaterPID.soakKp, heaterPID.soakKi, heaterPID.soakKd);
          heaterSetpoint = heaterInput;
          #ifdef WITH_BEEPER
              tone(PIN_BEEPER,BEEP_FREQ,100);
          #endif
        }

        updateRampSetpoint();

        if (heaterSetpoint >= activeProfile.soakTempA - 1) {
          currentState = Soak;
        }
        break;

      case Soak:
        if (stateChanged) {
          lastSoakTicks = zeroCrossTicks;
          stateChanged = false;
          heaterSetpoint = activeProfile.soakTempA;
        }

        updateSoakSetpoint();

        if (zeroCrossTicks - stateChangedTicks >= (uint32_t)activeProfile.soakDuration * TICKS_PER_SEC) {
          currentState = RampUp;
        }
        break;

      case RampUp:
        if (stateChanged) {
          stateChanged = false;
          lastRampTicks = zeroCrossTicks;
          myPID.SetTunings(heaterPID.peakKp, heaterPID.peakKi, heaterPID.peakKd);
        }

        updateRampSetpoint();

        if (heaterSetpoint >= activeProfile.peakTemp - 1) {
          heaterSetpoint = activeProfile.peakTemp;
          currentState = Peak;
        }
        break;

      case Peak:
        if (stateChanged) {
          stateChanged = false;
          heaterSetpoint = activeProfile.peakTemp;
        }

        if (zeroCrossTicks - stateChangedTicks >= (uint32_t)activeProfile.peakDuration * TICKS_PER_SEC) {
          currentState = RampDown;
        }
        break;

      case RampDown:
        if (stateChanged) {
          stateChanged = false;
          lastRampTicks = zeroCrossTicks;
          myPID.SetControllerDirection(REVERSE);
          heaterSetpoint = activeProfile.peakTemp - 15; // get it all going with a bit of a kick! v sluggish here otherwise, too hot too long
          #ifdef WITH_BEEPER
            tone(PIN_BEEPER, BEEP_FREQ, 3000);  // Beep as a reminder that CoolDown starts (and maybe open up the oven door for fast enough cooldown)
          #endif // WITH_BEEPER
        }

        updateRampSetpoint(true);

        if (heaterSetpoint <= idleTemp) {
          currentState = CoolDown;
        }
        break;

      case CoolDown:
        if (stateChanged) {
          stateChanged = false;
          myPID.SetControllerDirection(REVERSE);
          heaterSetpoint = idleTemp;
        }

        if (heaterInput < (idleTemp + 5)) {
          currentState = Complete;
          myPID.SetMode(MANUAL);
          heaterOutput = 0;
          #ifdef WITH_BEEPER
            tone(PIN_BEEPER, BEEP_FREQ, 500);  //End Beep
            delay(1500);
            tone(PIN_BEEPER, BEEP_FREQ, 500);
            delay(1500);
            tone(PIN_BEEPER, BEEP_FREQ, 1500);
          #endif
        }
        break;

      case PreTune:
        if (stateChanged) {
         #if DEBUG >= 1
            Serial.println("Started Pretune");
          #endif
          stateChanged = false;
        	myPID.SetMode(MANUAL);
        	heaterSetpoint = round((activeProfile.soakTempA + activeProfile.soakTempB) / 2);
          heaterOutput = 100;
        }

        if (actualTemperature >=  heaterSetpoint - 20) {
          currentState = TuneSoak;
        }
        break;

      case TuneSoak:
        if (stateChanged) {
          #if DEBUG >= 1
            Serial.println("Soak Tuning started");
          #endif 
          stateChanged = false;
          //heaterSetpoint = round((activeProfile.soakTempA + activeProfile.soakTempB) / 2);

          tuner.setTargetInputValue(heaterSetpoint);
          tuner.setLoopInterval(100000);
          tuner.setOutputRange(0, 100);
          tuner.setZNMode(PIDAutotuner::ZNModeLessOvershoot);
          tuner.startTuningLoop();
        }

        if (tuner.isFinished()) {
          currentState = TunePeak;

          heaterPID.soakKp = tuner.getKp();
          heaterPID.soakKi = tuner.getKi();
          heaterPID.soakKd = tuner.getKd();

          #if DEBUG >= 1
            Serial.println("Soak Tuning Finished");
            Serial.print("Soak Kp: ");
            Serial.println(heaterPID.soakKp);
            Serial.print("Soak Ki: ");
            Serial.println(heaterPID.soakKi);
            Serial.print("Soak Kd: ");
            Serial.println(heaterPID.soakKd);
          #endif // DEBUG

        }
        break;    

      case TunePeak:
        {
          if (stateChanged) {
            #if DEBUG >= 1
              Serial.println("Peak tuning started");
            #endif
            stateChanged = false;
            heaterSetpoint = activeProfile.peakTemp;
            
            tuner.setTargetInputValue(activeProfile.peakTemp);
            tuner.startTuningLoop();
          }

          if (tuner.isFinished()) {
            heaterOutput = 0;
            currentState = CoolDown;

            heaterPID.peakKp = tuner.getKp();
            heaterPID.peakKi = tuner.getKi();
            heaterPID.peakKd = tuner.getKd();

            savePID();

            tft.fillRect(10, 40, 120, 60, ST7735_RED);
            tft.setTextColor(ST7735_WHITE, ST7735_RED);

            tft.setCursor(50, 50);
            tft.print("Soak");
            tft.setCursor(90, 50);
            tft.print("Peak");

            tft.setCursor(20, 60);
            tft.print("Kp: "); 
            tft.setCursor(20, 70);
            tft.print("Ki: "); 
            tft.setCursor(20, 80);
            tft.print("Kd: "); 

            tft.setCursor(50, 60);
            printDouble(heaterPID.soakKp);
            tft.setCursor(50, 70);
            printDouble(heaterPID.soakKi);
            tft.setCursor(50, 80);
            printDouble(heaterPID.soakKd);

            tft.setCursor(90, 60);
            printDouble(heaterPID.peakKp);
            tft.setCursor(90, 70);
            printDouble(heaterPID.peakKi);
            tft.setCursor(90, 80);
            printDouble(heaterPID.peakKd);

            #if DEBUG >= 1
              Serial.println("Peak Tuning finished");
              Serial.print("Peak Kp: ");
              Serial.println(heaterPID.peakKp);
              Serial.print("Peak Ki: ");
              Serial.println(heaterPID.peakKi);
              Serial.print("Peak Kd: ");
              Serial.println(heaterPID.peakKd);
            #endif // DEBUG
          }
        }
        break;      

      default:
        break;
    }

  }

  // safety check that we're not doing something stupid. 
  // if the thermocouple is wired backwards, temp goes DOWN when it increases
  // during cooling, the t962a lags a long way behind, hence the hugely lenient cooling allowance.
  // both of these errors are blocking and do not exit!
  //if (heaterSetpoint > heaterInput + 50) abortWithError(10); // if we're 50 degree cooler than setpoint, abort
  //if (heaterInput > heaterSetpoint + 50) abortWithError(20); // or 50 degrees hotter, also abort
  long milliseconds;

  if (currentState == RampToSoak || currentState == Soak || currentState == RampUp || currentState == Peak || currentState == PreTune) {
    myPID.Compute();
    heaterPower = heaterOutput;
  } else if (currentState == TuneSoak || currentState == TunePeak) {
    milliseconds = millis();
    heaterPower = tuner.tunePID(actualTemperature);
    while (millis() - milliseconds < 100) delay(1);
  } else {
    heaterPower = 0;
  }


  Channels[CHANNEL_HEATER].target = heaterPower;
}


void saveProfile(unsigned int targetProfile, bool quiet) {
  activeProfileId = targetProfile;

  if (!quiet) {
    memoryFeedbackScreen(activeProfileId, false);
  }
  saveParameters(activeProfileId); // activeProfileId is modified by the menu code directly, this method is called by a menu action

  if (!quiet) delay(500);
}

#define WITH_CHECKSUM 1

bool firstRun() { 
  #ifndef ALWAYS_FIRST_RUN
    // if all bytes of a profile in the middle of the eeprom space are 255, we assume it's a first run
    unsigned int offset = 15 * sizeof(Profile_t);

    for (uint16_t i = offset; i < offset + sizeof(Profile_t); i++) {
      if (EEPROM.read(i) != 255) {
        return false;
      }
    }
  #endif
  return true;
}
