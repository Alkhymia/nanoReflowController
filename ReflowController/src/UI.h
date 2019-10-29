#ifndef UI_H
#define UI_H

#include <PDQ_GFX.h>        // PDQ: Core graphics library
#include "PDQ_ST7735_config.h"      // PDQ: ST7735 pins and other setup for this sketch
#include <PDQ_ST7735.h>     // PDQ: Hardware-specific driver library                                     
#include <Menu.h>
#include <ClickEncoder.h>
#include "globalDefs.h"
#include "eepromHelpers.h"
#include "config.h"

#define MENU_TEXT_XPOS 5
#define MENU_BAR_XPOS 3

// ----------------------------------------------------------------------------
// Hardware Configuration 

// 1.44" or 1.8" TFT via SPI -> breadboard

#define TEMPERATURE_WINDOW 1.2 // n times the profile's maximum temperature

// ----------------------------------------------------------------------------

PDQ_ST7735 tft;     // PDQ: create LCD object (using pins in "PDQ_ST7735_config.h")

// ------------ menu

Menu::Engine MenuEngine;

const uint8_t menuItemHeight = 12;
const uint8_t menuItemsVisible = 8;
bool menuUpdateRequest = true;
bool initialProcessDisplay = false;

// track menu item state to improve render preformance
typedef struct {
  const Menu::Item_t *mi;
  uint8_t pos;
  bool current;
} LastItemState_t;

LastItemState_t currentlyRenderedItems[menuItemsVisible];

// ------------ encoder
ClickEncoder Encoder(PIN_ENC_A, PIN_ENC_B, PIN_ENC_BTN, ENC_STEPS_PER_NOTCH, IS_ENC_ACTIVE);
int16_t encMovement;
int16_t encAbsolute;
int16_t encLastAbsolute = -1;

// ------------ 

float pxPerSec;
float pxPerC;
uint16_t xOffset; // used for wraparound on x axis

void setupTFT() {

  FastPin<ST7735_RST_PIN>::setOutput();
  FastPin<ST7735_RST_PIN>::hi();
  FastPin<ST7735_RST_PIN>::lo();
  delay(1);
  FastPin<ST7735_RST_PIN>::hi();
  
  tft.begin();
  
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setRotation(LCD_ROTATION);
  tft.fillScreen(ST7735_WHITE);
  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
}

void   setupMenu() {
/*    menuExit(Menu::actionDisplay); // reset to initial state
  MenuEngine.navigate(&miCycleStart);
  currentState = Settings;
  menuUpdateRequest = true;*/
}

void displaySplash() {
  tft.fillScreen(ST7735_WHITE);
       
  tft.setTextColor(ST7735_BLACK);
  // splash screen
  tft.setCursor(2, 30);
  tft.setTextSize(2);
  tft.print("Reflow");
  tft.setCursor(tft.width() - 120, 48);
  tft.print("Controller");
  tft.setTextSize(1);
  tft.setCursor(52, 67);
  tft.print("v"); tft.print(ver);
  
  tft.setCursor(0, 99);
  tft.print("(c)2014 karl@pitrich.com");
  tft.setCursor(0, 109);
  tft.print("(c)2017 Dasaki");
  tft.setCursor(0, 119);
  tft.print("(c)2019 gaGGi");
  delay(3000);
}


void displayError(int error) {

  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE, ST7735_RED);
  tft.fillScreen(ST7735_RED);

  tft.setCursor(10, 10);
  
  if (error < 9) {
    tft.println("Thermocouple Error");
    tft.setCursor(10, 30);
    switch (error) {
      case 0b001:
        tft.println("Open Circuit");
        break;
      case 0b010:
        tft.println("GND Short");
        break;
      case 0b100:
        tft.println("VCC Short");
        break;
    }
    tft.setCursor(10, 60);
    tft.println("Power off,");
    tft.setCursor(10, 75);
    tft.println("check connections");
  }
  else {
    tft.println("Temperature"); 
    tft.setCursor(10, 30);
    tft.println("following error");
    tft.setCursor(10, 45);
    tft.print("during ");
    tft.println((error == 10) ? "heating" : "cooling");
  }
  #ifdef WITH_BEEPER
    tone(PIN_BEEPER, BEEP_FREQ, 2000);  // Error Beep
  #endif
  while (1) { // stop
    ;
  }
}

// ----------------------------------------------------------------------------

void alignRightPrefix(uint16_t v) {
  if (v < 1e2) tft.print(' '); 
  if (v < 1e1) tft.print(' ');
}

// ----------------------------------------------------------------------------

void displayThermocoupleData(uint8_t xpos, uint8_t ypos) {
  tft.setCursor(xpos, ypos);
  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

  // temperature
  tft.setTextSize(2);
  alignRightPrefix((int)actualTemperature);
  switch (tcStat) {
    case 0:
      tft.print((uint8_t)actualTemperature);
      tft.print("\367C");
      break;
    case 1:
      tft.print("---");
      break;
  }
}

void displayHeaterPowerData(uint8_t xpos, uint8_t ypos) {
  tft.setCursor(xpos, ypos);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

  tft.print("\xef");
  alignRightPrefix((int)heaterPower); 
  tft.print((int)heaterPower);
  tft.print('%');
}

// ----------------------------------------------------------------------------

void clearLastMenuItemRenderState() {
  // memset(&currentlyRenderedItems, 0xff, sizeof(LastItemState_t) * menuItemsVisible);
  for (uint8_t i = 0; i < menuItemsVisible; i++) {
    currentlyRenderedItems[i].mi = NULL;
    currentlyRenderedItems[i].pos = 0xff;
    currentlyRenderedItems[i].current = false;
  }
}

// ----------------------------------------------------------------------------

extern const 
  Menu::Item_t miCycleStart,
    miRampUpRate,
    miSoakTempA,
    miSoakTempB,
    miSoakTime,
    miPeakTemp,
    miPeakTime,
    miRampDnRate,
    miLoadProfile,
    miSaveProfile,
    miPidSoakP,
    miPidSoakI,
    miPidSoakD,
    miPidPeakP,
    miPidPeakI,
    miPidPeakD;

// ----------------------------------------------------------------------------

bool menuExit(const Menu::Action_t a) {
  clearLastMenuItemRenderState();
  MenuEngine.lastInvokedItem = &Menu::NullItem;
  menuUpdateRequest = false;
  return false;
}

// ----------------------------------------------------------------------------

bool menuDummy(const Menu::Action_t a) {
  return true;
}

// ----------------------------------------------------------------------------

void printDouble(double val, uint8_t precision = 1) {
  ftoa(buf, val, precision);
  tft.print(buf);
}

// ----------------------------------------------------------------------------

void getItemValuePointer(const Menu::Item_t *mi, double **d, int16_t **i) {
  if (mi == &miRampUpRate)  *d = &activeProfile.rampUpRate;
  if (mi == &miRampDnRate)  *d = &activeProfile.rampDownRate;
  if (mi == &miSoakTime)    *i = &activeProfile.soakDuration;
  if (mi == &miSoakTempA)   *i = &activeProfile.soakTempA;
  if (mi == &miSoakTempB)   *i = &activeProfile.soakTempB;
  if (mi == &miPeakTime)    *i = &activeProfile.peakDuration;
  if (mi == &miPeakTemp)    *i = &activeProfile.peakTemp;
  if (mi == &miPidSoakP)    *d = &heaterPID.soakKp;
  if (mi == &miPidSoakI)    *d = &heaterPID.soakKi;
  if (mi == &miPidSoakD)    *d = &heaterPID.soakKd;
  if (mi == &miPidPeakP)    *d = &heaterPID.peakKp;
  if (mi == &miPidPeakI)    *d = &heaterPID.peakKi;
  if (mi == &miPidPeakD)    *d = &heaterPID.peakKd;
}

// ----------------------------------------------------------------------------

bool isPidSetting(const Menu::Item_t *mi) {
  return mi == &miPidSoakP || mi == &miPidSoakI || mi == &miPidSoakD || mi == &miPidPeakP || mi == &miPidPeakI || mi == &miPidPeakD;
}

bool isRampSetting(const Menu::Item_t *mi) {
  return mi == &miRampUpRate || mi == &miRampDnRate;
}

// ----------------------------------------------------------------------------

bool getItemValueLabel(const Menu::Item_t *mi, char *label) {
  int16_t *iValue = NULL;
  double  *dValue = NULL;
  char *p;
  
  getItemValuePointer(mi, &dValue, &iValue);

  if (isRampSetting(mi) || isPidSetting(mi)) {
    p = label;
    ftoa(p, *dValue, (isPidSetting(mi)) ? 2 : 1); // need greater precision with pid values
    p = label;
    
    if (isRampSetting(mi)) {
      while(*p != '\0') p++;
      *p++ = 0xf7; *p++ = 'C'; *p++ = '/'; *p++ = 's';
      *p = '\0';
    }
  }
  else {
    if (mi == &miPeakTemp || mi == &miSoakTempA || mi == &miSoakTempB ) {
      itostr(label, *iValue, (char*)"\367C");
    }
    if (mi == &miPeakTime || mi == &miSoakTime) {
      itostr(label, *iValue, (char*)"s");
    }
  }

  return dValue || iValue;
}

// ----------------------------------------------------------------------------

bool menu_editNumericalValue(const Menu::Action_t action) { 
  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    tft.setTextSize(1);
    if (initial) {
      tft.fillRect(15, 85, 132, 18, ST7735_RED);
      tft.setTextColor(ST7735_WHITE, ST7735_RED);
      tft.setCursor(22, 90);
      tft.print("Edit & click to save");
      Encoder.setAccelerationEnabled(true);
    }

    for (uint8_t i = 0; i < menuItemsVisible; i++) {
      if (currentlyRenderedItems[i].mi == MenuEngine.currentItem) {
        uint8_t y = currentlyRenderedItems[i].pos * menuItemHeight + 2;

        if (initial) {
          tft.fillRect(59 + MENU_TEXT_XPOS, y - 1, 60, menuItemHeight - 2, ST7735_RED);
        }

        tft.setCursor(60 + MENU_TEXT_XPOS, y);
        break;
      }
    }

    tft.setTextColor(ST7735_WHITE, ST7735_RED);

    int16_t *iValue = NULL;
    double  *dValue = NULL;
    getItemValuePointer(MenuEngine.currentItem, &dValue, &iValue);

    if (isRampSetting(MenuEngine.currentItem) || isPidSetting(MenuEngine.currentItem)) {
      double tmp;
      double factor = (isPidSetting(MenuEngine.currentItem)) ? 100 : 10;
      
      if (initial) {
        tmp = *dValue;
        tmp *= factor;
        encAbsolute = (int16_t)tmp;
      }
      else {
        tmp = encAbsolute;
        tmp /= factor;
        *dValue = tmp;
      }      
    }
    else {
      if (initial) encAbsolute = *iValue;
      else *iValue = encAbsolute;
    }

    getItemValueLabel(MenuEngine.currentItem, buf);
    tft.print(buf);
    tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
  }

  if (action == Menu::actionParent || action == Menu::actionTrigger) {
    clearLastMenuItemRenderState();
    menuUpdateRequest = true;
    MenuEngine.lastInvokedItem = &Menu::NullItem;
    tft.fillScreen(ST7735_WHITE);

    if (currentState == Edit) { // leave edit mode, return to menu
      if (isPidSetting(MenuEngine.currentItem)) {
        savePID();
      }
      // don't autosave profile, so that one can do "save as" without overwriting the current profile

      currentState = Settings;
      Encoder.setAccelerationEnabled(false);
      return false;
    }

    return true;
  }
  return(EXIT_SUCCESS);
}

bool menu_manualHeating(const Menu::Action_t action) {
  bool init=true;
  if ((init=((action == Menu::actionTrigger || action == Menu::actionDisplay) && currentState != Edit)) || action == Menu::actionDisplay) {
    if (init) {
      currentState = Edit;
      tft.fillRect(20, 20, 120, 80, ST7735_RED);
      tft.setTextSize(1);
      tft.setTextColor(ST7735_WHITE, ST7735_RED);
      tft.setCursor(40, 80);
      tft.print("Click to stop");
      tft.setTextSize(2);
      tft.setCursor(100,40);
      tft.print("\367C");
      Encoder.setAccelerationEnabled(true);
      encAbsolute = actualTemperature;
    }
    if (encAbsolute > 250) encAbsolute = 250;
    if (encAbsolute <  0) encAbsolute =  0;

    tft.setCursor(45, 40);
    tft.setTextSize(3);
    tft.setTextColor(ST7735_WHITE, ST7735_RED);
    alignRightPrefix(encAbsolute);
    tft.print(encAbsolute);
  } else if ((action == Menu::actionParent || action == Menu::actionTrigger ) && currentState == Edit) {
    tft.fillScreen(ST7735_WHITE);
    currentState = Settings;
    clearLastMenuItemRenderState();
  }
  return true;
}

// ----------------------------------------------------------------------------

void factoryReset() {
  makeDefaultProfile();
  
  tft.fillScreen(ST7735_BLUE);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
  tft.setCursor(10, 50);
  tft.print("Resetting...");

  // then save the same profile settings into all slots
  for (uint8_t i = 0; i < maxProfiles; i++) {
    saveParameters(i);
  }

  heaterPID.soakKp = FACTORY_KP; // 0.60; 
  heaterPID.soakKi = FACTORY_KI; //0.01;
  heaterPID.soakKd = FACTORY_KD; //19.70;

  heaterPID.peakKp = FACTORY_KP; // 0.60; 
  heaterPID.peakKi = FACTORY_KI; //0.01;
  heaterPID.peakKd = FACTORY_KD; //19.70;
  savePID();

  activeProfileId = 0;
  saveLastUsedProfile();

  delay(500);
}

// ----------------------------------------------------------------------------

bool menu_factoryReset(const Menu::Action_t action) {
  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    if (initial) { // TODO: add eyecandy: colors or icons
      tft.fillRect(15, 75, 130, 30, ST7735_RED);
      tft.setTextColor(ST7735_WHITE, ST7735_RED);
      tft.setTextSize(1);
      tft.setCursor(22, 80);
      tft.print("Click to confirm");
      tft.setCursor(22, 90);
      tft.print("Doubleclick to exit");
    }
  }

  if (action == Menu::actionTrigger) { // do it
    factoryReset();
    tft.fillScreen(ST7735_WHITE);
    MenuEngine.navigate(MenuEngine.getParent());
    return false;
  }

  if (action == Menu::actionParent) {
    if (currentState == Edit) { // leave edit mode only, returning to menu
      tft.fillScreen(ST7735_WHITE);
      currentState = Settings;
      clearLastMenuItemRenderState();
      return false;
    }
  }

return(EXIT_SUCCESS);
}

void memoryFeedbackScreen(uint8_t profileId, bool loading) {
  tft.fillScreen(ST7735_GREEN);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_BLACK);
  tft.setCursor(10, 50);
  tft.print(loading ? "Loading" : "Saving");
  tft.print(" profile ");
  tft.print(profileId);  
}

void saveProfile(unsigned int targetProfile, bool quiet) {
  activeProfileId = targetProfile;

  if (!quiet) {
    memoryFeedbackScreen(activeProfileId, false);
  }
  saveParameters(activeProfileId); // activeProfileId is modified by the menu code directly, this method is called by a menu action

  if (!quiet) delay(500);
}

void loadProfile(unsigned int targetProfile) {
  memoryFeedbackScreen(targetProfile, true);
  //bool ok = loadParameters(targetProfile);
  loadParameters(targetProfile);

  activeProfileId = targetProfile;
  saveLastUsedProfile();

  delay(500);
}

bool menu_saveLoadProfile(const Menu::Action_t action) {
  bool isLoad = MenuEngine.currentItem == &miLoadProfile;

  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    tft.setTextColor(ST7735_WHITE, ST7735_RED);
    tft.setTextSize(1);

    if (initial) {
      encAbsolute = activeProfileId;      
      tft.fillRect(15, 75, 130, 30, ST7735_RED);
      tft.setTextColor(ST7735_WHITE, ST7735_RED);
      tft.setCursor(22, 90);
      tft.print("Doubleclick to exit");
    }

    if (encAbsolute > maxProfiles) encAbsolute = maxProfiles;
    if (encAbsolute <  0) encAbsolute =  0;

    tft.setCursor(22, 80);
    tft.print("Click to ");
    tft.print((isLoad) ? "load" : "save");
    //tft.setTextColor(ST7735_WHITE, ST7735_RED);
    alignRightPrefix(encAbsolute);
    tft.print(encAbsolute);
  }

  if (action == Menu::actionTrigger) {
    Serial.println("actionTrigger");
    (isLoad) ? loadProfile(encAbsolute) : saveProfile(encAbsolute, false);
    tft.fillScreen(ST7735_WHITE);
    MenuEngine.navigate(MenuEngine.getParent());
    return false;
  }

  if (action == Menu::actionParent) {    
    Serial.println("actionParent");
    if (currentState == Edit) { // leave edit mode only, returning to menu
      tft.fillScreen(ST7735_WHITE);
      currentState = Settings;
      clearLastMenuItemRenderState();
      return false;
    }
  }

return(EXIT_SUCCESS);
}

// ----------------------------------------------------------------------------

bool menu_cycleStart(const Menu::Action_t action) {
  if (action == Menu::actionDisplay) {
    startCycleZeroCrossTicks = zeroCrossTicks;
    menuExit(action);
    
    if(MenuEngine.currentItem == &miCycleStart)
    {
      currentState = RampToSoak;      
    }
    else
    {
      currentState = PreTune;      
    }

    initialProcessDisplay = false;
    menuUpdateRequest = false;
  }

  return true;
}

// ----------------------------------------------------------------------------

void renderMenuItem(const Menu::Item_t *mi, uint8_t pos) {
  //ScopedTimer tm("  render menuitem");
  bool isCurrent = MenuEngine.currentItem == mi;
  uint8_t y = pos * menuItemHeight + 2;

  if (currentlyRenderedItems[pos].mi == mi 
      && currentlyRenderedItems[pos].pos == pos 
      && currentlyRenderedItems[pos].current == isCurrent) 
  {
    return; // don't render the same item in the same state twice
  }

  tft.setCursor(MENU_TEXT_XPOS, y);
  tft.setTextSize(1);

  // menu cursor bar
  tft.fillRect(MENU_BAR_XPOS, y - 2, tft.width() - 16, menuItemHeight, isCurrent ? ST7735_BLUE : ST7735_WHITE);
  if (isCurrent) tft.setTextColor(ST7735_WHITE, ST7735_BLUE);
  else tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

  tft.print(MenuEngine.getLabel(mi));

  // show values if in-place editable items
  if (getItemValueLabel(mi, buf)) {
    tft.print(' '); tft.print(buf); tft.print("   ");
  }

  // mark items that have children
  if (MenuEngine.getChild(mi) != &Menu::NullItem) {
    tft.print(" \x10   "); // 0x10 -> filled right arrow
  }

  currentlyRenderedItems[pos].mi = mi;
  currentlyRenderedItems[pos].pos = pos;
  currentlyRenderedItems[pos].current = isCurrent;
}

// ----------------------------------------------------------------------------
//       Name,             Label,             Next,           Previous,       Parent,         Child,          Callback

MenuItem(miExit,           "",                Menu::NullItem, Menu::NullItem, Menu::NullItem, miCycleStart,   menuExit);
MenuItem(miCycleStart,    "Start Cycle",      miManual,       Menu::NullItem, miExit,         Menu::NullItem, menu_cycleStart);
MenuItem(miManual,        "Manual Heating",   miEditProfile,  miCycleStart,   miExit,         Menu::NullItem, menu_manualHeating);
MenuItem(miEditProfile,   "Edit Profile",     miLoadProfile,  miManual,       miExit,         miRampUpRate,   menuDummy);
  MenuItem(miRampUpRate,  "Ramp up  ",          miSoakTempA,    Menu::NullItem, miEditProfile,  Menu::NullItem, menu_editNumericalValue);
  MenuItem(miSoakTempA,   "Soak temp A",        miSoakTempB,    miRampUpRate,   miEditProfile,  Menu::NullItem, menu_editNumericalValue);
  MenuItem(miSoakTempB,   "Soak temp B",        miSoakTime,     miSoakTempA,    miEditProfile,  Menu::NullItem, menu_editNumericalValue);
  MenuItem(miSoakTime,    "Soak time",          miPeakTemp,     miSoakTempB,    miEditProfile,  Menu::NullItem, menu_editNumericalValue);
  MenuItem(miPeakTemp,    "Peak temp",          miPeakTime,     miSoakTime,     miEditProfile,  Menu::NullItem, menu_editNumericalValue);
  MenuItem(miPeakTime,    "Peak time",          miRampDnRate,   miPeakTemp,     miEditProfile,  Menu::NullItem, menu_editNumericalValue);
  MenuItem(miRampDnRate,  "Ramp down",          Menu::NullItem, miPeakTime,     miEditProfile,  Menu::NullItem, menu_editNumericalValue);
MenuItem(miLoadProfile,   "Load Profile",     miSaveProfile,  miEditProfile,  miExit,         Menu::NullItem, menu_saveLoadProfile);
MenuItem(miSaveProfile,   "Save Profile",     miPidSettings,  miLoadProfile,  miExit,         Menu::NullItem, menu_saveLoadProfile);
MenuItem(miPidSettings,   "PID Settings",     miFactoryReset, miSaveProfile,  miExit,         miPidTune,  menuDummy);
  MenuItem(miPidTune,     "Start PID Autotune", miPidSoakP,     Menu::NullItem, miPidSettings,  Menu::NullItem, menu_cycleStart);
  MenuItem(miPidSoakP,    "Soak Kp  ",          miPidSoakI,     miPidTune,      miPidSettings,  Menu::NullItem, menu_editNumericalValue);
  MenuItem(miPidSoakI,    "Soak Ki  ",          miPidSoakD,     miPidSoakP,     miPidSettings,  Menu::NullItem, menu_editNumericalValue);
  MenuItem(miPidSoakD,    "Soak Kd  ",          miPidPeakP,     miPidSoakI,     miPidSettings,  Menu::NullItem, menu_editNumericalValue);
  MenuItem(miPidPeakP,    "Peak Kp  ",          miPidPeakI,     miPidSoakD,     miPidSettings,  Menu::NullItem, menu_editNumericalValue);
  MenuItem(miPidPeakI,    "Peak Ki  ",          miPidPeakD,     miPidPeakP,     miPidSettings,  Menu::NullItem, menu_editNumericalValue);
  MenuItem(miPidPeakD,    "Peak Kd  ",          Menu::NullItem, miPidPeakI,     miPidSettings,  Menu::NullItem, menu_editNumericalValue);
MenuItem(miFactoryReset,  "Factory Reset",    Menu::NullItem, miPidSettings,  miExit,         Menu::NullItem, menu_factoryReset);

// ----------------------------------------------------------------------------
void drawInitialProcessDisplay()
{
  const uint8_t h = tft.height() - 42;
  const uint8_t w = tft.width() - 24;
  const uint8_t yOffset = 30; // space not available for graph  
  double tmp;
  double estimatedTotalTime;
  initialProcessDisplay = true;

    tft.fillScreen(ST7735_WHITE);
    tft.fillRect(0, 0, tft.width(), menuItemHeight, ST7735_BLUE);
    tft.setCursor(1, 2);

    uint16_t maxTemp;

    if(currentState == PreTune) {
      tft.print("Tuning ");
      
      // estimate total run time for current profile
      estimatedTotalTime = 360;
      estimatedTotalTime += (activeProfile.peakTemp - actualTemperature)/(float)activeProfile.rampUpRate;
      estimatedTotalTime += (activeProfile.peakTemp - actualTemperature)/(float)activeProfile.rampDownRate;
      estimatedTotalTime *= 1.1; // add some spare
      maxTemp = activeProfile.peakTemp + 20;

    } else {
      tft.print("Profile ");
      tft.print(activeProfileId);

      maxTemp = activeProfile.peakTemp;

      //tmp = h / (activeProfile.peakTemp * TEMPERATURE_WINDOW) * 100.0;
      //pxPerC = tmp;
      
      estimatedTotalTime = 0; // 60 * 12;
      // estimate total run time for current profile
      estimatedTotalTime = activeProfile.soakDuration + activeProfile.peakDuration;
      estimatedTotalTime += (activeProfile.peakTemp - actualTemperature)/(float)activeProfile.rampUpRate;
      estimatedTotalTime += (activeProfile.peakTemp - actualTemperature)/(float)activeProfile.rampDownRate;
      estimatedTotalTime *= 1.1; // add some spare

      //tmp = w / estimatedTotalTime; 
      //pxPerSec = (float)tmp;

      //maxTemp = activeProfile.peakTemp * TEMPERATURE_WINDOW;
    }

      tmp = h / (maxTemp * TEMPERATURE_WINDOW) * 100.0;
      pxPerC = tmp;
      tmp = w / estimatedTotalTime; 
      pxPerSec = (float)tmp;
   
#ifdef SERIAL_VERBOSE
  Serial.print("estimatedTotalTime: ");
    Serial.println(estimatedTotalTime);
    Serial.print("pxPerSec: ");
    Serial.println(pxPerSec);
    Serial.print("Calc pxPerC/S: ");
    Serial.println(pxPerC);
    Serial.print("/");
    Serial.println(pxPerSec);
#endif // SERIAL_VERBOSE
    // 50°C grid
    uint16_t t = (uint16_t)(activeProfile.peakTemp * TEMPERATURE_WINDOW);
    tft.setTextColor(tft.Color565(0xa0, 0xa0, 0xa0));
    tft.setTextSize(1);
    for (uint16_t tg = 0; tg < t; tg += 50) {
      uint16_t l = h - (tg * pxPerC / 100) + yOffset;
      tft.drawFastHLine(0, l, tft.width(), tft.Color565(0xe0, 0xe0, 0xe0));
      tft.setCursor(tft.width() - 24, l - 7);
      alignRightPrefix((int)tg); 
      tft.print((int)tg);
      tft.print("\367");
    }
}
// ----------------------------------------------------------------------------
void updateProcessDisplay() {
  const uint8_t h = tft.height() - 42;
  const uint8_t w = tft.width() - 24;
  const uint8_t yOffset = 30; // space not available for graph  

  uint16_t dx, dy;
  uint8_t y = 2;
//  double tmp;

  // header & initial view
  tft.setTextColor(ST7735_WHITE, ST7735_BLUE);
  tft.setTextSize(1);

  if (!initialProcessDisplay) {
    drawInitialProcessDisplay();
  }

  // elapsed time
  uint16_t elapsed = (zeroCrossTicks - startCycleZeroCrossTicks) / (float)(TICKS_PER_SEC);
  tft.setCursor(tft.width() - 35, y);
  alignRightPrefix(elapsed); 
  tft.print(elapsed);
  tft.print("s");

  y += menuItemHeight + 2;

  displayThermocoupleData(1, y);
  
  tft.setTextSize(1);

  // current state
  y -= 2;
  tft.setCursor(tft.width() - 65, y);
  tft.setTextColor(ST7735_BLACK, ST7735_GREEN);
  
  switch (currentState) {
    #define casePrintState(state) case state: tft.print(#state); break;
    casePrintState(RampToSoak);
    casePrintState(Soak);
    casePrintState(RampUp);
    casePrintState(Peak);
    casePrintState(RampDown);
    casePrintState(CoolDown);
    casePrintState(Complete);
    default: tft.print((uint8_t)currentState); break;
  }
  tft.print("        "); // lazy: fill up space

  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

  // set point
  y += 10;
  tft.setCursor(tft.width() - 65, y);
  tft.print("Sp:"); 
  alignRightPrefix((int)heaterSetpoint); 
  printDouble(heaterSetpoint);
  tft.print("\367C  ");

  // draw temperature curves

  if (xOffset >= elapsed) {
    xOffset = 0;
  }

  do { // x with wrap around
    dx = (uint16_t)((elapsed - xOffset) * pxPerSec);
    if (dx > w) {
      xOffset = elapsed;
    }
  } while(dx > w);

  // temperature setpoint
  dy = h - ((uint16_t)heaterSetpoint * pxPerC / 100) + yOffset;
  tft.drawPixel(dx, dy, ST7735_BLUE);

  // actual temperature
  dy = h - ((uint16_t)actualTemperature * pxPerC / 100) + yOffset;
  tft.drawPixel(dx, dy, ST7735_RED);

  // bottom line
  y = 119;

  // set values
  tft.setCursor(1, y);
  tft.print("\xef");
  alignRightPrefix((int)heaterPower); 
  tft.print((int)heaterPower);
  tft.print('%');

  tft.print(" \x12 "); // alternative: \x7f
  printDouble(rampRate);
  tft.print("\367C/s    ");
  
}
// ----------------------------------------------------------------------------

#endif // UI_H
