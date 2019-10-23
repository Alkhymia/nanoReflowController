#ifndef EEPROM_HELPERS_H
#define EEPROM_HELPERS_H

#include <avr/eeprom.h>
#include <EEPROM.h>

// EEPROM offsets
const uint16_t offsetProfileNum = maxProfiles * sizeof(Profile_t) + 2; // one byte
const uint16_t offsetPidConfig  = maxProfiles * sizeof(Profile_t) + 3; // sizeof(PID_t)

bool savePID() {
  do {} while (!(eeprom_is_ready()));
  eeprom_write_block(&heaterPID, (void *)offsetPidConfig, sizeof(PID_t));
  return true;
}

bool loadPID() {
  do {} while (!(eeprom_is_ready()));
  eeprom_read_block(&heaterPID, (void *)offsetPidConfig, sizeof(PID_t));
  return true;  
}

void saveLastUsedProfile() {
  EEPROM.write(offsetProfileNum, (uint8_t)activeProfileId & 0xff);
}

bool loadParameters(uint8_t profile) {
  uint16_t offset = profile * sizeof(Profile_t);

  do {} while (!(eeprom_is_ready()));
  eeprom_read_block(&activeProfile, (void *)offset, sizeof(Profile_t));

  return true;  
}

void loadLastUsedProfile() {
  activeProfileId = EEPROM.read(offsetProfileNum) & 0xff;
  loadParameters(activeProfileId);
}

bool saveParameters(uint8_t profile) {
  uint16_t offset = profile * sizeof(Profile_t);

  do {} while (!(eeprom_is_ready()));
  eeprom_write_block(&activeProfile, (void *)offset, sizeof(Profile_t));
  return true;
}

#endif // EEPROM_HELPERS_H
