#ifndef HELPERS_H
#define HELPERS_H

// ----------------------------------------------------------------------------

#include <Arduino.h>

// ----------------------------------------------------------------------------
//

class ScopedTimer {
public:
  ScopedTimer(const char * Label)
    : label(Label), ts(millis())
  {
  }
  ~ScopedTimer() {
    Serial.print(label); Serial.print(": ");
    Serial.println(millis() - ts);
  }
private:
  const char *label;
  const unsigned long ts;
};

// ----------------------------------------------------------------------------

template<typename T> inline const T labs(T const& x) {
    return (x < 0) ? -x : x;
}

// ----------------------------------------------------------------------------

long lpow(int base, int exponent) {
  long result = 1;
  while (exponent) {
    if (exponent & 1) {
      result *= base;
    }
    exponent >>= 1;
    base *= base;
  }
  return result;
}

// ----------------------------------------------------------------------------

void itoa10(int32_t n, char *result, bool preventMinus = false) {
  uint32_t u;
  uint16_t i = 0;

  if (n < 0) { // for negative number, prepend '-' and invert
    if (!preventMinus) {
      result[0] = '-';
      result++;    
    }
    u = ((uint32_t) -(n + 1)) + 1;
  }
  else { 
    u = (uint32_t)n;
  }
  
  do {
    result[i++] = '0' + u % 10;
    u /= 10;
  } 
  while (u > 0);
  
  // rotate string bytewise
  for (uint16_t j = 0; j < i / 2; ++j) {
    char tmp = result[j];
    result[j] = result[i - j - 1];
    result[i - j - 1] = tmp;
  }
  result[i] = '\0';
}

// ----------------------------------------------------------------------------

uint8_t countDigits(uint32_t n) {
  uint8_t d = 1;
  switch (n) {
    case  100000000 ... 999999999:  d++;
    case   10000000 ... 99999999:   d++;
    case    1000000 ... 9999999:    d++;
    case     100000 ... 999999:     d++;
    case      10000 ... 99999:      d++;
    case       1000 ... 9999:       d++;
    case        100 ... 999:        d++;
    case         10 ... 99:         d++;
  }
  return d;
}

// ----------------------------------------------------------------------------

void ftoa(char *buf, float val, int places) {
  if (signbit(val)) *buf++ = '-';

  int32_t digit = (int32_t)(val);
  const int32_t precision = lpow(10, places);
  int32_t decimal = lround((val - digit) * precision);

  if (labs(decimal) == precision) {
    signbit(val) ? digit-- : digit++;
    decimal = 0;
  }

  itoa10(labs(digit), buf);
  while (*buf != '\0') buf++;
  *buf++ = '.';

  if (decimal) {
    int32_t tmp = labs(decimal) * 10;
    while (tmp < precision) {
      *buf++ = '0';
      tmp *= 10;
    }
  }

  itoa10(labs(decimal), buf);
  if (decimal == 0) {
    while (places--) *buf++ = '0';
    *buf = '\0';
  }
}

// ----------------------------------------------------------------------------

void itostr(char *r, int16_t val, char *unit = NULL) {
  char *p = r, *u = unit;
  itoa10(val, p);
  while(*p != 0x00) p++;
  while(*u != 0x00) *p++ = *u++;
  *p = '\0';
}

#endif // HELPERS_H
