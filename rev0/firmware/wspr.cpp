
// ============================================================================
//
// wspr.cpp  - WSPR encoding library
//
// Original WSPR code by NT7S - Jason Milldrum
// Modifed for Type2 and Type3 messages by SM7PNV Harry Zachrisson
//
// ============================================================================

#include <stdint.h>
#include <Arduino.h>
#include "wspr.h"

WSPR::WSPR() {
}

S_WSPRData WSPRData;

// Public functions

// wspr encoding
void WSPR::encode(const char* call, const char* loc, const uint8_t dbm, uint8_t* symbols, uint8_t WSPRMessageType) {
  char call_[7];
  char loc_[5];
  uint8_t dbm_ = dbm;
  strcpy(call_, call);
  strcpy(loc_, loc);
  uint32_t n, m;
  // run message checks
  message_prep(call_, loc_, dbm_);
  // bit packing
  uint8_t c[11];
  switch (WSPRMessageType) {
    case 1:
      // normal coding with callsign, 4-letter Maidenhead postion and power
      n = code(callsign[0]);
      n = n * 36 + code(callsign[1]);
      n = n * 10 + code(callsign[2]);
      n = n * 27 + (code(callsign[3]) - 10);
      n = n * 27 + (code(callsign[4]) - 10);
      n = n * 27 + (code(callsign[5]) - 10);

      m = ((179 - 10 * (locator[0] - 'A') - (locator[2] - '0')) * 180) +
          (10 * (locator[1] - 'A')) + (locator[3] - '0');
      m = (m * 128) + power + 64;
      break;
    case 2:
      // call sign and Prefix or suffix for it and power, no Maidenhead position
      n = code(callsign[0]);
      n = n * 36 + code(callsign[1]);
      n = n * 10 + code(callsign[2]);
      n = n * 27 + (code(callsign[3]) - 10);
      n = n * 27 + (code(callsign[4]) - 10);
      n = n * 27 + (code(callsign[5]) - 10);

      if (WSPRData.SufixPrefix) {
        // single number or letter or double number suffix
        m = (27232 + WSPRData.Sufix);
        m = (m * 128) + power + 2 + 64;
      } else {
        // three character prefix
        m =          encodeChar(WSPRData.Prefix[0]); //left Character
        m = 37 * m + encodeChar(WSPRData.Prefix[1]); //mid character
        m = 37 * m + encodeChar(WSPRData.Prefix[2]); //right character
        if (m > 32767) {
          m = m - 32768;
          m = (m * 128) + power + 66;
        } else {
          m = (m * 128) + power + 65;
        }
      }
      break;
    case 3:
      // hashed Callsign, 6-letter maidenhead position and power
      // encode the 6-letter Maidenhear postion in to n that is usually used
      // for callsign coding, reshuffle the character order to conform to the callsign rules
      n = code(WSPRData.MaidenHead6[1]);
      n = n * 36 + code(WSPRData.MaidenHead6[2]);
      n = n * 10 + code(WSPRData.MaidenHead6[3]);
      n = n * 27 + (code(WSPRData.MaidenHead6[4]) - 10);
      n = n * 27 + (code(WSPRData.MaidenHead6[5]) - 10);
      n = n * 27 + (code(WSPRData.MaidenHead6[0]) - 10);
      m = 128 * CallSignHash(call) - power - 1 + 64;
      break;
  }
  // callsign is 28 bits, locator/power is 22 bits.
  // a little less work to start with the least-significant bits
  c[3] = (uint8_t)((n & 0x0f) << 4);
  n = n >> 4;
  c[2] = (uint8_t)(n & 0xff);
  n = n >> 8;
  c[1] = (uint8_t)(n & 0xff);
  n = n >> 8;
  c[0] = (uint8_t)(n & 0xff);

  c[6] = (uint8_t)((m & 0x03) << 6);
  m = m >> 2;
  c[5] = (uint8_t)(m & 0xff);
  m = m >> 8;
  c[4] = (uint8_t)(m & 0xff);
  m = m >> 8;
  c[3] |= (uint8_t)(m & 0x0f);
  c[7] = 0;
  c[8] = 0;
  c[9] = 0;
  c[10] = 0;
  // convolutional encoding
  uint8_t s[WSPR_SYMBOL_COUNT];
  convolve(c, s, 11, WSPR_SYMBOL_COUNT);
  // interleaving
  interleave(s);
  // merge with sync vector
  merge_sync_vector(s, symbols);
}

// run message checks
void WSPR::message_prep(char* call, char* loc, uint8_t dbm) {
  // if only the 2nd character is a digit, then pad with a space.
  // if this happens, then the callsign will be truncated if it is
  // longer than 6 characters.
  if (isdigit(call[1]) && isupper(call[2])) {
    call[5] = call[4];
    call[4] = call[3];
    call[3] = call[2];
    call[2] = call[1];
    call[1] = call[0];
    call[0] = ' ';
  }
  // ensure that the only allowed characters are digits and uppercase letters
  uint8_t i;
  for (i = 0; i < 6; i++) {
    call[i] = toupper(call[i]);
    if (!(isdigit(call[i]) || isupper(call[i]))) {
      call[i] = ' ';
      if (i == 4) call[5] = ' ';
    }
  }
  memcpy(callsign, call, 6);
  // grid locator validation
  for (i = 0; i < 4; i++) {
    loc[i] = toupper(loc[i]);
    if (!(isdigit(loc[i]) || (loc[i] >= 'A' && loc[i] <= 'R'))) {
      memcpy(loc, "AA00", 5);    //loc = "AA00";
    }
  }
  memcpy(locator, loc, 4);
  power = dbm;
}

void WSPR::convolve(uint8_t* c, uint8_t* s, uint8_t message_size, uint8_t bit_size) {
  uint32_t reg_0 = 0;
  uint32_t reg_1 = 0;
  uint32_t reg_temp = 0;
  uint8_t input_bit, parity_bit;
  uint8_t bit_count = 0;
  uint8_t i, j, k;
  for (i = 0; i < message_size; i++) {
    for (j = 0; j < 8; j++) {
      // set input bit according the MSB of current element
      input_bit = (((c[i] << j) & 0x80) == 0x80) ? 1 : 0;
      // shift both registers and put in the new input bit
      reg_0 = reg_0 << 1;
      reg_1 = reg_1 << 1;
      reg_0 |= (uint32_t)input_bit;
      reg_1 |= (uint32_t)input_bit;
      // and Register 0 with feedback taps, calculate parity
      reg_temp = reg_0 & 0xf2d05351;
      parity_bit = 0;
      for (k = 0; k < 32; k++) {
        parity_bit = parity_bit ^ (reg_temp & 0x01);
        reg_temp = reg_temp >> 1;
      }
      s[bit_count] = parity_bit;
      bit_count++;
      // and Register 1 with feedback taps, calculate parity
      reg_temp = reg_1 & 0xe4613c47;
      parity_bit = 0;
      for (k = 0; k < 32; k++) {
        parity_bit = parity_bit ^ (reg_temp & 0x01);
        reg_temp = reg_temp >> 1;
      }
      s[bit_count] = parity_bit;
      bit_count++;
      if (bit_count >= bit_size) {
        break;
      }
    }
  }
}

void WSPR::interleave(uint8_t* s) {
  uint8_t d[WSPR_SYMBOL_COUNT];
  uint8_t rev, index_temp, i, j, k;
  i = 0;
  for (j = 0; j < 255; j++) {
    // bit reverse the index
    index_temp = j;
    rev = 0;
    for (k = 0; k < 8; k++) {
      if (index_temp & 0x01) {
        rev = rev | (1 << (7 - k));
      }
      index_temp = index_temp >> 1;
    }
    if (rev < WSPR_SYMBOL_COUNT) {
      d[rev] = s[i];
      i++;
    }
    if (i >= WSPR_SYMBOL_COUNT) {
      break;
    }
  }
  memcpy(s, d, WSPR_SYMBOL_COUNT);
}

// Private functions

void WSPR::merge_sync_vector(uint8_t* g, uint8_t* symbols) {
  uint8_t i;
  const uint8_t sync_vector[WSPR_SYMBOL_COUNT] =
  { 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0,
    1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1,
    0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0,
    1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1,
    0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1,
    1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0
  };
  for (i = 0; i < WSPR_SYMBOL_COUNT; i++) {
    symbols[i] = sync_vector[i] + (2 * g[i]);
  }
}

// type 3 call sign hash by RFZero www.rfzero.net modified by SM7PNV
uint32_t WSPR::CallSignHash(const char* call) {
#define rot(x, k) ((x << k) | (x >> (32 - k)))
  uint32_t a, b, c;
  char CallWithSuPrefix [11];
  uint8_t Length = strlen(call);
  uint8_t TenDigit = 0;
  uint8_t Number;
  uint8_t CharLoop;
  Serial.print("Length ");
  Serial.print(Length);
  strcpy(CallWithSuPrefix, call);
  if (WSPRData.SufixPrefix) {
    CallWithSuPrefix[Length] = '/';
    if (WSPRData.Sufix < 36) {
      CallWithSuPrefix[Length + 2] = 0;
      if (WSPRData.Sufix < 10) {
        CallWithSuPrefix[Length + 1] = '0' + WSPRData.Sufix;
      } else {
        CallWithSuPrefix[Length + 1] = 'A' + (WSPRData.Sufix - 10);
      }
    } else  {
      // suffix is double digits
      Number=WSPRData.Sufix-36;
      while (Number>9) {
        ++TenDigit;
        Number -= 10;
      }
      CallWithSuPrefix[Length+1] = '0'+TenDigit;
      CallWithSuPrefix[Length+2] = '0'+Number;
      CallWithSuPrefix[Length+3] = 0;
    }
  } else  if (!WSPRData.SufixPrefix) {
    CallWithSuPrefix[0] = WSPRData.Prefix[0];
    CallWithSuPrefix[1] = WSPRData.Prefix[1];
    CallWithSuPrefix[2] = WSPRData.Prefix[2];
    CallWithSuPrefix[3] = '/';

    for (CharLoop = 0; CharLoop < Length; CharLoop++) {
      CallWithSuPrefix[CharLoop + 4] = call[CharLoop];
    }
  }
  Length = strlen(CallWithSuPrefix);
  a = b = c = 0xdeadbeef + Length + 146;

  const uint32_t *k = (const uint32_t *)CallWithSuPrefix;

  switch (Length) {
    // length 3-10 chars, thus 0, 1, 2, 11 and 12 omitted
    case 10: c += k[2] & 0xffff; b += k[1]; a += k[0]; break;
    case 9:  c += k[2] & 0xff; b += k[1]; a += k[0]; break;
    case 8:  b += k[1]; a += k[0]; break;
    case 7:  b += k[1] & 0xffffff; a += k[0]; break;
    case 6:  b += k[1] & 0xffff; a += k[0]; break;
    case 5:  b += k[1] & 0xff; a += k[0]; break;
    case 4:  a += k[0]; break;
    case 3:  a += k[0] & 0xffffff; break;
  }
  c ^= b; c -= rot(b, 14);
  a ^= c; a -= rot(c, 11);
  b ^= a; b -= rot(a, 25);
  c ^= b; c -= rot(b, 16);
  a ^= c; a -= rot(c, 4);
  b ^= a; b -= rot(a, 14);
  c ^= b; c -= rot(b, 24);
  c &= 0xFFFF;
  return c;
}

// converts a letter or digit to WSPR message format
uint8_t WSPR::encodeChar (char Character) {
  uint8_t ConvertedNumber;
  if (Character == ' ') {
    ConvertedNumber = 36;
  }
  else {
    if (isdigit(Character)) {
      ConvertedNumber = Character - '0';
    } else {
      ConvertedNumber = 10 + (Character - 'A') ;
    }
  }
  return ConvertedNumber;
}

uint8_t WSPR::code(char c) {
  // validate the input then return the proper integer code.
  // return 255 as an error code if the char is not allowed.
  if (isdigit(c)) {
    return (uint8_t)(c - 48);
  } else if (c == ' ') {
    return 36;
  } else if (c >= 'A' && c <= 'Z') {
    return (uint8_t)(c - 55);
  } else {
    return 255;
  }
}

