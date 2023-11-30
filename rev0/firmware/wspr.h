
// ============================================================================
//
// wspr.h  - WSPR encoding library
//
// Original WSPR code by NT7S - Jason Milldrum
// Modifed for Type2 and Type3 messages by SM7PNV Harry Zachrisson
//
// ============================================================================

#include <Arduino.h>
#include <inttypes.h>

#ifndef WSPR_H
#define WSPR_H

#define WSPR_SYMBOL_COUNT     162

// data needed to transmit a WSPR packet
typedef struct S_WSPRData {
  char CallSign[7];         // call sign
  char Prefix[4];           // prefix three chars max
  uint8_t Sufix;            // sufix code in WSPR format
  uint8_t SufixPrefix;      // use sufix or prefix for type-3 data
  uint8_t LocPrecision;     // determines if a second xmit will be sent
  char MaidenHead4[5];      // 4-char maidenhead locator
  char MaidenHead6[7];      // 6-char maidenhead locator
  uint8_t TXPowerdBm;       // power data in dBm min=0 max=60
  uint8_t TimeSlotCode;     // determine the xmit time slot
};

class WSPR {
  public:

    char callsign[7];
    char locator[5];
    uint8_t power;

    WSPR();
    void encode(const char*, const char*, const uint8_t, uint8_t*, uint8_t);
    void message_prep(char*, char*, uint8_t);
    void convolve(uint8_t*, uint8_t*, uint8_t, uint8_t);
    void interleave(uint8_t*);
    boolean geoFence ();

  private:
    void merge_sync_vector(uint8_t*, uint8_t*);
    uint32_t CallSignHash(const char *);
    uint8_t encodeChar (char);
    uint8_t code(char);

};

#endif
