#ifndef _CRYPTO_H_
#define _CRYPTO_H_
#include <stdint.h>
class Crypto {
public:
  static const char* encrypt(uint8_t* in, uint32_t inl, uint8_t* out, uint32_t *outl);
  static const char* decrypt(uint8_t* in, uint32_t inl, uint8_t* out, uint32_t *outl);
};

#endif  // _CRYPTO_H_
