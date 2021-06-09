#include "crypto.h"
#include <openssl/evp.h>
#include <openssl/aes.h>

static unsigned char ClearKey[] = { 0xe2, 0x11, 0xb8, 0x7d, 0x8b, 0xb4, 0x5a, 0xb9, 0x90,0xcc, 0x44, 0x4a, 0xb0, 0xbd, 0xc5, 0xa2 };

const char* Crypto::encrypt(uint8_t* in, uint32_t inl, uint8_t* out, uint32_t *outl) {
  int l = 0, fl = 0, il = inl;
  unsigned char iv[8];
  EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();

  if (outl) {
    *outl = 0;
  }

  if (!in || !out) {
    static char buf[100];
#ifdef _WIN32
    _snprintf(buf, sizeof(buf), "Invalid parameters(in: %p, out: %p)", in, out);
#else
    snprintf(buf, sizeof(buf), "Invalid parameters(in: %p, out: %p)", in, out);
#endif
    return buf;
  }

  EVP_CIPHER_CTX_init(ctx);

  if (EVP_EncryptInit_ex(ctx, EVP_aes_128_ecb(), NULL, ClearKey, iv) != 1) {
    EVP_CIPHER_CTX_free(ctx);
    return "EVP_EncryptInit_ex failed";
  }
  
  if (EVP_CIPHER_CTX_set_padding(ctx, 1) != 1) {
    EVP_CIPHER_CTX_free(ctx);
    return "EVP_CIPHER_CTX_set_padding failed";
  }

  if (EVP_EncryptUpdate(ctx, out, &l, in, il) != 1) {
    EVP_CIPHER_CTX_free(ctx);
    return "EVP_EncryptUpdate failed";
  }
  
  if (EVP_EncryptFinal_ex(ctx, out + l, &fl) != 1) {
    EVP_CIPHER_CTX_free(ctx);
    return "EVP_EncryptFinal_ex failed";
  }

  if (outl) {
    *outl = l + fl;
  }

  EVP_CIPHER_CTX_free(ctx);

  return "";
}

const char* Crypto::decrypt(uint8_t* in, uint32_t inl, uint8_t* out, uint32_t *outl) {
  int l = 0, fl = 0, il = inl;
  unsigned char iv[8];
  EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();

  if (outl) {
    *outl = 0;
  }

  if (!in || !out) {
    static char buf[100];
#ifdef _WIN32
    _snprintf(buf, sizeof(buf), "Invalid parameters(in: %p, out: %p)", in, out);
#else
    snprintf(buf, sizeof(buf), "Invalid parameters(in: %p, out: %p)", in, out);
#endif
    return buf;
  }

  EVP_CIPHER_CTX_init(ctx);

  if (EVP_DecryptInit_ex(ctx, EVP_aes_128_ecb(), NULL, ClearKey, iv) != 1) {
    EVP_CIPHER_CTX_free(ctx);
    return "EVP_EncryptInit_ex failed";
  }
  
  if (EVP_CIPHER_CTX_set_padding(ctx, 1) != 1) {
    EVP_CIPHER_CTX_free(ctx);
    return "EVP_CIPHER_CTX_set_padding failed";
  }

  if (EVP_DecryptUpdate(ctx, out, &l, in, il) != 1) {
    EVP_CIPHER_CTX_free(ctx);
    return "EVP_EncryptUpdate failed";
  }

  if (EVP_DecryptFinal_ex(ctx, out + l, &fl) != 1) {
    EVP_CIPHER_CTX_free(ctx);
    return "EVP_EncryptFinal_ex failed";
  }

  if (outl) {
    *outl = l + fl;
  }

  EVP_CIPHER_CTX_free(ctx);

  return "";
}

