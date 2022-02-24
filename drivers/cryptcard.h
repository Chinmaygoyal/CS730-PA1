#ifndef __CRYPT_CARD_H_
#define __CRYPT_CARD_H_

#define DEVNAME "cryptcard"
#define CHAR_DEVNAME "crypt_chardev"
#define CRYPT_A 0xa
#define CRYPT_B 0xb
#define MMIO_LENGTH 0xc
#define MMIO_STATUS 0x20
#define ISR 0x24
#define MMIO_ADDRESS 0x80
#define MMIO_DATA 0xa8
#define DMA_DATA 0xa8

#define DMA_LENGTH 0x98
#define DMA_ADDRESS 0x90
#define DMA_STATUS 0xa0

#define KB 1024
#define MB 1024*1024
#define DECRYPT_BIT 1
#define MMIO_INTERRUPT 7
#define DMA_INTERRUPT 2


#define MMIO_LIMIT (1024*1024 - 256)
#define DMA_LIMIT (32*1024)


void set_a_b(uint8_t a, uint8_t b);


/*
operation = 1 -> encrypt
          = 2 -> decrypt
          = 3 -> set key
          = 4 -> set config
*/
struct crypt_data{
    void* address;
    uint64_t length;
    uint32_t operation;
    uint8_t a;
    uint8_t b;
    int interrupt;
    int mmio;
};

struct file_pvt{
    uint8_t a;
    uint8_t b;
    int interrupt;
    int mmio;
};

#endif
