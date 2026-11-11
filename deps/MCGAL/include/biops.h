#if __GNUC__ < 4 || (__GNUC__ == 4 && __GNUC_MINOR__ < 1)
/* Technically wrong, but this avoids compilation errors on some gcc
   versions. */
#    define BITOP_ADDR(x) "=m"(*(volatile long*)(x))
#else
#    define BITOP_ADDR(x) "+m"(*(volatile long*)(x))
#endif

#define ADDR BITOP_ADDR(addr)

static inline void set_bit(int nr, volatile unsigned long* addr) {
    asm volatile("bts %1,%0" : ADDR : "Ir"(nr) : "memory");
}

static inline int test_bit(int nr, volatile const unsigned long* addr) {
    int oldbit;

    asm volatile("bt %2,%1\n\t"
                 "sbb %0,%0"
                 : "=r"(oldbit)
                 : "m"(*(unsigned long*)addr), "Ir"(nr));

    return oldbit;
}

static inline void setBit(char* bitmap, int bitIndex) {
    int byteIndex = bitIndex / 8;
    char bitMask = 1 << (bitIndex % 8);
    bitmap[byteIndex] |= bitMask;
}

static inline void clearBit(char* bitmap, int bitIndex) {
    int byteIndex = bitIndex / 8;
    char bitMask = ~(1 << (bitIndex % 8));
    bitmap[byteIndex] &= bitMask;
}

static inline void setBit(unsigned char* bitmap, int bitIndex) {
    int byteIndex = bitIndex / 8;
    unsigned char bitMask = 1 << (bitIndex % 8);
    bitmap[byteIndex] |= bitMask;
}

static inline void clearBit(unsigned char* bitmap, int bitIndex) {
    int byteIndex = bitIndex / 8;
    unsigned char bitMask = ~(1 << (bitIndex % 8));
    bitmap[byteIndex] &= bitMask;
}

static inline bool getBit(const char* bitmap, int bitIndex) {
    int byteIndex = bitIndex / 8;
    char bitMask = 1 << (bitIndex % 8);
    return (bitmap[byteIndex] & bitMask) != 0;
}

static inline bool getBit(const unsigned char* bitmap, int bitIndex) {
    int byteIndex = bitIndex / 8;
    unsigned char bitMask = 1 << (bitIndex % 8);
    return (bitmap[byteIndex] & bitMask) != 0;
}