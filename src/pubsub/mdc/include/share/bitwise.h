#ifndef BITWISE_H
#define BITWISE_H

#define CHECK_BIT(a, b) ((a & (1 << b)) > 0)
#define SET_BIT(a, b) do { a |= (1 << b); } while (0)
#define CLEAR_BIT(a, b) do { a &= ~(1 << b); } while (0)

#define CONFIG_BIT(a, b, c) do { \
    if (c) \
        a |= (1 << b); \
    else \
        a &= ~(1 << b); \
} while (0)

#endif