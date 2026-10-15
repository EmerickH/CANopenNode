/*
 * CAN driver for host unit tests: in-memory CAN (FD) bus, no hardware.
 *
 * Transmitted frames are stored in a log, received frames are injected by the test with test_CANrx().
 */

#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef CO_DRIVER_CUSTOM
#include "CO_driver_custom.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CO_CONFIG_FD) && (((CO_CONFIG_FD)&CO_CONFIG_FD_ENABLE) != 0)
#define CO_CAN_MAX_DLEN 64U
#else
#define CO_CAN_MAX_DLEN 8U
#endif

#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x
typedef uint_fast8_t bool_t;
typedef float float32_t;
typedef double float64_t;

/* Received frame, as passed to CANrx_callback */
typedef struct {
    uint16_t ident;
    uint8_t DLC;
    uint8_t data[CO_CAN_MAX_DLEN];
} CO_CANrxMsg_t;

#define CO_CANrxMsg_readIdent(msg) (((CO_CANrxMsg_t*)(msg))->ident)
#define CO_CANrxMsg_readDLC(msg)   (((CO_CANrxMsg_t*)(msg))->DLC)
#define CO_CANrxMsg_readData(msg)  ((const uint8_t*)(((CO_CANrxMsg_t*)(msg))->data))

typedef struct {
    uint16_t ident;
    uint16_t mask;
    void* object;
    void (*CANrx_callback)(void* object, void* message);
} CO_CANrx_t;

typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[CO_CAN_MAX_DLEN];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
} CO_CANtx_t;

typedef struct {
    void* CANptr;
    CO_CANrx_t* rxArray;
    uint16_t rxSize;
    CO_CANtx_t* txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t bufferInhibitFlag;
    volatile bool_t firstCANtxMessage;
    volatile uint16_t CANtxCount;
    uint32_t errOld;
    uint16_t dataBitRate;
} CO_CANmodule_t;

typedef struct {
    void* addr;
    size_t len;
    uint8_t subIndexOD;
    uint8_t attr;
    void* addrNV;
} CO_storage_entry_t;

#define CO_LOCK_CAN_SEND(CAN_MODULE)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)
#define CO_LOCK_EMCY(CAN_MODULE)
#define CO_UNLOCK_EMCY(CAN_MODULE)
#define CO_LOCK_OD(CAN_MODULE)
#define CO_UNLOCK_OD(CAN_MODULE)

#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)                                                                                             \
    {                                                                                                                  \
        CO_MemoryBarrier();                                                                                            \
        rxNew = (void*)1L;                                                                                             \
    }
#define CO_FLAG_CLEAR(rxNew)                                                                                           \
    {                                                                                                                  \
        CO_MemoryBarrier();                                                                                            \
        rxNew = NULL;                                                                                                  \
    }

/* Test helpers, implemented in CO_driver_test.c */
#define TEST_TX_LOG_SIZE 64

typedef struct {
    uint16_t ident;
    uint8_t len;
    uint8_t data[CO_CAN_MAX_DLEN];
} test_frame_t;

extern test_frame_t test_txLog[TEST_TX_LOG_SIZE];
extern int test_txCount;

/* Deliver a frame to all receive buffers of the module, which match the identifier. Returns number of matches. */
int test_CANrx(CO_CANmodule_t* CANmodule, uint16_t ident, const uint8_t* data, uint8_t len);
void test_txClear(void);

#ifdef __cplusplus
}
#endif

#endif /* CO_DRIVER_TARGET_H */
