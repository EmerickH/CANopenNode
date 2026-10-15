/*
 * CAN driver for host unit tests: in-memory CAN (FD) bus.
 */

#include <string.h>

#include "301/CO_driver.h"

test_frame_t test_txLog[TEST_TX_LOG_SIZE];
int test_txCount = 0;

void
test_txClear(void) {
    test_txCount = 0;
    (void)memset(test_txLog, 0, sizeof(test_txLog));
}

int
test_CANrx(CO_CANmodule_t* CANmodule, uint16_t ident, const uint8_t* data, uint8_t len) {
    CO_CANrxMsg_t msg;
    int matches = 0;

    (void)memset(&msg, 0, sizeof(msg));
    msg.ident = ident;
    msg.DLC = len;
    (void)memcpy(msg.data, data, len);

    for (uint16_t i = 0; i < CANmodule->rxSize; i++) {
        CO_CANrx_t* buffer = &CANmodule->rxArray[i];
        if ((buffer->CANrx_callback != NULL) && (((ident ^ buffer->ident) & buffer->mask) == 0U)) {
            buffer->CANrx_callback(buffer->object, (void*)&msg);
            matches++;
        }
    }
    return matches;
}

void
CO_CANsetConfigurationMode(void* CANptr) {
    (void)CANptr;
}

void
CO_CANsetNormalMode(CO_CANmodule_t* CANmodule) {
    CANmodule->CANnormal = true;
}

#if ((CO_CONFIG_FD)&CO_CONFIG_FD_ENABLE) != 0
CO_ReturnError_t
CO_CANsetDataBitRate(CO_CANmodule_t* CANmodule, uint16_t dataBitRate) {
    if (CANmodule == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    CANmodule->dataBitRate = dataBitRate;
    return CO_ERROR_NO;
}
#endif

CO_ReturnError_t
CO_CANmodule_init(CO_CANmodule_t* CANmodule, void* CANptr, CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[],
                  uint16_t txSize, uint16_t CANbitRate) {
    (void)CANbitRate;
    if ((CANmodule == NULL) || (rxArray == NULL) || (txArray == NULL)) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    (void)memset(CANmodule, 0, sizeof(*CANmodule));
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    (void)memset(rxArray, 0, sizeof(CO_CANrx_t) * rxSize);
    (void)memset(txArray, 0, sizeof(CO_CANtx_t) * txSize);
    return CO_ERROR_NO;
}

void
CO_CANmodule_disable(CO_CANmodule_t* CANmodule) {
    (void)CANmodule;
}

CO_ReturnError_t
CO_CANrxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, uint16_t mask, bool_t rtr, void* object,
                   void (*CANrx_callback)(void* object, void* message)) {
    if ((CANmodule == NULL) || (object == NULL) || (CANrx_callback == NULL) || (index >= CANmodule->rxSize) || rtr) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    CO_CANrx_t* buffer = &CANmodule->rxArray[index];
    buffer->object = object;
    buffer->CANrx_callback = CANrx_callback;
    buffer->ident = ident & 0x07FFU;
    buffer->mask = mask & 0x07FFU;
    return CO_ERROR_NO;
}

CO_CANtx_t*
CO_CANtxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes,
                   bool_t syncFlag) {
    if ((CANmodule == NULL) || (index >= CANmodule->txSize) || rtr || (noOfBytes > CO_CAN_MAX_DLEN)) {
        return NULL;
    }
    CO_CANtx_t* buffer = &CANmodule->txArray[index];
    buffer->ident = (uint32_t)ident & 0x07FFU;
    buffer->DLC = noOfBytes;
    buffer->bufferFull = false;
    buffer->syncFlag = syncFlag;
    return buffer;
}

CO_ReturnError_t
CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {
    (void)CANmodule;
    if (test_txCount >= TEST_TX_LOG_SIZE) {
        return CO_ERROR_TX_OVERFLOW;
    }
    test_frame_t* f = &test_txLog[test_txCount++];
    f->ident = (uint16_t)buffer->ident;
    f->len = buffer->DLC;
    (void)memcpy(f->data, buffer->data, buffer->DLC);
    return CO_ERROR_NO;
}

void
CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule) {
    (void)CANmodule;
}

void
CO_CANmodule_process(CO_CANmodule_t* CANmodule) {
    (void)CANmodule;
}
