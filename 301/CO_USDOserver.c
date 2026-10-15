/*
 * CANopen FD Universal Service Data Object - server protocol.
 *
 * @file        CO_USDOserver.c
 * @ingroup     CO_USDOserver
 * @author      CANopenNode contributors
 * @copyright   2026 CANopenNode contributors
 *
 * This file is part of <https://github.com/CANopenNode/CANopenNode>, a CANopen Stack.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
 * an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 */

#include <string.h>

#include "301/CO_USDOserver.h"

#if ((CO_CONFIG_FD)&CO_CONFIG_FD_ENABLE) != 0

/* CANopen data types (CiA 301 / CiA 1301), used in the data type field */
#define CO_USDO_DT_UNSIGNED8      0x05U
#define CO_USDO_DT_UNSIGNED16     0x06U
#define CO_USDO_DT_UNSIGNED32     0x07U
#define CO_USDO_DT_VISIBLE_STRING 0x09U
#define CO_USDO_DT_DOMAIN         0x0FU
#define CO_USDO_DT_UNSIGNED24     0x16U
#define CO_USDO_DT_UNSIGNED64     0x1BU

#ifdef CO_BIG_ENDIAN
static void
reverseBytes(void* start, OD_size_t size) {
    uint8_t* lo = (uint8_t*)start;
    uint8_t* hi = (uint8_t*)start + size - 1;
    uint8_t swap;
    while (lo < hi) {
        swap = *lo;
        *lo++ = *hi;
        *hi-- = swap;
    }
}
#endif

/* Is this server addressed by destination address */
static bool_t
isAddressed(const CO_USDOserver_t* USDO, uint8_t da) {
    bool_t ret;
    if ((da == CO_USDO_DA_BROADCAST) || (da == USDO->nodeId)) {
        ret = true;
    } else if (da >= CO_USDO_DA_MULTICAST) {
        uint8_t group = da - CO_USDO_DA_MULTICAST;
        ret = (USDO->multicast[group >> 5] & ((uint32_t)1U << (group & 0x1FU))) != 0U;
    } else {
        ret = false;
    }
    return ret;
}

/*
 * Read received frame from CAN module.
 *
 * Function will be called (by CAN receive interrupt) every time, when CAN frame with correct identifier
 * will be received. For more information and description of parameters see file CO_driver.h.
 */
static void
CO_USDO_receive(void* object, void* msg) {
    CO_USDOserver_t* USDO = (CO_USDOserver_t*)object;
    uint8_t DLC = CO_CANrxMsg_readDLC(msg);
    const uint8_t* data = CO_CANrxMsg_readData(msg);
    uint8_t clientId = (uint8_t)(CO_CANrxMsg_readIdent(msg) & 0x7FU);

    if (!USDO->valid || (clientId == 0U) || (DLC < CO_USDO_LEN_SEG_CONFIRM) || (DLC > CO_USDO_MAX_DLEN)
        || !isAddressed(USDO, data[CO_USDO_POS_DA])) {
        return;
    }

    bool_t sameSession = (clientId == USDO->clientId) && (data[CO_USDO_POS_SID] == USDO->sessionId);

    if (data[CO_USDO_POS_CS] == (uint8_t)CO_USDO_CS_ABORT) {
        /* abort from client of the current session, just make idle */
        if (sameSession && (USDO->state != CO_USDO_ST_IDLE)) {
            USDO->state = CO_USDO_ST_IDLE;
            CO_FLAG_CLEAR(USDO->CANrxNew);
        }
        return;
    }

    /* Accept new request only if idle, otherwise only frames of the current session. Previous frame must be
     * processed first. */
    if (CO_FLAG_READ(USDO->CANrxNew)
        || ((USDO->state != CO_USDO_ST_IDLE) && !((USDO->state == CO_USDO_ST_DOWNLOAD_SEG) && sameSession)
            && !((USDO->state == CO_USDO_ST_UPLOAD_SEG) && sameSession))) {
        return;
    }

    (void)memcpy(USDO->CANrxData, data, DLC);
    USDO->CANrxLen = DLC;
    USDO->CANrxClient = clientId;
    CO_FLAG_SET(USDO->CANrxNew);

#if ((CO_CONFIG_USDO_SRV)&CO_CONFIG_FLAG_CALLBACK_PRE) != 0
    /* Optional signal to RTOS, which can resume task, which handles USDO server. */
    if (USDO->pFunctSignalPre != NULL) {
        USDO->pFunctSignalPre(USDO->functSignalObjectPre);
    }
#endif
}

CO_ReturnError_t
CO_USDOserver_init(CO_USDOserver_t* USDO, OD_t* OD, uint8_t nodeId, uint16_t timeoutTime_ms, CO_CANmodule_t* CANdevRx,
                   uint16_t CANdevRxIdx, CO_CANmodule_t* CANdevTx, uint16_t CANdevTxIdx, uint32_t* errInfo) {
    (void)errInfo;

    /* verify arguments */
    if ((USDO == NULL) || (OD == NULL) || (CANdevRx == NULL) || (CANdevTx == NULL)) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    (void)memset(USDO, 0, sizeof(CO_USDOserver_t));
    USDO->OD = OD;
    USDO->nodeId = nodeId;
    USDO->valid = (nodeId >= 1U) && (nodeId <= 127U);
    USDO->CANdevTx = CANdevTx;
    USDO->state = CO_USDO_ST_IDLE;
    USDO->timeoutTime_us = (uint32_t)timeoutTime_ms * 1000U;
    CO_FLAG_CLEAR(USDO->CANrxNew);

    /* Requests from all USDO clients: 600h + node-ID of the client (601h to 67Fh) */
    CO_ReturnError_t ret = CO_CANrxBufferInit(CANdevRx, CANdevRxIdx, CO_CAN_ID_SDO_CLI, 0x780U, false, (void*)USDO,
                                              CO_USDO_receive);
    if (ret != CO_ERROR_NO) {
        return ret;
    }

    /* Responses: 580h + node-ID of this server. Frame length is set before each transmission. */
    uint16_t canIdTx = USDO->valid ? (uint16_t)(CO_CAN_ID_SDO_SRV + nodeId) : 0U;
    USDO->CANtxBuff = CO_CANtxBufferInit(CANdevTx, CANdevTxIdx, canIdTx, false, CO_USDO_MAX_DLEN, false);
    if (USDO->CANtxBuff == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return CO_ERROR_NO;
}

#if ((CO_CONFIG_USDO_SRV)&CO_CONFIG_FLAG_CALLBACK_PRE) != 0
void
CO_USDOserver_initCallbackPre(CO_USDOserver_t* USDO, void* object, void (*pFunctSignal)(void* object)) {
    if (USDO != NULL) {
        USDO->functSignalObjectPre = object;
        USDO->pFunctSignalPre = pFunctSignal;
    }
}
#endif

CO_ReturnError_t
CO_USDOserver_setMulticast(CO_USDOserver_t* USDO, uint8_t group, bool_t member) {
    if ((USDO == NULL) || (group < CO_USDO_DA_MULTICAST)) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    uint8_t bit = group - CO_USDO_DA_MULTICAST;
    uint32_t mask = (uint32_t)1U << (bit & 0x1FU);
    if (member) {
        USDO->multicast[bit >> 5] |= mask;
    } else {
        USDO->multicast[bit >> 5] &= ~mask;
    }
    return CO_ERROR_NO;
}

/* Data type of the OD variable, derived from its attributes and length (OD does not store data types). */
static uint8_t
dataTypeOf(const OD_stream_t* stream) {
    uint8_t dt;
    if ((stream->attribute & (OD_attr_t)ODA_STR) != 0U) {
        dt = CO_USDO_DT_VISIBLE_STRING;
    } else {
        switch (stream->dataLength) {
            case 1: dt = CO_USDO_DT_UNSIGNED8; break;
            case 2: dt = CO_USDO_DT_UNSIGNED16; break;
            case 3: dt = CO_USDO_DT_UNSIGNED24; break;
            case 4: dt = CO_USDO_DT_UNSIGNED32; break;
            case 8: dt = CO_USDO_DT_UNSIGNED64; break;
            default: dt = CO_USDO_DT_DOMAIN; break;
        }
    }
    return dt;
}

/* Prepare frame header for response: destination address, command specifier, session-ID, sub-index and index */
static void
prepareHeader(CO_USDOserver_t* USDO, uint8_t cs, bool_t withIndex) {
    (void)memset(USDO->txData, 0, sizeof(USDO->txData));
    USDO->txData[CO_USDO_POS_DA] = USDO->clientId;
    USDO->txData[CO_USDO_POS_CS] = cs;
    USDO->txData[CO_USDO_POS_SID] = USDO->sessionId;
    if (withIndex) {
        USDO->txData[CO_USDO_POS_SUBIDX] = USDO->subIndex;
        (void)CO_setUint16(&USDO->txData[CO_USDO_POS_IDX], CO_SWAP_16(USDO->index));
    }
}

/* Prepare abort frame */
static void
prepareAbort(CO_USDOserver_t* USDO, uint32_t abortCode) {
    prepareHeader(USDO, (uint8_t)CO_USDO_CS_ABORT, true);
    (void)CO_setUint32(&USDO->txData[CO_USDO_POS_ABORT], CO_SWAP_32(abortCode));
    USDO->txLen = CO_USDO_LEN_ABORT;
    USDO->abortCode = abortCode;
    USDO->state = CO_USDO_ST_ABORT;
}

/* Write data to the OD variable, handle string termination. Returns abort code. */
static uint32_t
writeToOd(CO_USDOserver_t* USDO, uint8_t* buf, OD_size_t count, bool_t lastSegment) {
    OD_size_t countWritten = 0;
    ODR_t odRet;

#ifdef CO_BIG_ENDIAN
    if (((USDO->OD_IO.stream.attribute & (OD_attr_t)ODA_MB) != 0U) && lastSegment
        && (USDO->OD_IO.stream.dataOffset == 0U)) {
        reverseBytes(buf, count);
    }
#endif

    CO_LOCK_OD(USDO->CANdevTx);
    odRet = USDO->OD_IO.write(&USDO->OD_IO.stream, buf, count, &countWritten);
    CO_UNLOCK_OD(USDO->CANdevTx);

    uint32_t abortCode = (uint32_t)CO_SDO_AB_NONE;
    if (lastSegment) {
        if (odRet == ODR_PARTIAL) {
            /* OD variable expects more data */
            abortCode = (uint32_t)CO_SDO_AB_DATA_SHORT;
        } else if (odRet != ODR_OK) {
            abortCode = OD_getSDOabCode(odRet);
        } else { /* MISRA C 2004 14.10 */
        }
    } else if (odRet != ODR_PARTIAL) {
        /* more data will come, but OD variable is already full */
        abortCode = (odRet == ODR_OK) ? (uint32_t)CO_SDO_AB_DATA_LONG : OD_getSDOabCode(odRet);
    } else { /* MISRA C 2004 14.10 */
    }
    return abortCode;
}

/*
 * Verify size of data to be downloaded against size of the OD variable and prepare stream.dataLength. For strings
 * up to two zero bytes are added to the data size to terminate the string. Returns abort code and size, which
 * will be written to OD.
 */
static uint32_t
verifyDownloadSize(CO_USDOserver_t* USDO, OD_size_t sizeInd, OD_size_t* sizeToWrite) {
    OD_size_t sizeInOd = USDO->OD_IO.stream.dataLength;
    uint32_t abortCode = (uint32_t)CO_SDO_AB_NONE;
    *sizeToWrite = sizeInd;

    if (((USDO->OD_IO.stream.attribute & (OD_attr_t)ODA_STR) != 0U) && ((sizeInOd == 0U) || (sizeInd < sizeInOd))) {
        OD_size_t delta = sizeInOd - sizeInd;
        *sizeToWrite += (delta == 1U) ? 1U : 2U;
        USDO->OD_IO.stream.dataLength = *sizeToWrite;
    } else if (sizeInOd == 0U) {
        USDO->OD_IO.stream.dataLength = sizeInd;
    } else if (sizeInd != sizeInOd) {
        abortCode = (sizeInd > sizeInOd) ? (uint32_t)CO_SDO_AB_DATA_LONG : (uint32_t)CO_SDO_AB_DATA_SHORT;
    } else { /* MISRA C 2004 14.10 */
    }
    return abortCode;
}

/* Download expedited request */
static void
downloadExpedited(CO_USDOserver_t* USDO, const uint8_t* rx, uint8_t rxLen) {
    uint8_t size = rx[CO_USDO_POS_EXP_SIZE];
    if ((size == 0U) || (size > CO_USDO_EXP_MAX_DATA) || (rxLen < (CO_USDO_POS_EXP_DATA + size))) {
        prepareAbort(USDO, (uint32_t)CO_SDO_AB_CMD);
        return;
    }

    OD_size_t sizeToWrite;
    uint32_t abortCode = verifyDownloadSize(USDO, size, &sizeToWrite);
    if (abortCode == (uint32_t)CO_SDO_AB_NONE) {
        uint8_t buf[CO_USDO_EXP_MAX_DATA + 2U] = {0};
        (void)memcpy(buf, &rx[CO_USDO_POS_EXP_DATA], size);
        abortCode = writeToOd(USDO, buf, sizeToWrite, true);
    }

    if (abortCode != (uint32_t)CO_SDO_AB_NONE) {
        prepareAbort(USDO, abortCode);
    } else {
        prepareHeader(USDO, (uint8_t)CO_USDO_CS_DOWN_EXP_RSP, true);
        USDO->txLen = CO_USDO_LEN_INITIATE;
        USDO->state = CO_USDO_ST_RESPONSE;
    }
}

#if ((CO_CONFIG_USDO_SRV)&CO_CONFIG_USDO_SRV_SEGMENTED) != 0
/*
 * Read data from OD variable into the free space of the buffer. Sets USDO->finished when variable is completely read.
 * For strings, data is truncated at the first zero byte. Returns abort code.
 */
static uint32_t
readFromOd(CO_USDOserver_t* USDO) {
    /* move unread data to the beginning of the buffer */
    OD_size_t unread = USDO->bufOffsetWr - USDO->bufOffsetRd;
    if ((unread > 0U) && (USDO->bufOffsetRd > 0U)) {
        (void)memmove(USDO->buf, &USDO->buf[USDO->bufOffsetRd], unread);
    }
    USDO->bufOffsetRd = 0;
    USDO->bufOffsetWr = unread;

    if (USDO->finished) {
        return (uint32_t)CO_SDO_AB_NONE;
    }

    OD_size_t countFree = (OD_size_t)sizeof(USDO->buf) - USDO->bufOffsetWr;
    OD_size_t countRd = 0;
    uint8_t* bufShifted = &USDO->buf[USDO->bufOffsetWr];
    bool_t firstRead = USDO->OD_IO.stream.dataOffset == 0U;

    CO_LOCK_OD(USDO->CANdevTx);
    ODR_t odRet = USDO->OD_IO.read(&USDO->OD_IO.stream, bufShifted, countFree, &countRd);
    CO_UNLOCK_OD(USDO->CANdevTx);

    if ((odRet != ODR_OK) && (odRet != ODR_PARTIAL)) {
        return OD_getSDOabCode(odRet);
    }

    USDO->finished = odRet == ODR_OK;

    /* string data ends at the first zero byte */
    if ((USDO->OD_IO.stream.attribute & (OD_attr_t)ODA_STR) != 0U) {
        for (OD_size_t i = 0; i < countRd; i++) {
            if (bufShifted[i] == 0U) {
                countRd = i;
                USDO->finished = true;
                break;
            }
        }
    }
#ifdef CO_BIG_ENDIAN
    else if (((USDO->OD_IO.stream.attribute & (OD_attr_t)ODA_MB) != 0U) && USDO->finished && firstRead) {
        reverseBytes(bufShifted, countRd);
    }
#endif
    (void)firstRead;

    USDO->bufOffsetWr += countRd;
    USDO->sizeTran += countRd;
    return (uint32_t)CO_SDO_AB_NONE;
}

/* Download segmented initial request */
static void
downloadSegmentedInitiate(CO_USDOserver_t* USDO, const uint8_t* rx, uint8_t rxLen) {
    if (rxLen < CO_USDO_LEN_SEG_INITIATE) {
        prepareAbort(USDO, (uint32_t)CO_SDO_AB_CMD);
        return;
    }
    uint32_t sizeInd = CO_SWAP_32(CO_getUint32(&rx[CO_USDO_POS_SEG_SIZE]));
    OD_size_t sizeToWrite = 0;
    uint32_t abortCode = (uint32_t)CO_SDO_AB_NONE;

    if (sizeInd != 0U) {
        abortCode = verifyDownloadSize(USDO, (OD_size_t)sizeInd, &sizeToWrite);
    }
    if (abortCode != (uint32_t)CO_SDO_AB_NONE) {
        prepareAbort(USDO, abortCode);
        return;
    }

    USDO->sizeInd = (OD_size_t)sizeInd;
    USDO->sizeTran = 0;
    USDO->bufOffsetWr = 0;
    USDO->bufOffsetRd = 0;
    USDO->seq = 0;
    USDO->timeoutTimer = 0;
    prepareHeader(USDO, (uint8_t)CO_USDO_CS_DOWN_SEG_INIT_RSP, true);
    USDO->txLen = CO_USDO_LEN_SEG_INITIATE;
    USDO->state = CO_USDO_ST_DOWNLOAD_SEG;
}

/* Download segment */
static void
downloadSegment(CO_USDOserver_t* USDO, const uint8_t* rx, uint8_t rxLen, bool_t last) {
    uint8_t count = rx[CO_USDO_POS_SEGDATA_SIZE];
    if ((rx[CO_USDO_POS_SEQ] != USDO->seq) || (count > CO_USDO_SEG_MAX_DATA)
        || (rxLen < (CO_USDO_POS_SEGDATA + count))) {
        prepareAbort(USDO, (uint32_t)((rx[CO_USDO_POS_SEQ] != USDO->seq) ? CO_SDO_AB_SEQ_NUM : CO_SDO_AB_CMD));
        return;
    }

    USDO->sizeTran += count;
    if ((USDO->sizeInd > 0U) && (USDO->sizeTran > USDO->sizeInd)) {
        prepareAbort(USDO, (uint32_t)CO_SDO_AB_DATA_LONG);
        return;
    }

    /* flush buffer to OD, if new segment does not fit */
    uint32_t abortCode = (uint32_t)CO_SDO_AB_NONE;
    if ((USDO->bufOffsetWr + count) > (OD_size_t)sizeof(USDO->buf)) {
        abortCode = writeToOd(USDO, USDO->buf, USDO->bufOffsetWr, false);
        USDO->bufOffsetWr = 0;
    }
    if (abortCode == (uint32_t)CO_SDO_AB_NONE) {
        (void)memcpy(&USDO->buf[USDO->bufOffsetWr], &rx[CO_USDO_POS_SEGDATA], count);
        USDO->bufOffsetWr += count;

        if (last) {
            if ((USDO->sizeInd > 0U) && (USDO->sizeTran < USDO->sizeInd)) {
                abortCode = (uint32_t)CO_SDO_AB_DATA_SHORT;
            } else {
                OD_size_t sizeInOd = USDO->OD_IO.stream.dataLength;
                OD_size_t count2 = USDO->bufOffsetWr;
                /* size was not indicated, verify it now; terminate strings */
                if (USDO->sizeInd == 0U) {
                    OD_size_t sizeToWrite;
                    USDO->OD_IO.stream.dataLength = sizeInOd;
                    abortCode = verifyDownloadSize(USDO, USDO->sizeTran, &sizeToWrite);
                    count2 += sizeToWrite - USDO->sizeTran;
                } else {
                    count2 += sizeInOd - USDO->sizeTran;
                }
                if ((abortCode == (uint32_t)CO_SDO_AB_NONE) && (count2 > (OD_size_t)sizeof(USDO->buf))) {
                    /* no space for string termination, write data first */
                    abortCode = writeToOd(USDO, USDO->buf, USDO->bufOffsetWr, false);
                    count2 -= USDO->bufOffsetWr;
                    USDO->bufOffsetWr = 0;
                }
                if (abortCode == (uint32_t)CO_SDO_AB_NONE) {
                    (void)memset(&USDO->buf[USDO->bufOffsetWr], 0, count2 - USDO->bufOffsetWr);
                    abortCode = writeToOd(USDO, USDO->buf, count2, true);
                }
            }
        }
    }

    if (abortCode != (uint32_t)CO_SDO_AB_NONE) {
        prepareAbort(USDO, abortCode);
    } else {
        prepareHeader(USDO, (uint8_t)(last ? CO_USDO_CS_DOWN_SEG_LAST_RSP : CO_USDO_CS_DOWN_SEG_RSP), false);
        USDO->txData[CO_USDO_POS_SEQ] = USDO->seq;
        USDO->txLen = CO_USDO_LEN_SEG_CONFIRM;
        USDO->seq++;
        USDO->timeoutTimer = 0;
        USDO->state = last ? CO_USDO_ST_RESPONSE : CO_USDO_ST_DOWNLOAD_SEG;
    }
}

/* Prepare next upload segment */
static void
uploadSegment(CO_USDOserver_t* USDO, const uint8_t* rx) {
    if (rx[CO_USDO_POS_SEQ] != USDO->seq) {
        prepareAbort(USDO, (uint32_t)CO_SDO_AB_SEQ_NUM);
        return;
    }

    uint32_t abortCode = (uint32_t)CO_SDO_AB_NONE;
    if ((USDO->bufOffsetWr - USDO->bufOffsetRd) < CO_USDO_SEG_MAX_DATA) {
        abortCode = readFromOd(USDO);
    }
    if ((abortCode == (uint32_t)CO_SDO_AB_NONE) && (USDO->sizeInd > 0U) && (USDO->sizeTran > USDO->sizeInd)) {
        abortCode = (uint32_t)CO_SDO_AB_DEVICE_INCOMPAT;
    }
    if (abortCode != (uint32_t)CO_SDO_AB_NONE) {
        prepareAbort(USDO, abortCode);
        return;
    }

    OD_size_t count = USDO->bufOffsetWr - USDO->bufOffsetRd;
    if (count > CO_USDO_SEG_MAX_DATA) {
        count = CO_USDO_SEG_MAX_DATA;
    }
    bool_t last = USDO->finished && ((USDO->bufOffsetRd + count) == USDO->bufOffsetWr);

    prepareHeader(USDO, (uint8_t)(last ? CO_USDO_CS_UP_SEG_LAST_RSP : CO_USDO_CS_UP_SEG_RSP), false);
    USDO->txData[CO_USDO_POS_SEQ] = USDO->seq;
    USDO->txData[CO_USDO_POS_SEGDATA_SIZE] = (uint8_t)count;
    (void)memcpy(&USDO->txData[CO_USDO_POS_SEGDATA], &USDO->buf[USDO->bufOffsetRd], count);
    USDO->bufOffsetRd += count;
    USDO->txLen = (uint8_t)(CO_USDO_POS_SEGDATA + count);
    USDO->seq++;
    USDO->timeoutTimer = 0;
    USDO->state = last ? CO_USDO_ST_RESPONSE : CO_USDO_ST_UPLOAD_SEG;
}
#endif /* (CO_CONFIG_USDO_SRV)&CO_CONFIG_USDO_SRV_SEGMENTED */

/* Upload request */
static void
uploadInitiate(CO_USDOserver_t* USDO) {
#if ((CO_CONFIG_USDO_SRV)&CO_CONFIG_USDO_SRV_SEGMENTED) != 0
    USDO->bufOffsetRd = 0;
    USDO->bufOffsetWr = 0;
    USDO->sizeTran = 0;
    USDO->finished = false;
    USDO->seq = 0;
    USDO->timeoutTimer = 0;

    uint32_t abortCode = readFromOd(USDO);
    if (abortCode != (uint32_t)CO_SDO_AB_NONE) {
        prepareAbort(USDO, abortCode);
        return;
    }
    uint8_t* data = USDO->buf;
    OD_size_t size = USDO->bufOffsetWr;
    bool_t finished = USDO->finished;
#else
    uint8_t data[CO_USDO_EXP_MAX_DATA];
    OD_size_t size = 0;

    CO_LOCK_OD(USDO->CANdevTx);
    ODR_t odRet = USDO->OD_IO.read(&USDO->OD_IO.stream, data, sizeof(data), &size);
    CO_UNLOCK_OD(USDO->CANdevTx);

    if ((odRet != ODR_OK) && (odRet != ODR_PARTIAL)) {
        prepareAbort(USDO, OD_getSDOabCode(odRet));
        return;
    }
    bool_t finished = odRet == ODR_OK;
    if ((USDO->OD_IO.stream.attribute & (OD_attr_t)ODA_STR) != 0U) {
        for (OD_size_t i = 0; i < size; i++) {
            if (data[i] == 0U) {
                size = i;
                finished = true;
                break;
            }
        }
    }
#ifdef CO_BIG_ENDIAN
    else if (((USDO->OD_IO.stream.attribute & (OD_attr_t)ODA_MB) != 0U) && finished) {
        reverseBytes(data, size);
    }
#endif
#endif

    if (finished && (size <= CO_USDO_EXP_MAX_DATA)) {
        /* expedited response */
        if ((USDO->OD_IO.stream.dataLength > 0U) && ((USDO->OD_IO.stream.attribute & (OD_attr_t)ODA_STR) == 0U)
            && (size != USDO->OD_IO.stream.dataLength)) {
            prepareAbort(USDO, (uint32_t)CO_SDO_AB_DEVICE_INCOMPAT);
            return;
        }
        prepareHeader(USDO, (uint8_t)CO_USDO_CS_UP_EXP_RSP, true);
        USDO->txData[CO_USDO_POS_DT] = dataTypeOf(&USDO->OD_IO.stream);
        USDO->txData[CO_USDO_POS_EXP_SIZE] = (uint8_t)size;
        (void)memcpy(&USDO->txData[CO_USDO_POS_EXP_DATA], data, size);
        USDO->txLen = (uint8_t)(CO_USDO_POS_EXP_DATA + size);
        USDO->state = CO_USDO_ST_RESPONSE;
    } else {
#if ((CO_CONFIG_USDO_SRV)&CO_CONFIG_USDO_SRV_SEGMENTED) != 0
        /* segmented response, size is not known for strings */
        USDO->sizeInd = ((USDO->OD_IO.stream.attribute & (OD_attr_t)ODA_STR) == 0U) ? USDO->OD_IO.stream.dataLength
                                                                                    : 0U;
        prepareHeader(USDO, (uint8_t)CO_USDO_CS_UP_SEG_INIT_RSP, true);
        USDO->txData[CO_USDO_POS_DT] = dataTypeOf(&USDO->OD_IO.stream);
        (void)CO_setUint32(&USDO->txData[CO_USDO_POS_SEG_SIZE], CO_SWAP_32((uint32_t)USDO->sizeInd));
        USDO->txLen = CO_USDO_LEN_SEG_INITIATE;
        USDO->state = CO_USDO_ST_UPLOAD_SEG;
#else
        prepareAbort(USDO, (uint32_t)CO_SDO_AB_OUT_OF_MEM);
#endif
    }
}

/* Process new request, which starts the transfer */
static void
processInitiate(CO_USDOserver_t* USDO, const uint8_t* rx, uint8_t rxLen) {
    uint8_t cs = rx[CO_USDO_POS_CS];

    USDO->clientId = USDO->CANrxClient;
    USDO->sessionId = rx[CO_USDO_POS_SID];
    USDO->destAddr = rx[CO_USDO_POS_DA];
    USDO->subIndex = (rxLen > CO_USDO_POS_SUBIDX) ? rx[CO_USDO_POS_SUBIDX] : 0U;
    USDO->index = (rxLen >= CO_USDO_LEN_INITIATE) ? CO_SWAP_16(CO_getUint16(&rx[CO_USDO_POS_IDX])) : 0U;

    bool_t upload = cs == (uint8_t)CO_USDO_CS_UP_REQ;
    bool_t download = (cs == (uint8_t)CO_USDO_CS_DOWN_EXP_REQ)
#if ((CO_CONFIG_USDO_SRV)&CO_CONFIG_USDO_SRV_SEGMENTED) != 0
                      || (cs == (uint8_t)CO_USDO_CS_DOWN_SEG_INIT_REQ)
#endif
        ;

    if ((!upload && !download) || (rxLen < CO_USDO_LEN_INITIATE)) {
        /* Remote access, bulk transfer and segments without session are not supported */
        prepareAbort(USDO, (uint32_t)CO_SDO_AB_CMD);
        return;
    }

    ODR_t odRet = OD_getSub(OD_find(USDO->OD, USDO->index), USDO->subIndex, &USDO->OD_IO, false);
    if (odRet != ODR_OK) {
        prepareAbort(USDO, OD_getSDOabCode(odRet));
        return;
    }
    OD_attr_t attr = USDO->OD_IO.stream.attribute;
    if ((attr & (OD_attr_t)ODA_SDO_RW) == 0U) {
        prepareAbort(USDO, (uint32_t)CO_SDO_AB_UNSUPPORTED_ACCESS);
    } else if (upload && ((attr & (OD_attr_t)ODA_SDO_R) == 0U)) {
        prepareAbort(USDO, (uint32_t)CO_SDO_AB_WRITEONLY);
    } else if (download && ((attr & (OD_attr_t)ODA_SDO_W) == 0U)) {
        prepareAbort(USDO, (uint32_t)CO_SDO_AB_READONLY);
    } else if (upload) {
        uploadInitiate(USDO);
    } else if (cs == (uint8_t)CO_USDO_CS_DOWN_EXP_REQ) {
        downloadExpedited(USDO, rx, rxLen);
    } else {
#if ((CO_CONFIG_USDO_SRV)&CO_CONFIG_USDO_SRV_SEGMENTED) != 0
        downloadSegmentedInitiate(USDO, rx, rxLen);
#endif
    }
}

CO_USDO_return_t
CO_USDOserver_process(CO_USDOserver_t* USDO, bool_t NMTisPreOrOperational, uint32_t timeDifference_us,
                      uint32_t* timerNext_us) {
    (void)timerNext_us;      /* may be unused */
    (void)timeDifference_us; /* may be unused */
    CO_USDO_return_t ret = CO_USDO_RT_waitingResponse;

    if (!NMTisPreOrOperational || !USDO->valid) {
        /* USDO is allowed only in operational or pre-operational NMT state */
        USDO->state = CO_USDO_ST_IDLE;
        CO_FLAG_CLEAR(USDO->CANrxNew);
        return CO_USDO_RT_ok_communicationEnd;
    }

    /* process received frame */
    if (CO_FLAG_READ(USDO->CANrxNew) && (USDO->txLen == 0U)) {
        const uint8_t* rx = USDO->CANrxData;
        uint8_t rxLen = USDO->CANrxLen;
        uint8_t cs = rx[CO_USDO_POS_CS];

        if (USDO->state == CO_USDO_ST_IDLE) {
            processInitiate(USDO, rx, rxLen);
        }
#if ((CO_CONFIG_USDO_SRV)&CO_CONFIG_USDO_SRV_SEGMENTED) != 0
        else if ((USDO->state == CO_USDO_ST_DOWNLOAD_SEG)
                 && ((cs == (uint8_t)CO_USDO_CS_DOWN_SEG_REQ) || (cs == (uint8_t)CO_USDO_CS_DOWN_SEG_LAST_REQ))) {
            downloadSegment(USDO, rx, rxLen, cs == (uint8_t)CO_USDO_CS_DOWN_SEG_LAST_REQ);
        } else if ((USDO->state == CO_USDO_ST_UPLOAD_SEG) && (cs == (uint8_t)CO_USDO_CS_UP_SEG_REQ)) {
            uploadSegment(USDO, rx);
        }
#endif
        else {
            prepareAbort(USDO, (uint32_t)CO_SDO_AB_CMD);
        }
        CO_FLAG_CLEAR(USDO->CANrxNew);
    }
#if ((CO_CONFIG_USDO_SRV)&CO_CONFIG_USDO_SRV_SEGMENTED) != 0
    else if ((USDO->state == CO_USDO_ST_DOWNLOAD_SEG) || (USDO->state == CO_USDO_ST_UPLOAD_SEG)) {
        /* waiting for the client */
        if (USDO->txLen == 0U) {
            if (USDO->timeoutTimer < USDO->timeoutTime_us) {
                USDO->timeoutTimer += timeDifference_us;
            }
            if (USDO->timeoutTimer >= USDO->timeoutTime_us) {
                prepareAbort(USDO, (uint32_t)CO_SDO_AB_TIMEOUT);
            }
#if ((CO_CONFIG_USDO_SRV)&CO_CONFIG_FLAG_TIMERNEXT) != 0
            else if (timerNext_us != NULL) {
                uint32_t diff = USDO->timeoutTime_us - USDO->timeoutTimer;
                if (*timerNext_us > diff) {
                    *timerNext_us = diff;
                }
            }
#endif
        }
    }
#endif
    else { /* MISRA C 2004 14.10 */
    }

    /* transmit prepared response */
    if (USDO->txLen > 0U) {
        if (USDO->CANtxBuff->bufferFull) {
            ret = CO_USDO_RT_transmittBufferFull;
        } else {
            uint8_t frameLen = CO_CANfd_roundLength(USDO->txLen);
            (void)memset(USDO->CANtxBuff->data, 0, CO_USDO_MAX_DLEN);
            (void)memcpy(USDO->CANtxBuff->data, USDO->txData, USDO->txLen);
            USDO->CANtxBuff->DLC = frameLen;
            (void)CO_CANsend(USDO->CANdevTx, USDO->CANtxBuff);
            USDO->txLen = 0;

            if (USDO->state == CO_USDO_ST_ABORT) {
                USDO->state = CO_USDO_ST_IDLE;
                ret = CO_USDO_RT_endedWithServerAbort;
            } else if (USDO->state == CO_USDO_ST_RESPONSE) {
                USDO->state = CO_USDO_ST_IDLE;
                ret = CO_USDO_RT_ok_communicationEnd;
            } else { /* MISRA C 2004 14.10 */
            }
        }
    } else if (USDO->state == CO_USDO_ST_IDLE) {
        ret = CO_USDO_RT_ok_communicationEnd;
    } else { /* MISRA C 2004 14.10 */
    }

    return ret;
}

#endif /* ((CO_CONFIG_FD)&CO_CONFIG_FD_ENABLE) != 0 */
