/**
 * CANopen FD Universal Service Data Object - server protocol.
 *
 * @file        CO_USDOserver.h
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

#ifndef CO_USDO_SERVER_H
#define CO_USDO_SERVER_H

#include "301/CO_driver.h"
#include "301/CO_ODinterface.h"
#include "301/CO_SDOserver.h"

/* default configuration, see CO_config.h */
#ifndef CO_CONFIG_USDO_SRV
#define CO_CONFIG_USDO_SRV                                                                                             \
    (CO_CONFIG_USDO_SRV_SEGMENTED | CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE | CO_CONFIG_GLOBAL_FLAG_TIMERNEXT)
#endif
#ifndef CO_CONFIG_USDO_SRV_BUFFER_SIZE
#define CO_CONFIG_USDO_SRV_BUFFER_SIZE 128
#endif

#if (((CO_CONFIG_FD)&CO_CONFIG_FD_ENABLE) != 0) || defined CO_DOXYGEN

#if CO_CONFIG_USDO_SRV_BUFFER_SIZE < 64
#error CO_CONFIG_USDO_SRV_BUFFER_SIZE must be at least 64.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup CO_USDOserver USDO server
 * CANopen FD Universal Service Data Object - server protocol, specified by CiA 1301.
 *
 * @ingroup CO_CANopen_301
 * @{
 * USDO server provides read (upload) and write (download) access to the Object Dictionary for USDO clients. Unlike
 * classic SDO (CiA 301), the destination of the request is carried inside the protocol, so a USDO request may be sent
 * to one node (unicast), to a group of nodes (multicast) or to all nodes (broadcast). Each server responds
 * individually.
 *
 * CAN identifiers of the pre-defined connection set (not configurable):
 * - Request from USDO client: 600h + node-ID of the client.
 * - Response from USDO server: 580h + node-ID of the server.
 *
 * Implemented services (local access only):
 * - Download expedited and upload expedited, up to @ref CO_USDO_EXP_MAX_DATA bytes in one CAN FD frame.
 * - Download segmented and upload segmented, if CO_CONFIG_USDO_SRV_SEGMENTED is set.
 * - Abort.
 *
 * Not yet implemented: bulk transfer, remote access (command specifiers 81h to FFh), concurrent sessions.
 *
 * @warning The byte layout of the USDO frames below follows public descriptions of CiA 1301 (command specifiers,
 * field order and frame lengths). It must be verified against the CiA 1301 document before claiming conformance. All
 * byte positions are defined by the CO_USDO_POS_xx macros in this file, so they can be corrected in one place.
 *
 * Frame layout used (byte positions, multi-byte values little endian):
 * - All frames: 0 destination address (DA), 1 command specifier (CS), 2 session-ID (SID).
 * - Initiate requests, expedited frames and abort: 3 sub-index, 4..5 index.
 * - Expedited data (CS 01h, 31h): 6 data type, 7 data size, 8.. data.
 * - Segmented initiate (CS 02h, 32h): 6 data type, 7 reserved, 8..11 total data size (0 if not known).
 * - Segments (CS 03h, 04h, 33h, 34h): 3 sequence number, 4 data size in this segment, 5.. data.
 * - Segment requests and confirmations (CS 13h, 23h, 24h): 3 sequence number.
 * - Abort (CS 7Fh): 6..9 abort code.
 *
 * Data type field contains CANopen data type index (for example 07h for UNSIGNED32). CANopenNode Object Dictionary
 * does not store data types, so data type in upload response is derived from the attributes and the length of the
 * variable, and data type in download request is not verified. Data length is always verified.
 */

/** Maximum data length of the USDO frame */
#define CO_USDO_MAX_DLEN         64U

/** Position of destination address */
#define CO_USDO_POS_DA           0U
/** Position of command specifier */
#define CO_USDO_POS_CS           1U
/** Position of session-ID */
#define CO_USDO_POS_SID          2U
/** Position of sub-index */
#define CO_USDO_POS_SUBIDX       3U
/** Position of index (2 bytes) */
#define CO_USDO_POS_IDX          4U
/** Position of data type */
#define CO_USDO_POS_DT           6U
/** Position of data size in expedited frame */
#define CO_USDO_POS_EXP_SIZE     7U
/** Position of data in expedited frame */
#define CO_USDO_POS_EXP_DATA     8U
/** Position of total data size in segmented initiate frame (4 bytes) */
#define CO_USDO_POS_SEG_SIZE     8U
/** Position of sequence number in segment frame */
#define CO_USDO_POS_SEQ          3U
/** Position of data size in segment frame */
#define CO_USDO_POS_SEGDATA_SIZE 4U
/** Position of data in segment frame */
#define CO_USDO_POS_SEGDATA      5U
/** Position of abort code (4 bytes) */
#define CO_USDO_POS_ABORT        6U

/** Length of the initiate request (upload) and of expedited download response */
#define CO_USDO_LEN_INITIATE     6U
/** Length of the segmented initiate request and response */
#define CO_USDO_LEN_SEG_INITIATE 12U
/** Length of the segment confirmation and segment request */
#define CO_USDO_LEN_SEG_CONFIRM  4U
/** Length of the abort frame */
#define CO_USDO_LEN_ABORT        10U

/** Maximum data size in expedited frame */
#define CO_USDO_EXP_MAX_DATA     (CO_USDO_MAX_DLEN - CO_USDO_POS_EXP_DATA)
/** Maximum data size in one segment */
#define CO_USDO_SEG_MAX_DATA     (CO_USDO_MAX_DLEN - CO_USDO_POS_SEGDATA)

/** Destination address for broadcast */
#define CO_USDO_DA_BROADCAST     0x00U
/** First destination address of multicast groups */
#define CO_USDO_DA_MULTICAST     0x80U

/**
 * USDO command specifiers, local access. Remote access uses the same value + 80h.
 */
typedef enum {
    CO_USDO_CS_DOWN_EXP_REQ = 0x01U,       /**< Download expedited request */
    CO_USDO_CS_DOWN_SEG_INIT_REQ = 0x02U,  /**< Download segmented initial request */
    CO_USDO_CS_DOWN_SEG_REQ = 0x03U,       /**< Download segmented intermediate request (segment) */
    CO_USDO_CS_DOWN_SEG_LAST_REQ = 0x04U,  /**< Download segmented last request (end) */
    CO_USDO_CS_DOWN_BULK_INIT_REQ = 0x06U, /**< Download bulk transfer initial request */
    CO_USDO_CS_DOWN_BULK_SEG_REQ = 0x07U,  /**< Download bulk transfer segmented */
    CO_USDO_CS_DOWN_BULK_END_REQ = 0x08U,  /**< Download bulk transfer final */
    CO_USDO_CS_UP_REQ = 0x11U,             /**< Upload request */
    CO_USDO_CS_UP_SEG_REQ = 0x13U,         /**< Upload segmented intermediate request (segment) */
    CO_USDO_CS_DOWN_EXP_RSP = 0x21U,       /**< Download expedited response */
    CO_USDO_CS_DOWN_SEG_INIT_RSP = 0x22U,  /**< Download segmented initial response */
    CO_USDO_CS_DOWN_SEG_RSP = 0x23U,       /**< Download segmented intermediate response (segment) */
    CO_USDO_CS_DOWN_SEG_LAST_RSP = 0x24U,  /**< Download segmented last response (end) */
    CO_USDO_CS_DOWN_BULK_INIT_RSP = 0x26U, /**< Download bulk transfer initial response */
    CO_USDO_CS_DOWN_BULK_END_RSP = 0x28U,  /**< Download bulk transfer final response */
    CO_USDO_CS_UP_EXP_RSP = 0x31U,         /**< Upload expedited response */
    CO_USDO_CS_UP_SEG_INIT_RSP = 0x32U,    /**< Upload segmented initial response */
    CO_USDO_CS_UP_SEG_RSP = 0x33U,         /**< Upload segmented intermediate response (segment) */
    CO_USDO_CS_UP_SEG_LAST_RSP = 0x34U,    /**< Upload segmented last response (end) */
    CO_USDO_CS_ABORT = 0x7FU,              /**< USDO abort */
    CO_USDO_CS_REMOTE = 0x80U              /**< Added to command specifier for remote access */
} CO_USDO_CS_t;

/**
 * Internal states of the USDO server state machine
 */
typedef enum {
    CO_USDO_ST_IDLE = 0x00U,         /**< No transfer in progress */
    CO_USDO_ST_ABORT = 0x01U,        /**< Abort is pending */
    CO_USDO_ST_DOWNLOAD_SEG = 0x02U, /**< Download segmented, waiting for next segment */
    CO_USDO_ST_UPLOAD_SEG = 0x03U,   /**< Upload segmented, waiting for next segment request */
    CO_USDO_ST_RESPONSE = 0x04U      /**< Response is prepared and pending for transmission */
} CO_USDO_state_t;

/**
 * Return values of USDO server process function
 */
typedef enum {
    CO_USDO_RT_ok_communicationEnd = 0,  /**< Idle or transfer finished successfully */
    CO_USDO_RT_waitingResponse = 1,      /**< Transfer in progress */
    CO_USDO_RT_transmittBufferFull = 2,  /**< Response is waiting, because CAN transmit buffer is full */
    CO_USDO_RT_endedWithServerAbort = -1 /**< Transfer ended with abort */
} CO_USDO_return_t;

/**
 * USDO server object.
 */
typedef struct {
    OD_t* OD;                 /**< From CO_USDOserver_init() */
    uint8_t nodeId;           /**< From CO_USDOserver_init() */
    bool_t valid;             /**< True if nodeId is valid (1 to 127) */
    CO_CANmodule_t* CANdevTx; /**< From CO_USDOserver_init() */
    CO_CANtx_t* CANtxBuff;    /**< CAN transmit buffer inside CANdevTx for USDO responses */
    CO_USDO_state_t state;    /**< Internal state of the USDO server */
    uint32_t multicast[4];    /**< Bitfield of multicast groups (destination addresses 80h to FFh) of this server */
    OD_IO_t OD_IO;            /**< Object dictionary interface for current transfer */
    uint16_t index;           /**< Index of the current object in Object Dictionary */
    uint8_t subIndex;         /**< Subindex of the current object in Object Dictionary */
    uint8_t clientId;         /**< Node-ID of the USDO client of the current transfer */
    uint8_t sessionId;        /**< Session-ID of the current transfer */
    uint8_t destAddr;         /**< Destination address of the request of the current transfer */
    uint8_t seq;              /**< Expected sequence number of the next segment */
    uint32_t abortCode;       /**< Abort code of pending abort, see @ref CO_SDO_abortCode_t */
    uint8_t txData[CO_USDO_MAX_DLEN]; /**< Prepared response */
    uint8_t txLen;                    /**< Length of prepared response, 0 if none */
    uint32_t timeoutTime_us;          /**< Timeout time for segmented transfer in microseconds */
    uint32_t timeoutTimer;            /**< Timeout timer for segmented transfer */
#if (((CO_CONFIG_USDO_SRV)&CO_CONFIG_USDO_SRV_SEGMENTED) != 0) || defined CO_DOXYGEN
    uint8_t buf[CO_CONFIG_USDO_SRV_BUFFER_SIZE]; /**< Data buffer for segmented transfer */
    OD_size_t bufOffsetWr;                       /**< Offset of next free data byte in buf */
    OD_size_t bufOffsetRd;                       /**< Offset of first unread data byte in buf */
    OD_size_t sizeInd;                           /**< Indicated data size, 0 if not known */
    OD_size_t sizeTran;                          /**< Data size transferred to or from Object Dictionary */
    bool_t finished;                             /**< True if Object Dictionary variable was completely read */
#endif
    volatile void* CANrxNew;             /**< Indicates, if new USDO frame is received. Set by receive callback */
    uint8_t CANrxData[CO_USDO_MAX_DLEN]; /**< Data of the received frame */
    uint8_t CANrxLen;                    /**< Length of the received frame */
    uint8_t CANrxClient;                 /**< Node-ID of the client, which sent the received frame */
#if (((CO_CONFIG_USDO_SRV)&CO_CONFIG_FLAG_CALLBACK_PRE) != 0) || defined CO_DOXYGEN
    void (*pFunctSignalPre)(void* object); /**< From CO_USDOserver_initCallbackPre() or NULL */
    void* functSignalObjectPre;            /**< From CO_USDOserver_initCallbackPre() or NULL */
#endif
} CO_USDOserver_t;

/**
 * Initialize USDO server object.
 *
 * Function must be called in the communication reset section.
 *
 * @param USDO This object will be initialized.
 * @param OD Object Dictionary.
 * @param nodeId CANopen Node ID of this device. If it is not 1 to 127, USDO server is disabled.
 * @param timeoutTime_ms Timeout time for segmented transfer in milliseconds.
 * @param CANdevRx CAN device for USDO server reception.
 * @param CANdevRxIdx Index of receive buffer in the above CAN device.
 * @param CANdevTx CAN device for USDO server transmission.
 * @param CANdevTxIdx Index of transmit buffer in the above CAN device.
 * @param [out] errInfo Additional information in case of error, may be NULL.
 *
 * @return #CO_ReturnError_t: CO_ERROR_NO or CO_ERROR_ILLEGAL_ARGUMENT.
 */
CO_ReturnError_t CO_USDOserver_init(CO_USDOserver_t* USDO, OD_t* OD, uint8_t nodeId, uint16_t timeoutTime_ms,
                                    CO_CANmodule_t* CANdevRx, uint16_t CANdevRxIdx, CO_CANmodule_t* CANdevTx,
                                    uint16_t CANdevTxIdx, uint32_t* errInfo);

#if (((CO_CONFIG_USDO_SRV)&CO_CONFIG_FLAG_CALLBACK_PRE) != 0) || defined CO_DOXYGEN
/**
 * Initialize USDO server callback function.
 *
 * Function initializes optional callback function, which should immediately start processing of CO_USDOserver_process()
 * function. Callback is called after USDO server message is received from the CAN bus or when new call without delay
 * is necessary (segmented transfer is in progress).
 *
 * @param USDO This object.
 * @param object Pointer to object, which will be passed to pFunctSignal(). Can be NULL
 * @param pFunctSignal Pointer to the callback function. Not called if NULL.
 */
void CO_USDOserver_initCallbackPre(CO_USDOserver_t* USDO, void* object, void (*pFunctSignal)(void* object));
#endif

/**
 * Add or remove USDO server to or from a multicast group.
 *
 * @param USDO This object.
 * @param group Multicast destination address, 80h to FFh.
 * @param member True to join, false to leave the group.
 *
 * @return #CO_ReturnError_t: CO_ERROR_NO or CO_ERROR_ILLEGAL_ARGUMENT.
 */
CO_ReturnError_t CO_USDOserver_setMulticast(CO_USDOserver_t* USDO, uint8_t group, bool_t member);

/**
 * Process USDO communication.
 *
 * Function must be called cyclically.
 *
 * @param USDO This object.
 * @param NMTisPreOrOperational True if NMT state is pre-operational or operational.
 * @param timeDifference_us Time difference from previous function call in [microseconds].
 * @param [out] timerNext_us info to OS - see CO_process().
 *
 * @return #CO_USDO_return_t.
 */
CO_USDO_return_t CO_USDOserver_process(CO_USDOserver_t* USDO, bool_t NMTisPreOrOperational, uint32_t timeDifference_us,
                                       uint32_t* timerNext_us);

/** @} */ /* CO_USDOserver */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* ((CO_CONFIG_FD)&CO_CONFIG_FD_ENABLE) != 0 */

#endif /* CO_USDO_SERVER_H */
