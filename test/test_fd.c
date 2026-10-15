/*
 * Host unit tests for CANopen FD support: CAN FD frame lengths, PDO up to 64 bytes and USDO server.
 *
 * Build and run: make -C test
 */

#include <stdio.h>
#include <string.h>

#define OD_DEFINITION
#include "301/CO_ODinterface.h"
#include "301/CO_PDO.h"
#if ((CO_CONFIG_FD)&CO_CONFIG_FD_ENABLE) != 0
#include "301/CO_USDOserver.h"
#endif

/* Minimal test framework ****************************************************/
static int failures = 0;
static int checks = 0;
#define CHECK(cond)                                                                                                    \
    do {                                                                                                               \
        checks++;                                                                                                      \
        if (!(cond)) {                                                                                                 \
            failures++;                                                                                                \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                                   \
        }                                                                                                              \
    } while (0)

/* Object Dictionary for tests ***********************************************/
#define NODE_ID              5U

/* Values of CO_RPDO_t.receiveError, private in CO_PDO.c */
#define RPDO_RX_ACK_NO_ERROR 0U
#define RPDO_RX_SHORT        12U
#define RPDO_RX_LONG         13U

static uint32_t x1000 = 0x00010191UL;
static char x1008[16] = "CANopenNode FD";

static struct {
    uint8_t sub0;
    uint32_t cobId;
    uint8_t transType;
    uint16_t eventTimer;
} x1400 = {5, 0x200U + NODE_ID, 0xFE, 0};

static struct {
    uint8_t count;
    uint32_t map[16];
} x1600, x1A00;

static struct {
    uint8_t sub0;
    uint32_t cobId;
    uint8_t transType;
    uint16_t inhibitTime;
    uint16_t eventTimer;
    uint8_t syncStart;
} x1800 = {6, 0x180U + NODE_ID, 0xFE, 0, 0, 0};

static uint8_t x2000_sub0 = 16;
static uint32_t x2000[16];
static uint8_t x2100[200];
static uint8_t x2101 = 7;

static OD_obj_var_t o1000 = {&x1000, ODA_SDO_RW | ODA_MB, 4};
static OD_obj_var_t o1008 = {x1008, ODA_SDO_R | ODA_STR, 16};
static OD_obj_record_t o1400[] = {
    {&x1400.sub0, 0, ODA_SDO_R, 1},
    {&x1400.cobId, 1, ODA_SDO_RW | ODA_MB, 4},
    {&x1400.transType, 2, ODA_SDO_RW, 1},
    {&x1400.eventTimer, 5, ODA_SDO_RW | ODA_MB, 2},
};
static OD_obj_record_t o1800[] = {
    {&x1800.sub0, 0, ODA_SDO_R, 1},
    {&x1800.cobId, 1, ODA_SDO_RW | ODA_MB, 4},
    {&x1800.transType, 2, ODA_SDO_RW, 1},
    {&x1800.inhibitTime, 3, ODA_SDO_RW | ODA_MB, 2},
    {&x1800.eventTimer, 5, ODA_SDO_RW | ODA_MB, 2},
    {&x1800.syncStart, 6, ODA_SDO_RW, 1},
};
#define MAP_RECORD(name, var)                                                                                          \
    static OD_obj_record_t name[17] = {                                                                                \
        {&var.count, 0, ODA_SDO_RW, 1},                                                                                \
        {&var.map[0], 1, ODA_SDO_RW | ODA_MB, 4},                                                                      \
        {&var.map[1], 2, ODA_SDO_RW | ODA_MB, 4},                                                                      \
        {&var.map[2], 3, ODA_SDO_RW | ODA_MB, 4},                                                                      \
        {&var.map[3], 4, ODA_SDO_RW | ODA_MB, 4},                                                                      \
        {&var.map[4], 5, ODA_SDO_RW | ODA_MB, 4},                                                                      \
        {&var.map[5], 6, ODA_SDO_RW | ODA_MB, 4},                                                                      \
        {&var.map[6], 7, ODA_SDO_RW | ODA_MB, 4},                                                                      \
        {&var.map[7], 8, ODA_SDO_RW | ODA_MB, 4},                                                                      \
        {&var.map[8], 9, ODA_SDO_RW | ODA_MB, 4},                                                                      \
        {&var.map[9], 10, ODA_SDO_RW | ODA_MB, 4},                                                                     \
        {&var.map[10], 11, ODA_SDO_RW | ODA_MB, 4},                                                                    \
        {&var.map[11], 12, ODA_SDO_RW | ODA_MB, 4},                                                                    \
        {&var.map[12], 13, ODA_SDO_RW | ODA_MB, 4},                                                                    \
        {&var.map[13], 14, ODA_SDO_RW | ODA_MB, 4},                                                                    \
        {&var.map[14], 15, ODA_SDO_RW | ODA_MB, 4},                                                                    \
        {&var.map[15], 16, ODA_SDO_RW | ODA_MB, 4},                                                                    \
    }
MAP_RECORD(o1600, x1600);
MAP_RECORD(o1A00, x1A00);
static OD_obj_array_t o2000 = {&x2000_sub0, x2000, ODA_SDO_R, ODA_SDO_RW | ODA_TRPDO | ODA_MB, 4, 4};
static OD_obj_var_t o2100 = {x2100, ODA_SDO_RW, sizeof(x2100)};
static OD_obj_var_t o2101 = {&x2101, ODA_SDO_R, 1};

static OD_entry_t odList[] = {
    {0x1000, 1, ODT_VAR, &o1000, NULL},  {0x1008, 1, ODT_VAR, &o1008, NULL}, {0x1400, 4, ODT_REC, o1400, NULL},
    {0x1600, 17, ODT_REC, o1600, NULL},  {0x1800, 6, ODT_REC, o1800, NULL},  {0x1A00, 17, ODT_REC, o1A00, NULL},
    {0x2000, 17, ODT_ARR, &o2000, NULL}, {0x2100, 1, ODT_VAR, &o2100, NULL}, {0x2101, 1, ODT_VAR, &o2101, NULL},
    {0x0000, 0, 0, NULL, NULL},
};
static OD_t od = {(uint16_t)(sizeof(odList) / sizeof(odList[0]) - 1U), odList};

/* CAN module ****************************************************************/
static CO_CANmodule_t can;
static CO_EM_t em; /* zero initialized, no emergency producer */
static CO_CANrx_t rxArray[4];
static CO_CANtx_t txArray[4];
#define RX_IDX_RPDO 0
#define RX_IDX_USDO 1
#define TX_IDX_TPDO 0
#define TX_IDX_USDO 1

/* Tests: CAN FD frame length helpers ****************************************/
#if ((CO_CONFIG_FD)&CO_CONFIG_FD_ENABLE) != 0
static void
test_canfd_lengths(void) {
    printf("CAN FD frame lengths\n");
    static const uint8_t valid[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    for (uint8_t dlc = 0; dlc < 16U; dlc++) {
        CHECK(CO_CANfd_dlcToLength(dlc) == valid[dlc]);
        CHECK(CO_CANfd_lengthToDlc(valid[dlc]) == dlc);
    }
    for (uint8_t len = 0; len <= 64U; len++) {
        uint8_t r = CO_CANfd_roundLength(len);
        uint8_t prev = 0;
        int isValid = 0;
        for (unsigned i = 0; i < sizeof(valid); i++) {
            if (valid[i] == r) {
                isValid = 1;
            }
            if (valid[i] < len) {
                prev = valid[i];
            }
        }
        (void)prev;
        CHECK(isValid);
        CHECK(r >= len);
        /* no valid length between len and r */
        for (unsigned i = 0; i < sizeof(valid); i++) {
            CHECK(!((valid[i] >= len) && (valid[i] < r)));
        }
    }
    CHECK(CO_CANfd_roundLength(9) == 12U);
    CHECK(CO_CANfd_roundLength(25) == 32U);
    CHECK(CO_CANfd_roundLength(33) == 48U);
    CHECK(CO_CANfd_roundLength(49) == 64U);
}
#endif

/* Tests: PDO ****************************************************************/
static void
configureMapping(uint8_t count) {
    x1600.count = count;
    x1A00.count = count;
    for (uint8_t i = 0; i < 16U; i++) {
        x1600.map[i] = 0x20000020UL | ((uint32_t)(i + 1U) << 8);
        x1A00.map[i] = 0x20000020UL | ((uint32_t)(i + 1U) << 8);
    }
}

static void
test_pdo(uint8_t count) {
    CO_RPDO_t rpdo;
    CO_TPDO_t tpdo;
    uint32_t errInfo = 0;
    uint8_t pdoLen = (uint8_t)(count * 4U);
    uint8_t frameLen = CO_PDO_FRAME_LENGTH(pdoLen);

    printf("PDO with %u mapped UNSIGNED32 (%u bytes, frame %u bytes)\n", count, pdoLen, frameLen);
    configureMapping(count);
    for (uint8_t i = 0; i < 16U; i++) {
        x2000[i] = 0x11111111UL * (uint32_t)(i + 1U);
    }

    CHECK(CO_CANmodule_init(&can, NULL, rxArray, 4, txArray, 4, 1000) == CO_ERROR_NO);
    CHECK(CO_TPDO_init(&tpdo, &od, &em, 0x180U + NODE_ID, &odList[4], &odList[5], &can, TX_IDX_TPDO, &errInfo)
          == CO_ERROR_NO);
    CHECK(CO_RPDO_init(&rpdo, &od, &em, 0x200U + NODE_ID, &odList[2], &odList[3], &can, RX_IDX_RPDO, &errInfo)
          == CO_ERROR_NO);
    CHECK(tpdo.PDO_common.valid);
    CHECK(rpdo.PDO_common.valid);
    CHECK(tpdo.PDO_common.dataLength == pdoLen);

    /* TPDO: frame has padded length, padding is zero */
    test_txClear();
    (void)memset(txArray[TX_IDX_TPDO].data, 0xAA, sizeof(txArray[TX_IDX_TPDO].data));
    CO_TPDOsendRequest(&tpdo);
    CO_TPDO_process(&tpdo, true, false);
    CHECK(test_txCount == 1);
    CHECK(test_txLog[0].ident == 0x180U + NODE_ID);
    CHECK(test_txLog[0].len == frameLen);
    for (uint8_t i = 0; i < count; i++) {
        uint32_t v;
        (void)memcpy(&v, &test_txLog[0].data[i * 4U], 4);
        CHECK(v == x2000[i]);
    }
    for (uint8_t i = pdoLen; i < frameLen; i++) {
        CHECK(test_txLog[0].data[i] == 0U);
    }

    /* RPDO: receive the padded frame, data written to OD, no length error */
    uint8_t frame[64];
    (void)memcpy(frame, test_txLog[0].data, frameLen);
    for (uint8_t i = 0; i < count; i++) {
        uint32_t v = 0xA0000000UL + i;
        (void)memcpy(&frame[i * 4U], &v, 4);
    }
    CHECK(test_CANrx(&can, 0x200U + NODE_ID, frame, frameLen) == 1);
    CHECK(rpdo.receiveError == RPDO_RX_ACK_NO_ERROR);
    CO_RPDO_process(&rpdo, true, false);
    for (uint8_t i = 0; i < count; i++) {
        CHECK(x2000[i] == 0xA0000000UL + i);
    }
    CHECK(!CO_isError(&em, CO_EM_RPDO_WRONG_LENGTH));

    /* RPDO: frame shorter than PDO is an error */
    if (pdoLen > 1U) {
        CHECK(test_CANrx(&can, 0x200U + NODE_ID, frame, (uint8_t)(pdoLen - 1U)) == 1);
        CHECK(rpdo.receiveError == RPDO_RX_SHORT);
        CO_RPDO_process(&rpdo, true, false);
        CHECK(CO_isError(&em, CO_EM_RPDO_WRONG_LENGTH));
        /* correct frame clears the error */
        CHECK(test_CANrx(&can, 0x200U + NODE_ID, frame, frameLen) == 1);
        CO_RPDO_process(&rpdo, true, false);
        CHECK(!CO_isError(&em, CO_EM_RPDO_WRONG_LENGTH));
    }

    /* RPDO: frame longer than the padded length is reported as too long */
    if (frameLen < CO_CAN_MAX_DLEN) {
        rpdo.receiveError = RPDO_RX_ACK_NO_ERROR;
        CHECK(test_CANrx(&can, 0x200U + NODE_ID, frame, CO_CAN_MAX_DLEN) == 1);
        CHECK(rpdo.receiveError == RPDO_RX_LONG);
    }
}

#if ((CO_CONFIG_FD)&CO_CONFIG_FD_ENABLE) != 0
/* Tests: USDO server ********************************************************/
#define CLIENT_ID 3U
static CO_USDOserver_t usdo;

static void
usdoInit(void) {
    uint32_t errInfo = 0;
    CHECK(CO_CANmodule_init(&can, NULL, rxArray, 4, txArray, 4, 1000) == CO_ERROR_NO);
    CHECK(CO_USDOserver_init(&usdo, &od, NODE_ID, 100, &can, RX_IDX_USDO, &can, TX_IDX_USDO, &errInfo) == CO_ERROR_NO);
}

/* Send request from client and process server. Returns pointer to response or NULL. */
static test_frame_t*
usdoRequest(uint8_t client, const uint8_t* req, uint8_t len) {
    uint8_t frameLen = CO_CANfd_roundLength(len);
    uint8_t frame[64] = {0};
    (void)memcpy(frame, req, len);
    test_txClear();
    (void)test_CANrx(&can, (uint16_t)(0x600U + client), frame, frameLen);
    (void)CO_USDOserver_process(&usdo, true, 1000, NULL);
    return (test_txCount == 1) ? &test_txLog[0] : NULL;
}

static uint32_t
abortCodeOf(const test_frame_t* rsp) {
    uint32_t code;
    (void)memcpy(&code, &rsp->data[CO_USDO_POS_ABORT], 4);
    return code;
}

static void
test_usdo_expedited(void) {
    printf("USDO expedited download and upload\n");
    usdoInit();

    /* download 4 bytes to 1000h */
    uint8_t dn[] = {NODE_ID, 0x01, 0x42, 0x00, 0x00, 0x10, 0x07, 4, 0x78, 0x56, 0x34, 0x12};
    test_frame_t* rsp = usdoRequest(CLIENT_ID, dn, sizeof(dn));
    CHECK(rsp != NULL);
    if (rsp != NULL) {
        CHECK(rsp->ident == 0x580U + NODE_ID);
        CHECK(rsp->len == CO_USDO_LEN_INITIATE);
        CHECK(rsp->data[CO_USDO_POS_DA] == CLIENT_ID);
        CHECK(rsp->data[CO_USDO_POS_CS] == CO_USDO_CS_DOWN_EXP_RSP);
        CHECK(rsp->data[CO_USDO_POS_SID] == 0x42);
        CHECK(rsp->data[CO_USDO_POS_SUBIDX] == 0x00);
        CHECK((rsp->data[4] == 0x00) && (rsp->data[5] == 0x10));
    }
    CHECK(x1000 == 0x12345678UL);

    /* upload 1000h */
    uint8_t up[] = {NODE_ID, 0x11, 0x43, 0x00, 0x00, 0x10};
    rsp = usdoRequest(CLIENT_ID, up, sizeof(up));
    CHECK(rsp != NULL);
    if (rsp != NULL) {
        CHECK(rsp->data[CO_USDO_POS_CS] == CO_USDO_CS_UP_EXP_RSP);
        CHECK(rsp->data[CO_USDO_POS_SID] == 0x43);
        CHECK(rsp->data[CO_USDO_POS_DT] == 0x07);
        CHECK(rsp->data[CO_USDO_POS_EXP_SIZE] == 4);
        CHECK(rsp->len == CO_CANfd_roundLength(12));
        uint32_t v;
        (void)memcpy(&v, &rsp->data[CO_USDO_POS_EXP_DATA], 4);
        CHECK(v == 0x12345678UL);
    }

    /* upload string, terminated at first zero */
    uint8_t upStr[] = {NODE_ID, 0x11, 0x44, 0x00, 0x08, 0x10};
    rsp = usdoRequest(CLIENT_ID, upStr, sizeof(upStr));
    CHECK(rsp != NULL);
    if (rsp != NULL) {
        CHECK(rsp->data[CO_USDO_POS_CS] == CO_USDO_CS_UP_EXP_RSP);
        CHECK(rsp->data[CO_USDO_POS_DT] == 0x09);
        CHECK(rsp->data[CO_USDO_POS_EXP_SIZE] == strlen(x1008));
        CHECK(memcmp(&rsp->data[CO_USDO_POS_EXP_DATA], x1008, strlen(x1008)) == 0);
    }

    /* upload array element 2000h sub 3 and sub 0 */
    x2000[2] = 0xCAFEBABEUL;
    uint8_t upArr[] = {NODE_ID, 0x11, 0x45, 0x03, 0x00, 0x20};
    rsp = usdoRequest(CLIENT_ID, upArr, sizeof(upArr));
    CHECK((rsp != NULL) && (memcmp(&rsp->data[CO_USDO_POS_EXP_DATA], &x2000[2], 4) == 0));
    uint8_t upArr0[] = {NODE_ID, 0x11, 0x46, 0x00, 0x00, 0x20};
    rsp = usdoRequest(CLIENT_ID, upArr0, sizeof(upArr0));
    CHECK((rsp != NULL) && (rsp->data[CO_USDO_POS_EXP_SIZE] == 1) && (rsp->data[CO_USDO_POS_EXP_DATA] == 16));
}

static void
test_usdo_errors(void) {
    printf("USDO aborts\n");
    usdoInit();

    /* object does not exist */
    uint8_t up[] = {NODE_ID, 0x11, 0x01, 0x00, 0x00, 0x30};
    test_frame_t* rsp = usdoRequest(CLIENT_ID, up, sizeof(up));
    CHECK((rsp != NULL) && (rsp->data[CO_USDO_POS_CS] == CO_USDO_CS_ABORT) && (rsp->len == 12U));
    CHECK((rsp != NULL) && (abortCodeOf(rsp) == 0x06020000UL));
    CHECK((rsp != NULL) && (rsp->data[CO_USDO_POS_SID] == 0x01) && (rsp->data[5] == 0x30));

    /* sub-index does not exist */
    uint8_t upSub[] = {NODE_ID, 0x11, 0x02, 0x20, 0x00, 0x20};
    rsp = usdoRequest(CLIENT_ID, upSub, sizeof(upSub));
    CHECK((rsp != NULL) && (abortCodeOf(rsp) == 0x06090011UL));

    /* write to read-only object */
    uint8_t dnRo[] = {NODE_ID, 0x01, 0x03, 0x00, 0x01, 0x21, 0x05, 1, 9};
    rsp = usdoRequest(CLIENT_ID, dnRo, sizeof(dnRo));
    CHECK((rsp != NULL) && (abortCodeOf(rsp) == 0x06010002UL));
    CHECK(x2101 == 7U);

    /* wrong length */
    uint8_t dnLong[] = {NODE_ID, 0x01, 0x04, 0x00, 0x00, 0x10, 0x07, 5, 1, 2, 3, 4, 5};
    rsp = usdoRequest(CLIENT_ID, dnLong, sizeof(dnLong));
    CHECK((rsp != NULL) && (abortCodeOf(rsp) == 0x06070012UL));
    uint8_t dnShort[] = {NODE_ID, 0x01, 0x05, 0x00, 0x00, 0x10, 0x07, 2, 1, 2};
    rsp = usdoRequest(CLIENT_ID, dnShort, sizeof(dnShort));
    CHECK((rsp != NULL) && (abortCodeOf(rsp) == 0x06070013UL));

    /* unsupported command specifiers: remote access and bulk transfer */
    uint8_t remote[] = {NODE_ID, 0x91, 0x06, 0x00, 0x00, 0x10};
    rsp = usdoRequest(CLIENT_ID, remote, sizeof(remote));
    CHECK((rsp != NULL) && (abortCodeOf(rsp) == 0x05040001UL));
    uint8_t bulk[] = {NODE_ID, 0x06, 0x07, 0x00, 0x00, 0x21, 0x0F, 0, 200, 0, 0, 0};
    rsp = usdoRequest(CLIENT_ID, bulk, sizeof(bulk));
    CHECK((rsp != NULL) && (abortCodeOf(rsp) == 0x05040001UL));
}

static void
test_usdo_addressing(void) {
    printf("USDO addressing: unicast, broadcast, multicast\n");
    usdoInit();
    uint8_t up[] = {0, 0x11, 0x10, 0x00, 0x00, 0x10};

    /* other node: no response */
    up[0] = NODE_ID + 1U;
    CHECK(usdoRequest(CLIENT_ID, up, sizeof(up)) == NULL);

    /* broadcast */
    up[0] = 0x00;
    test_frame_t* rsp = usdoRequest(CLIENT_ID, up, sizeof(up));
    CHECK((rsp != NULL) && (rsp->data[CO_USDO_POS_CS] == CO_USDO_CS_UP_EXP_RSP) && (rsp->data[0] == CLIENT_ID));

    /* response goes to the requesting client */
    up[0] = NODE_ID;
    rsp = usdoRequest(0x7FU, up, sizeof(up));
    CHECK((rsp != NULL) && (rsp->data[CO_USDO_POS_DA] == 0x7FU));

    /* multicast group 85h: not member, then member, then left */
    up[0] = 0x85;
    CHECK(usdoRequest(CLIENT_ID, up, sizeof(up)) == NULL);
    CHECK(CO_USDOserver_setMulticast(&usdo, 0x85, true) == CO_ERROR_NO);
    CHECK(usdoRequest(CLIENT_ID, up, sizeof(up)) != NULL);
    up[0] = 0xFF;
    CHECK(usdoRequest(CLIENT_ID, up, sizeof(up)) == NULL);
    CHECK(CO_USDOserver_setMulticast(&usdo, 0x85, false) == CO_ERROR_NO);
    up[0] = 0x85;
    CHECK(usdoRequest(CLIENT_ID, up, sizeof(up)) == NULL);
    CHECK(CO_USDOserver_setMulticast(&usdo, 0x10, true) == CO_ERROR_ILLEGAL_ARGUMENT);

    /* no response in stopped state */
    up[0] = NODE_ID;
    test_txClear();
    uint8_t upFrame[8] = {0};
    (void)memcpy(upFrame, up, sizeof(up));
    (void)test_CANrx(&can, 0x600U + CLIENT_ID, upFrame, sizeof(upFrame));
    (void)CO_USDOserver_process(&usdo, false, 1000, NULL);
    CHECK(test_txCount == 0);

    /* unconfigured node-ID: no response */
    uint32_t errInfo;
    CHECK(CO_USDOserver_init(&usdo, &od, 0xFF, 100, &can, RX_IDX_USDO, &can, TX_IDX_USDO, &errInfo) == CO_ERROR_NO);
    up[0] = 0x00;
    CHECK(usdoRequest(CLIENT_ID, up, sizeof(up)) == NULL);
}

static void
test_usdo_segmented(void) {
    printf("USDO segmented download and upload (200 bytes)\n");
    usdoInit();
    uint8_t pattern[200];
    for (unsigned i = 0; i < sizeof(pattern); i++) {
        pattern[i] = (uint8_t)(i * 7U + 1U);
    }
    (void)memset(x2100, 0, sizeof(x2100));

    /* download initiate */
    uint8_t init[] = {NODE_ID, 0x02, 0x21, 0x00, 0x00, 0x21, 0x0F, 0, 200, 0, 0, 0};
    test_frame_t* rsp = usdoRequest(CLIENT_ID, init, sizeof(init));
    CHECK((rsp != NULL) && (rsp->data[CO_USDO_POS_CS] == CO_USDO_CS_DOWN_SEG_INIT_RSP) && (rsp->len == 12U));

    /* segments of 59 bytes */
    unsigned offset = 0;
    uint8_t seq = 0;
    while (offset < sizeof(pattern)) {
        unsigned n = sizeof(pattern) - offset;
        if (n > CO_USDO_SEG_MAX_DATA) {
            n = CO_USDO_SEG_MAX_DATA;
        }
        bool_t last = (offset + n) == sizeof(pattern);
        uint8_t seg[64] = {NODE_ID, last ? 0x04 : 0x03, 0x21, seq, (uint8_t)n};
        (void)memcpy(&seg[5], &pattern[offset], n);
        rsp = usdoRequest(CLIENT_ID, seg, (uint8_t)(5U + n));
        CHECK(rsp != NULL);
        if (rsp != NULL) {
            CHECK(rsp->data[CO_USDO_POS_CS] == (last ? CO_USDO_CS_DOWN_SEG_LAST_RSP : CO_USDO_CS_DOWN_SEG_RSP));
            CHECK(rsp->data[CO_USDO_POS_SEQ] == seq);
            CHECK(rsp->len == CO_USDO_LEN_SEG_CONFIRM);
        }
        offset += n;
        seq++;
    }
    CHECK(memcmp(x2100, pattern, sizeof(pattern)) == 0);
    CHECK(usdo.state == CO_USDO_ST_IDLE);

    /* upload */
    uint8_t up[] = {NODE_ID, 0x11, 0x22, 0x00, 0x00, 0x21};
    rsp = usdoRequest(CLIENT_ID, up, sizeof(up));
    CHECK((rsp != NULL) && (rsp->data[CO_USDO_POS_CS] == CO_USDO_CS_UP_SEG_INIT_RSP));
    if (rsp != NULL) {
        uint32_t size;
        (void)memcpy(&size, &rsp->data[CO_USDO_POS_SEG_SIZE], 4);
        CHECK(size == 200U);
        CHECK(rsp->data[CO_USDO_POS_DT] == 0x0F);
    }
    uint8_t received[256];
    unsigned rxCount = 0;
    seq = 0;
    for (int guard = 0; guard < 10; guard++) {
        uint8_t req[] = {NODE_ID, 0x13, 0x22, seq};
        rsp = usdoRequest(CLIENT_ID, req, sizeof(req));
        CHECK(rsp != NULL);
        if (rsp == NULL) {
            break;
        }
        CHECK(rsp->data[CO_USDO_POS_SEQ] == seq);
        uint8_t n = rsp->data[CO_USDO_POS_SEGDATA_SIZE];
        CHECK(rsp->len == CO_CANfd_roundLength((uint8_t)(5U + n)));
        (void)memcpy(&received[rxCount], &rsp->data[CO_USDO_POS_SEGDATA], n);
        rxCount += n;
        seq++;
        if (rsp->data[CO_USDO_POS_CS] == CO_USDO_CS_UP_SEG_LAST_RSP) {
            break;
        }
        CHECK(rsp->data[CO_USDO_POS_CS] == CO_USDO_CS_UP_SEG_RSP);
    }
    CHECK(rxCount == 200U);
    CHECK(memcmp(received, pattern, 200) == 0);
    CHECK(usdo.state == CO_USDO_ST_IDLE);
}

static void
test_usdo_segmented_errors(void) {
    printf("USDO segmented: sequence error, timeout, client abort, other client\n");
    usdoInit();
    uint8_t init[] = {NODE_ID, 0x02, 0x31, 0x00, 0x00, 0x21, 0x0F, 0, 200, 0, 0, 0};

    /* wrong sequence number */
    CHECK(usdoRequest(CLIENT_ID, init, sizeof(init)) != NULL);
    uint8_t seg[64] = {NODE_ID, 0x03, 0x31, 1, 10};
    test_frame_t* rsp = usdoRequest(CLIENT_ID, seg, 15);
    CHECK((rsp != NULL) && (rsp->data[CO_USDO_POS_CS] == CO_USDO_CS_ABORT) && (abortCodeOf(rsp) == 0x05040003UL));
    CHECK(usdo.state == CO_USDO_ST_IDLE);

    /* indicated size too large */
    uint8_t initLong[] = {NODE_ID, 0x02, 0x32, 0x00, 0x00, 0x21, 0x0F, 0, 201, 0, 0, 0};
    rsp = usdoRequest(CLIENT_ID, initLong, sizeof(initLong));
    CHECK((rsp != NULL) && (abortCodeOf(rsp) == 0x06070012UL));

    /* timeout */
    CHECK(usdoRequest(CLIENT_ID, init, sizeof(init)) != NULL);
    test_txClear();
    uint32_t timerNext = 1000000;
    (void)CO_USDOserver_process(&usdo, true, 50000, &timerNext);
    CHECK(test_txCount == 0);
    CHECK(timerNext == 50000U);
    (void)CO_USDOserver_process(&usdo, true, 50000, NULL);
    CHECK((test_txCount == 1) && (abortCodeOf(&test_txLog[0]) == 0x05040000UL));
    CHECK(usdo.state == CO_USDO_ST_IDLE);

    /* request from other client is ignored during transfer; abort from client ends transfer */
    CHECK(usdoRequest(CLIENT_ID, init, sizeof(init)) != NULL);
    uint8_t up[] = {NODE_ID, 0x11, 0x40, 0x00, 0x00, 0x10};
    CHECK(usdoRequest(CLIENT_ID + 1U, up, sizeof(up)) == NULL);
    uint8_t ab[] = {NODE_ID, 0x7F, 0x31, 0x00, 0x00, 0x21, 0, 0, 0, 0x08};
    CHECK(usdoRequest(CLIENT_ID, ab, sizeof(ab)) == NULL);
    CHECK(usdo.state == CO_USDO_ST_IDLE);
    CHECK(usdoRequest(CLIENT_ID + 1U, up, sizeof(up)) != NULL);
}
#endif /* CO_CONFIG_FD */

int
main(void) {
#if ((CO_CONFIG_FD)&CO_CONFIG_FD_ENABLE) != 0
    printf("CANopenNode tests, CANopen FD build\n");
    test_canfd_lengths();
    test_pdo(1);
    test_pdo(2);
    test_pdo(3);
    test_pdo(5);
    test_pdo(7);
    test_pdo(10);
    test_pdo(16);
    test_usdo_expedited();
    test_usdo_errors();
    test_usdo_addressing();
    test_usdo_segmented();
    test_usdo_segmented_errors();
#else
    printf("CANopenNode tests, classic CAN build\n");
    test_pdo(1);
    test_pdo(2);
#endif
    printf("%d checks, %d failures\n", checks, failures);
    return (failures == 0) ? 0 : 1;
}
