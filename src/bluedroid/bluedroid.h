/* bluedroid.h - minimal Bluedroid internal API shim.
 *
 * Declares only the symbols ps5Controller.cpp actually uses, so we can talk
 * directly to the precompiled Bluedroid blob shipped with Arduino-ESP32
 * without dragging in ESP-IDF's full header tree (which transitively pulls
 * in sdkconfig.h, bt_user_config.h, etc., which aren't available to user
 * libraries).
 *
 * The precompiled blob exports these functions with C linkage. Their
 * implementations live inside the Arduino-ESP32 BT library; we just need
 * matching declarations to link against.
 *
 * If a future Arduino-ESP32 / ESP-IDF release renames or changes the
 * signature of any symbol below, the *only* place to patch is this file.
 *
 * Verified against ESP-IDF v5.5.2 (matches Arduino-ESP32 v3.3.6).
 * Upstream paths in espressif/esp-idf:
 *   stack symbols  -> components/bt/host/bluedroid/stack/include/stack/{bt_types,l2c_api,l2cdefs,btm_api}.h
 *   osi symbols    -> components/bt/common/osi/include/osi/allocator.h
 */

#ifndef PS5_BLUEDROID_H
#define PS5_BLUEDROID_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- bt_types.h ---- */

typedef uint8_t  UINT8;
typedef uint16_t UINT16;
typedef uint32_t UINT32;
typedef bool     BOOLEAN;
typedef uint8_t  BD_ADDR[6];

/* L2CAP buffer header. The actual payload starts at `data + offset`. */
typedef struct {
    uint16_t event;
    uint16_t length;
    uint16_t offset;
    uint16_t layer_specific;
    uint8_t  data[];
} BT_HDR;

#define BT_DEFAULT_BUFFER_SIZE (4096 + 16)

/* HID PSMs - L2CAP-level. */
#define BT_PSM_HIDC 0x0011  /* HID Control */
#define BT_PSM_HIDI 0x0013  /* HID Interrupt */

/* ---- l2cdefs.h ---- */

#define L2CAP_CONN_OK       0
#define L2CAP_CONN_PENDING  1
#define L2CAP_CFG_OK        0
#define L2CAP_DW_SUCCESS    1     /* TRUE */
#define L2CAP_DW_CONGESTED  2

/* ---- l2c_api.h ---- */

#define L2CAP_MIN_OFFSET 13   /* L2CAP header reserve in BT_HDR */

/* tL2CAP_CFG_INFO is large in upstream; we never inspect or mutate any
 * field except `result` from inside config_ind_cback, but the struct must
 * be the same size the blob expects. The simplest stable approach: use a
 * plain byte buffer big enough for any historical layout (~96 B is the
 * upstream maximum), and only treat the first uint16_t as `result`. The
 * blob writes the rest; we leave it untouched. */
typedef struct {
    uint16_t result;        /* L2CAP_CFG_OK etc. */
    uint8_t  _opaque[126];  /* room for mtu/qos/fcr/fcs/ext-flow/... */
} tL2CAP_CFG_INFO;

/* tL2CAP_APPL_INFO callback typedefs - signatures must match the blob. */
typedef void (tL2CA_CONNECT_IND_CB)    (BD_ADDR, uint16_t cid, uint16_t psm, uint8_t id);
typedef void (tL2CA_CONNECT_CFM_CB)    (uint16_t cid, uint16_t result);
typedef void (tL2CA_CONNECT_PND_CB)    (uint16_t cid);
typedef void (tL2CA_CONFIG_IND_CB)     (uint16_t cid, tL2CAP_CFG_INFO*);
typedef void (tL2CA_CONFIG_CFM_CB)     (uint16_t cid, tL2CAP_CFG_INFO*);
typedef void (tL2CA_DISCONNECT_IND_CB) (uint16_t cid, bool ack_needed);
typedef void (tL2CA_DISCONNECT_CFM_CB) (uint16_t cid, uint16_t result);
typedef void (tL2CA_QOS_VIOLATION_IND_CB)(BD_ADDR);
typedef void (tL2CA_DATA_IND_CB)       (uint16_t cid, BT_HDR*);
typedef void (tL2CA_CONGESTION_STATUS_CB)(uint16_t cid, bool congested);
typedef void (tL2CA_TX_COMPLETE_CB)    (uint16_t cid, uint16_t count);

typedef struct {
    tL2CA_CONNECT_IND_CB        *pL2CA_ConnectInd_Cb;
    tL2CA_CONNECT_CFM_CB        *pL2CA_ConnectCfm_Cb;
    tL2CA_CONNECT_PND_CB        *pL2CA_ConnectPnd_Cb;
    tL2CA_CONFIG_IND_CB         *pL2CA_ConfigInd_Cb;
    tL2CA_CONFIG_CFM_CB         *pL2CA_ConfigCfm_Cb;
    tL2CA_DISCONNECT_IND_CB     *pL2CA_DisconnectInd_Cb;
    tL2CA_DISCONNECT_CFM_CB     *pL2CA_DisconnectCfm_Cb;
    tL2CA_QOS_VIOLATION_IND_CB  *pL2CA_QoSViolationInd_Cb;
    tL2CA_DATA_IND_CB           *pL2CA_DataInd_Cb;
    tL2CA_CONGESTION_STATUS_CB  *pL2CA_CongestionStatus_Cb;
    tL2CA_TX_COMPLETE_CB        *pL2CA_TxComplete_Cb;
} tL2CAP_APPL_INFO;

/* L2CAP function exports (signatures match upstream l2c_api.h). */
extern uint16_t L2CA_Register      (uint16_t psm, tL2CAP_APPL_INFO* p_cb_info);
extern void     L2CA_Deregister    (uint16_t psm);
extern uint16_t L2CA_ErtmConnectReq(uint16_t psm, BD_ADDR bd_addr, void* p_ertm_info);
extern bool     L2CA_ErtmConnectRsp(BD_ADDR bd_addr, uint8_t id, uint16_t cid,
                                    uint16_t result, uint16_t status, void* p_ertm_info);
extern bool     L2CA_ConfigReq     (uint16_t cid, tL2CAP_CFG_INFO* p_cfg);
extern bool     L2CA_ConfigRsp     (uint16_t cid, tL2CAP_CFG_INFO* p_cfg);
extern bool     L2CA_DisconnectReq (uint16_t cid);
extern bool     L2CA_DisconnectRsp (uint16_t cid);
extern uint8_t  L2CA_DataWrite     (uint16_t cid, BT_HDR* p_data);

/* Convenience macros mirroring upstream l2c_api.h. */
#define L2CA_CONNECT_REQ(psm,bd,ertm,sec)            L2CA_ErtmConnectReq((psm),(bd),(ertm))
#define L2CA_CONNECT_RSP(bd,id,cid,res,sta,ertm,sec) L2CA_ErtmConnectRsp((bd),(id),(cid),(res),(sta),(ertm))
#define L2CA_CONFIG_REQ(cid,cfg)                     L2CA_ConfigReq((cid),(cfg))

/* ---- btm_api.h ---- */

#define BTM_SEC_SERVICE_FIRST_EMPTY 54

extern bool BTM_SetSecurityLevel(bool is_originator, const char* p_name,
                                 uint8_t service_id, uint16_t mx_sec_flags,
                                 uint16_t psm, uint32_t mx_proto_id,
                                 uint32_t mx_chan_id);

/* ---- osi/allocator.h ---- */

extern void* osi_malloc_func(size_t size);
extern void  osi_free_func  (void* p);

#define osi_malloc(size) osi_malloc_func((size))
#define osi_free(p)      osi_free_func((p))

#ifdef __cplusplus
}
#endif

#endif /* PS5_BLUEDROID_H */
