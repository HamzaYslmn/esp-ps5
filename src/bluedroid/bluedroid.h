/* bluedroid.h - hand-declared Bluedroid internals (L2CAP / BTM / osi) that ESP-IDF
 * doesn't expose to Arduino libraries. They link against the Bluedroid blob in the core.
 * Verified vs ESP-IDF v5.5.2 (core 3.3.x) and v6.1.1 (core 4.0.0), files:
 *   components/bt/host/bluedroid/stack/include/stack/{bt_types,l2c_api,l2cdefs,btm_api}.h
 *   components/bt/common/osi/include/osi/allocator.h */
#ifndef PS5_BLUEDROID_H
#define PS5_BLUEDROID_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t BD_ADDR[6];
typedef struct { uint16_t event, len, offset, layer_specific; uint8_t data[]; } BT_HDR;

#define BT_PSM_HIDC        0x0011
#define BT_PSM_HIDI        0x0013
#define L2CAP_CONN_OK      0
#define L2CAP_CONN_PENDING 1
#define L2CAP_CFG_OK       0
#define L2CAP_DW_FAILED    0
#define L2CAP_MIN_OFFSET   13
#define BTM_SEC_SERVICE_FIRST_EMPTY 54

/* Only `result` is touched; padding covers the upstream layout (< 128 B). */
typedef struct { uint16_t result; uint8_t _opaque[126]; } tL2CAP_CFG_INFO;

typedef struct {
  void (*connect_ind)(BD_ADDR, uint16_t cid, uint16_t psm, uint8_t id);
  void (*connect_cfm)(uint16_t cid, uint16_t result);
  void (*connect_pnd)(uint16_t cid);
  void (*config_ind)(uint16_t cid, tL2CAP_CFG_INFO*);
  void (*config_cfm)(uint16_t cid, tL2CAP_CFG_INFO*);
  void (*disconnect_ind)(uint16_t cid, bool ack_needed);
  void (*disconnect_cfm)(uint16_t cid, uint16_t result);
  void (*qos_violation_ind)(BD_ADDR);
  void (*data_ind)(uint16_t cid, BT_HDR*);
  void (*congestion)(uint16_t cid, bool congested);
  void (*tx_complete)(uint16_t cid, uint16_t count);
  void* le_reconfig_ind;   /* IDF 6.x with enhanced CoC; unused/ignored on 5.x */
} tL2CAP_APPL_INFO;

uint16_t L2CA_Register(uint16_t psm, tL2CAP_APPL_INFO* p_cb_info);
uint16_t L2CA_ErtmConnectReq(uint16_t psm, BD_ADDR bd_addr, void* p_ertm_info);
bool     L2CA_ErtmConnectRsp(BD_ADDR bd_addr, uint8_t id, uint16_t cid, uint16_t result, uint16_t status, void* p_ertm_info);
bool     L2CA_ConfigReq(uint16_t cid, tL2CAP_CFG_INFO* p_cfg);
bool     L2CA_ConfigRsp(uint16_t cid, tL2CAP_CFG_INFO* p_cfg);
bool     L2CA_DisconnectRsp(uint16_t cid);
uint8_t  L2CA_DataWrite(uint16_t cid, BT_HDR* p_data);
bool     BTM_SetSecurityLevel(bool is_originator, const char* p_name, uint8_t service_id,
                              uint16_t sec_level, uint16_t psm, uint32_t mx_proto_id, uint32_t mx_chan_id);
uint8_t  BTM_SetConnectability(uint16_t page_mode, uint16_t window, uint16_t interval);  /* btm_api.h */
uint8_t  BTM_SetPageScanType(uint16_t scan_type);
void*    osi_malloc_func(size_t size);
void     osi_free_func(void* p);

#ifdef __cplusplus
}
#endif

#endif
