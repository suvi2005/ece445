#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "soc/clk_tree_defs.h"

#include <stdlib.h>

#include "host/ble_gatt.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "store/config/ble_store_config.h"

#define I2C_BUS_1_SDA 2
#define I2C_BUS_1_SCL 1

#define I2C_BUS_2_SDA 41
#define I2C_BUS_2_SCL 42

#define X_SHUT_TOF_2 45
#define X_SHUT_TOF_3 40

#define AIN1 46
#define BIN1 21
#define AIN2 14
#define BIN2 38
#define PWMA 47
#define PWMB 48

#define I2C_BUS_1 I2C_NUM_0
#define I2C_BUS_2 I2C_NUM_1
#define I2C_FREQ_HZ 400000
#define I2C_TIMEOUT_MS 100

#define VL53L0X_DEFAULT_ADDR 0x29
#define VL53L0X_SENSOR_2_ADDR 0x30
#define VL53L0X_SENSOR_3_ADDR 0x31

#define REG_SYSRANGE_START 0x00
#define REG_SYSTEM_INTERRUPT_CLEAR 0x0B
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO 0x0A
#define REG_RESULT_INTERRUPT_STATUS 0x13
#define REG_RESULT_RANGE_STATUS 0x14
#define REG_GPIO_HV_MUX_ACTIVE_HIGH 0x84
#define REG_I2C_SLAVE_DEVICE_ADDRESS 0x8A
#define REG_IDENTIFICATION_MODEL_ID 0xC0

#define RANGE_READ_TIMEOUT_MS 100

#define OBSTACLE_STOP_MM 250
#define MOTOR_SPEED_DUTY 200

#define CMD_TEXT_MAX_LEN 32
#define INVALID_HANDLE 0

#define RETURN_ON_ERROR(expr, log_tag, message)                                \
  do {                                                                         \
    esp_err_t __err_rc = (expr);                                               \
    if (__err_rc != ESP_OK) {                                                  \
      ESP_LOGE((log_tag), "%s: %s", (message), esp_err_to_name(__err_rc));     \
      return __err_rc;                                                         \
    }                                                                          \
  } while (0)

static const char *TAG = "BT_ROBOT_CTRL";
void ble_store_config_init(void);

typedef enum {
  MOTION_STOP = 0,
  MOTION_FORWARD,
  MOTION_BACKWARD,
  MOTION_LEFT,
  MOTION_RIGHT,
} motion_cmd_t;

static void set_motion_command(motion_cmd_t cmd);
static void set_motion_inhibit(bool inhibit);
static void motor_stop(void);

typedef struct {
  const char *name;
  i2c_master_dev_handle_t handle;
  uint8_t address;
} tof_sensor_t;

static i2c_master_bus_handle_t s_bus_1 = NULL;
static i2c_master_bus_handle_t s_bus_2 = NULL;

static SemaphoreHandle_t s_state_mutex = NULL;
static motion_cmd_t s_motion_cmd = MOTION_STOP;
static uint16_t s_lidar_mm[3] = {0};
static bool s_lidar_valid[3] = {false, false, false};
static volatile bool s_motion_inhibit = true;
static TaskHandle_t s_motor_task_handle = NULL;

typedef struct {
  char text[CMD_TEXT_MAX_LEN];
} bt_rx_msg_t;

static QueueHandle_t s_bt_rx_queue = NULL;
static volatile bool s_ble_command_link_ready = false;

static const char s_ble_target_name[] = "ESP32S3_GLOVE";

static const ble_uuid128_t s_ble_service_uuid = BLE_UUID128_INIT(
    0xab, 0x90, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12,
    0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static const ble_uuid128_t s_ble_char_uuid = BLE_UUID128_INIT(
    0xef, 0xcd, 0xab, 0xef, 0xcd, 0xab, 0x34, 0x12,
    0x34, 0x12, 0x34, 0x12, 0xab, 0xef, 0xcd, 0xab);

typedef struct {
  uint16_t conn_handle;
  uint16_t service_start_handle;
  uint16_t service_end_handle;
  uint16_t char_def_handle;
  uint16_t char_val_handle;
  uint16_t cccd_handle;
  bool service_found;
  bool char_found;
  bool connecting;
  bool subscribed;
} ble_client_state_t;

static ble_client_state_t s_ble_client = {
    .conn_handle = BLE_HS_CONN_HANDLE_NONE,
};

static void ble_scan_start(void);
static const char *ble_addr_to_str(const ble_addr_t *addr);
static void ble_clear_pending_commands(void);
static void ble_queue_received_bytes(const uint8_t *data, size_t length);
static void ble_handle_connection_lost(void);
static void ble_reset_client_state(void);
static void ble_start_service_discovery(uint16_t conn_handle);
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static int ble_service_disc_cb(uint16_t conn_handle,
                               const struct ble_gatt_error *error,
                               const struct ble_gatt_svc *service, void *arg);
static int ble_char_disc_cb(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_chr *chr, void *arg);
static int ble_dsc_disc_cb(uint16_t conn_handle,
                           const struct ble_gatt_error *error,
                           uint16_t chr_val_handle,
                           const struct ble_gatt_dsc *dsc, void *arg);
static int ble_subscribe_write_cb(uint16_t conn_handle,
                                  const struct ble_gatt_error *error,
                                  struct ble_gatt_attr *attr, void *arg);
static int ble_mtu_event_cb(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            uint16_t mtu, void *arg);
static void ble_host_task(void *param);
static void ble_on_reset(int reason);
static void ble_on_sync(void);

static void ble_queue_received_bytes(const uint8_t *data, size_t length) {
  if (data == NULL || length == 0 || s_bt_rx_queue == NULL) {
    return;
  }

  bt_rx_msg_t msg = {0};
  size_t copy_len = length;
  if (copy_len >= sizeof(msg.text)) {
    copy_len = sizeof(msg.text) - 1;
  }

  memcpy(msg.text, data, copy_len);
  msg.text[copy_len] = '\0';
  xQueueSend(s_bt_rx_queue, &msg, 0);
}

static void ble_clear_pending_commands(void) {
  if (s_bt_rx_queue == NULL) {
    return;
  }

  bt_rx_msg_t discarded = {0};
  while (xQueueReceive(s_bt_rx_queue, &discarded, 0) == pdTRUE) {
  }
}

static void ble_handle_connection_lost(void) {
  s_ble_command_link_ready = false;
  ble_clear_pending_commands();
  set_motion_inhibit(true);
  set_motion_command(MOTION_STOP);
  motor_stop();
  if (s_motor_task_handle != NULL) {
    xTaskNotifyGive(s_motor_task_handle);
  }
  ESP_LOGW(TAG, "Robot stopped after BLE link loss; restarting discovery");
  ble_reset_client_state();
  ble_scan_start();
}

static void ble_reset_client_state(void) {
  s_ble_client.conn_handle = BLE_HS_CONN_HANDLE_NONE;
  s_ble_client.service_start_handle = 0;
  s_ble_client.service_end_handle = 0;
  s_ble_client.char_def_handle = 0;
  s_ble_client.char_val_handle = 0;
  s_ble_client.cccd_handle = 0;
  s_ble_client.service_found = false;
  s_ble_client.char_found = false;
  s_ble_client.connecting = false;
  s_ble_client.subscribed = false;
}

static const char *ble_addr_to_str(const ble_addr_t *addr) {
  static char buffer[18];

  if (addr == NULL) {
    return "<null>";
  }

  snprintf(buffer, sizeof(buffer), "%02x:%02x:%02x:%02x:%02x:%02x",
           addr->val[5], addr->val[4], addr->val[3], addr->val[2],
           addr->val[1], addr->val[0]);
  return buffer;
}

static bool ble_adv_name_matches(const struct ble_hs_adv_fields *fields) {
  if (fields == NULL) {
    return false;
  }

  if (fields->name != NULL && fields->name_len == strlen(s_ble_target_name) &&
      memcmp(fields->name, s_ble_target_name, fields->name_len) == 0) {
    return true;
  }

  if (fields->name_is_complete == 0 && fields->name != NULL) {
    size_t target_len = strlen(s_ble_target_name);
    size_t compare_len = fields->name_len;
    if (compare_len > target_len) {
      compare_len = target_len;
    }
    return compare_len > 0 &&
           memcmp(fields->name, s_ble_target_name, compare_len) == 0;
  }

  return false;
}

static void ble_scan_start(void) {
  uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
  struct ble_gap_disc_params disc_params = {0};
  int rc = ble_hs_id_infer_auto(0, &own_addr_type);
  if (rc != 0) {
    ESP_LOGE(TAG, "BLE infer address type failed: %d", rc);
    return;
  }

  disc_params.filter_duplicates = 1;
  disc_params.passive = 0;
  disc_params.itvl = 0x50;
  disc_params.window = 0x30;

  rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                    ble_gap_event, NULL);
  if (rc != 0 && rc != BLE_HS_EALREADY) {
    ESP_LOGE(TAG, "BLE scan start failed: %d", rc);
  } else {
    ESP_LOGI(TAG, "BLE scan started");
  }
}

static void ble_start_service_discovery(uint16_t conn_handle) {
  int rc = ble_gattc_disc_svc_by_uuid(conn_handle, &s_ble_service_uuid.u,
                                      ble_service_disc_cb, NULL);
  if (rc != 0) {
    ESP_LOGE(TAG, "Service discovery start failed: %d", rc);
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
  }
}

static int ble_service_disc_cb(uint16_t conn_handle,
                               const struct ble_gatt_error *error,
                               const struct ble_gatt_svc *service, void *arg) {
  (void)arg;

  if (error->status == 0 && service != NULL) {
    s_ble_client.service_found = true;
    s_ble_client.service_start_handle = service->start_handle;
    s_ble_client.service_end_handle = service->end_handle;
    ESP_LOGI(TAG, "Target service found: start=%u end=%u",
             s_ble_client.service_start_handle, s_ble_client.service_end_handle);
    return 0;
  }

  if (error->status == BLE_HS_EDONE) {
    int rc;

    if (!s_ble_client.service_found) {
      ESP_LOGE(TAG, "Target service not found");
      ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
      return 0;
    }

    rc = ble_gattc_disc_chrs_by_uuid(conn_handle,
                                     s_ble_client.service_start_handle,
                                     s_ble_client.service_end_handle,
                                     &s_ble_char_uuid.u, ble_char_disc_cb,
                                     NULL);
    if (rc != 0) {
      ESP_LOGE(TAG, "Characteristic discovery start failed: %d", rc);
      ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
  }

  ESP_LOGE(TAG, "Service discovery failed: %d", error->status);
  ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
  return 0;
}

static int ble_char_disc_cb(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_chr *chr, void *arg) {
  (void)arg;

  if (error->status == 0 && chr != NULL) {
    s_ble_client.char_found = true;
    s_ble_client.char_def_handle = chr->def_handle;
    s_ble_client.char_val_handle = chr->val_handle;
    ESP_LOGI(TAG, "Target characteristic found: def=%u val=%u props=0x%02x",
             s_ble_client.char_def_handle, s_ble_client.char_val_handle,
             chr->properties);
    return 0;
  }

  if (error->status == BLE_HS_EDONE) {
    int rc;

    if (!s_ble_client.char_found) {
      ESP_LOGE(TAG, "Target characteristic not found");
      ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
      return 0;
    }

    rc = ble_gattc_disc_all_dscs(conn_handle, s_ble_client.char_def_handle,
                                 s_ble_client.service_end_handle,
                                 ble_dsc_disc_cb, NULL);
    if (rc != 0) {
      ESP_LOGE(TAG, "Descriptor discovery start failed: %d", rc);
      ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
  }

  ESP_LOGE(TAG, "Characteristic discovery failed: %d", error->status);
  ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
  return 0;
}

static int ble_dsc_disc_cb(uint16_t conn_handle,
                           const struct ble_gatt_error *error,
                           uint16_t chr_val_handle,
                           const struct ble_gatt_dsc *dsc, void *arg) {
  (void)arg;
  (void)chr_val_handle;

  if (error->status == 0 && dsc != NULL) {
    if (ble_uuid_u16(&dsc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16) {
      s_ble_client.cccd_handle = dsc->handle;
      ESP_LOGI(TAG, "Found CCCD handle: %u", s_ble_client.cccd_handle);
    }
    return 0;
  }

  if (error->status == BLE_HS_EDONE) {
    uint8_t notify_en[2] = {1, 0};
    int rc;

    if (s_ble_client.cccd_handle == 0) {
      ESP_LOGE(TAG, "Notify CCCD not found");
      ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
      return 0;
    }

    rc = ble_gattc_write_flat(conn_handle, s_ble_client.cccd_handle, notify_en,
                              sizeof(notify_en), ble_subscribe_write_cb, NULL);
    if (rc != 0) {
      ESP_LOGE(TAG, "CCCD write failed to start: %d", rc);
      ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
  }

  ESP_LOGE(TAG, "Descriptor discovery failed: %d", error->status);
  ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
  return 0;
}

static int ble_subscribe_write_cb(uint16_t conn_handle,
                                  const struct ble_gatt_error *error,
                                  struct ble_gatt_attr *attr, void *arg) {
  (void)conn_handle;
  (void)arg;

  if (error->status != 0) {
    ESP_LOGE(TAG, "Notification subscribe failed: %d", error->status);
    if (s_ble_client.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
      ble_gap_terminate(s_ble_client.conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    return 0;
  }

  s_ble_client.subscribed = true;
  s_ble_command_link_ready = true;
  ESP_LOGI(TAG, "Notification subscription requested on handle %u",
           attr != NULL ? attr->handle : 0);
  return 0;
}

static int ble_mtu_event_cb(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            uint16_t mtu, void *arg) {
  (void)conn_handle;
  (void)arg;

  if (error->status == 0) {
    ESP_LOGI(TAG, "MTU configured: %u", mtu);
  } else {
    ESP_LOGW(TAG, "MTU exchange failed: %d", error->status);
  }
  return 0;
}

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
  (void)arg;
  int rc;

  switch (event->type) {
  case BLE_GAP_EVENT_DISC: {
    struct ble_hs_adv_fields fields;
    char adv_name[32] = {0};
    bool name_match = false;

    if (s_ble_client.connecting) {
      return 0;
    }

    rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                 event->disc.length_data);
    if (rc != 0) {
      ESP_LOGW(TAG, "Failed to parse advertisement from %s, event_type=%u rc=%d",
               ble_addr_to_str(&event->disc.addr), event->disc.event_type, rc);
      return 0;
    }

    if (fields.name != NULL) {
      size_t copy_len = fields.name_len;
      if (copy_len >= sizeof(adv_name)) {
        copy_len = sizeof(adv_name) - 1;
      }
      memcpy(adv_name, fields.name, copy_len);
      adv_name[copy_len] = '\0';
    }

    name_match = ble_adv_name_matches(&fields);

    ESP_LOGI(TAG,
             "BLE adv: addr=%s type=%u rssi=%d name='%s'%s",
             ble_addr_to_str(&event->disc.addr), event->disc.event_type,
             event->disc.rssi, adv_name[0] != '\0' ? adv_name : "<none>",
             name_match ? " [target]" : "");

    if (name_match) {
      uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;

      ESP_LOGI(TAG, "Found target BLE transmitter: %s at %s", s_ble_target_name,
               ble_addr_to_str(&event->disc.addr));
      s_ble_client.connecting = true;

      rc = ble_gap_disc_cancel();
      if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "Failed to cancel scan before connect: %d", rc);
      }

      rc = ble_hs_id_infer_auto(0, &own_addr_type);
      if (rc != 0) {
        ESP_LOGE(TAG, "BLE infer address type failed: %d", rc);
        s_ble_command_link_ready = false;
        ble_clear_pending_commands();
        set_motion_command(MOTION_STOP);
        motor_stop();
        s_ble_client.connecting = false;
        ble_scan_start();
        return 0;
      }

      rc = ble_gap_connect(own_addr_type, &event->disc.addr, 30000, NULL,
                           ble_gap_event, NULL);
      if (rc != 0) {
        ESP_LOGE(TAG, "BLE connect failed: addr_type=%d addr=%s rc=%d",
                 event->disc.addr.type, ble_addr_to_str(&event->disc.addr), rc);
        s_ble_command_link_ready = false;
        ble_clear_pending_commands();
        set_motion_command(MOTION_STOP);
        motor_stop();
        s_ble_client.connecting = false;
        ble_scan_start();
      }
    }
    return 0;
  }

  case BLE_GAP_EVENT_CONNECT:
    if (event->connect.status != 0) {
      ESP_LOGE(TAG, "BLE connection failed: %d", event->connect.status);
      s_ble_command_link_ready = false;
      ble_clear_pending_commands();
      set_motion_command(MOTION_STOP);
      motor_stop();
      ble_reset_client_state();
      ble_scan_start();
      return 0;
    }

    s_ble_client.conn_handle = event->connect.conn_handle;
    s_ble_client.connecting = false;
    ESP_LOGI(TAG, "BLE connected, conn_handle=%u", s_ble_client.conn_handle);

    rc = ble_gattc_exchange_mtu(event->connect.conn_handle, ble_mtu_event_cb,
                                NULL);
    if (rc != 0) {
      ESP_LOGW(TAG, "MTU exchange start failed: %d", rc);
    }

    ble_start_service_discovery(event->connect.conn_handle);
    return 0;

  case BLE_GAP_EVENT_DISCONNECT:
    ESP_LOGW(TAG, "BLE disconnected, reason=%d", event->disconnect.reason);
    ble_handle_connection_lost();
    return 0;

  case BLE_GAP_EVENT_NOTIFY_RX: {
    uint16_t length = OS_MBUF_PKTLEN(event->notify_rx.om);
    uint8_t buffer[CMD_TEXT_MAX_LEN] = {0};
    size_t copy_len = length;

    if (copy_len > sizeof(buffer) - 1) {
      copy_len = sizeof(buffer) - 1;
    }

    os_mbuf_copydata(event->notify_rx.om, 0, copy_len, buffer);
    ble_queue_received_bytes(buffer, copy_len);
    return 0;
  }

  case BLE_GAP_EVENT_MTU:
    ESP_LOGI(TAG, "MTU update event: conn_handle=%u mtu=%u",
             event->mtu.conn_handle, event->mtu.value);
    return 0;

  case BLE_GAP_EVENT_DISC_COMPLETE:
    ESP_LOGI(TAG, "BLE discovery procedure complete; reason=%d",
             event->disc_complete.reason);
    return 0;

  default:
    return 0;
  }
}

static void ble_on_reset(int reason) {
  ESP_LOGE(TAG, "NimBLE reset; reason=%d", reason);
}

static void ble_on_sync(void) {
  int rc = ble_hs_util_ensure_addr(0);
  if (rc != 0) {
    ESP_LOGE(TAG, "BLE ensure address failed: %d", rc);
    return;
  }

  ble_reset_client_state();
  s_ble_command_link_ready = false;
  ble_clear_pending_commands();
  ble_scan_start();
}

static void ble_host_task(void *param) {
  (void)param;
  ESP_LOGI(TAG, "NimBLE host task started");
  nimble_port_run();
  nimble_port_freertos_deinit();
}

static esp_err_t bluetooth_init(void) {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase failed");
    RETURN_ON_ERROR(nvs_flash_init(), TAG, "nvs init failed after erase");
  } else if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs init failed: %s", esp_err_to_name(err));
    return err;
  }

  RETURN_ON_ERROR(nimble_port_init(), TAG, "nimble init failed");

  ble_hs_cfg.reset_cb = ble_on_reset;
  ble_hs_cfg.sync_cb = ble_on_sync;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

#if CONFIG_BT_NIMBLE_GAP_SERVICE
  int rc = ble_svc_gap_device_name_set("bt-lidar-cent");
  if (rc != 0) {
    ESP_LOGW(TAG, "failed to set local GAP name: %d", rc);
  }
#endif

  ble_store_config_init();
  nimble_port_freertos_init(ble_host_task);
  ble_reset_client_state();

  ESP_LOGI(TAG, "BLE client initialized, scanning for %s", s_ble_target_name);
  return ESP_OK;
}

static const char *motion_cmd_to_string(motion_cmd_t cmd) {
  switch (cmd) {
  case MOTION_FORWARD:
    return "forward";
  case MOTION_BACKWARD:
    return "backward";
  case MOTION_LEFT:
    return "left";
  case MOTION_RIGHT:
    return "right";
  case MOTION_STOP:
  default:
    return "stop";
  }
}

static void set_motion_inhibit(bool inhibit) {
  s_motion_inhibit = inhibit;
}

static void set_motion_command(motion_cmd_t cmd) {
  if (s_state_mutex == NULL) {
    return;
  }

  if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    s_motion_cmd = cmd;
    xSemaphoreGive(s_state_mutex);
  }
}

static motion_cmd_t get_motion_command(void) {
  motion_cmd_t cmd = MOTION_STOP;

  if (s_state_mutex == NULL) {
    return cmd;
  }

  if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    cmd = s_motion_cmd;
    xSemaphoreGive(s_state_mutex);
  }

  return cmd;
}

static void update_lidar_reading(size_t index, uint16_t mm, bool valid) {
  if (index >= 3 || s_state_mutex == NULL) {
    return;
  }

  if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    s_lidar_mm[index] = mm;
    s_lidar_valid[index] = valid;
    xSemaphoreGive(s_state_mutex);
  }
}

static void get_lidar_snapshot(uint16_t mm[3], bool valid[3]) {
  if (s_state_mutex == NULL) {
    for (size_t i = 0; i < 3; ++i) {
      mm[i] = 0;
      valid[i] = false;
    }
    return;
  }

  if (xSemaphoreTake(s_state_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    for (size_t i = 0; i < 3; ++i) {
      mm[i] = s_lidar_mm[i];
      valid[i] = s_lidar_valid[i];
    }
    xSemaphoreGive(s_state_mutex);
  }
}

static bool __attribute__((unused)) parse_motion_command(const char *input,
                                                         motion_cmd_t *cmd) {
  if (input == NULL || cmd == NULL) {
    return false;
  }

  char cleaned[CMD_TEXT_MAX_LEN] = {0};
  size_t out = 0;
  size_t len = strnlen(input, CMD_TEXT_MAX_LEN - 1);

  while (out < len && isspace((unsigned char)input[out])) {
    out++;
  }

  size_t start = out;
  while (len > start && isspace((unsigned char)input[len - 1])) {
    len--;
  }

  size_t cleaned_len = 0;
  for (size_t i = start; i < len && cleaned_len < sizeof(cleaned) - 1; ++i) {
    cleaned[cleaned_len++] = (char)tolower((unsigned char)input[i]);
  }
  cleaned[cleaned_len] = '\0';

  if (strcmp(cleaned, "forward") == 0) {
    *cmd = MOTION_FORWARD;
    return true;
  }
  if (strcmp(cleaned, "f") == 0) {
    *cmd = MOTION_FORWARD;
    return true;
  }
  if (strcmp(cleaned, "backward") == 0) {
    *cmd = MOTION_BACKWARD;
    return true;
  }
  if (strcmp(cleaned, "b") == 0) {
    *cmd = MOTION_BACKWARD;
    return true;
  }
  if (strcmp(cleaned, "left") == 0) {
    *cmd = MOTION_LEFT;
    return true;
  }
  if (strcmp(cleaned, "l") == 0) {
    *cmd = MOTION_LEFT;
    return true;
  }
  if (strcmp(cleaned, "right") == 0) {
    *cmd = MOTION_RIGHT;
    return true;
  }
  if (strcmp(cleaned, "r") == 0) {
    *cmd = MOTION_RIGHT;
    return true;
  }
  if (strcmp(cleaned, "stop") == 0) {
    *cmd = MOTION_STOP;
    return true;
  }
  if (strcmp(cleaned, "x") == 0) {
    *cmd = MOTION_STOP;
    return true;
  }

  return false;
}

static esp_err_t motor_driver_init(void) {
  const gpio_config_t dir_cfg = {
      .pin_bit_mask =
          (1ULL << AIN1) | (1ULL << AIN2) | (1ULL << BIN1) | (1ULL << BIN2),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  RETURN_ON_ERROR(gpio_config(&dir_cfg), TAG,
                  "motor direction pin init failed");

  const ledc_timer_config_t timer_cfg = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_8_BIT,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = 20000,
      .clk_cfg = LEDC_AUTO_CLK,
      .deconfigure = false,
  };
  RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "LEDC timer init failed");

  const ledc_channel_config_t ch_a = {
      .gpio_num = PWMA,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_0,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0,
      .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
      .flags.output_invert = 0,
  };

  const ledc_channel_config_t ch_b = {
      .gpio_num = PWMB,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LEDC_CHANNEL_1,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0,
      .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
      .flags.output_invert = 0,
  };

  RETURN_ON_ERROR(ledc_channel_config(&ch_a), TAG,
                  "LEDC channel A init failed");
  RETURN_ON_ERROR(ledc_channel_config(&ch_b), TAG,
                  "LEDC channel B init failed");

  return ESP_OK;
}

static void motor_set_direction(bool left_forward, bool right_forward) {
  gpio_set_level(AIN1, left_forward ? 1 : 0);
  gpio_set_level(AIN2, left_forward ? 0 : 1);
  gpio_set_level(BIN1, right_forward ? 1 : 0);
  gpio_set_level(BIN2, right_forward ? 0 : 1);
}

static void motor_set_speed(uint32_t duty) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

static void motor_stop(void) {
  gpio_set_level(AIN1, 0);
  gpio_set_level(AIN2, 0);
  gpio_set_level(BIN1, 0);
  gpio_set_level(BIN2, 0);
  motor_set_speed(0);
}

static void motor_apply_command(motion_cmd_t cmd) {
  switch (cmd) {
  case MOTION_FORWARD:
    motor_set_direction(true, true);
    motor_set_speed(MOTOR_SPEED_DUTY);
    break;
  case MOTION_BACKWARD:
    motor_set_direction(false, false);
    motor_set_speed(MOTOR_SPEED_DUTY);
    break;
  case MOTION_LEFT:
    motor_set_direction(false, true);
    motor_set_speed(MOTOR_SPEED_DUTY);
    break;
  case MOTION_RIGHT:
    motor_set_direction(true, false);
    motor_set_speed(MOTOR_SPEED_DUTY);
    break;
  case MOTION_STOP:
  default:
    motor_stop();
    break;
  }
}

static esp_err_t i2c_bus_init(i2c_port_num_t port, gpio_num_t sda,
                              gpio_num_t scl,
                              i2c_master_bus_handle_t *bus_handle) {
  const i2c_master_bus_config_t config = {
      .i2c_port = port,
      .sda_io_num = sda,
      .scl_io_num = scl,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,
      .flags.enable_internal_pullup = 1,
      .flags.allow_pd = 0,
  };

  return i2c_new_master_bus(&config, bus_handle);
}

static esp_err_t i2c_add_device(i2c_master_bus_handle_t bus, uint8_t address,
                                i2c_master_dev_handle_t *device_handle) {
  const i2c_device_config_t config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = address,
      .scl_speed_hz = I2C_FREQ_HZ,
      .scl_wait_us = 0,
      .flags.disable_ack_check = 0,
  };

  return i2c_master_bus_add_device(bus, &config, device_handle);
}

static esp_err_t i2c_write_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg,
                               uint8_t value) {
  const uint8_t payload[] = {reg, value};
  return i2c_master_transmit(dev_handle, payload, sizeof(payload),
                             I2C_TIMEOUT_MS);
}

static esp_err_t i2c_read_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg,
                              uint8_t *value) {
  return i2c_master_transmit_receive(dev_handle, &reg, 1, value, 1,
                                     I2C_TIMEOUT_MS);
}

static esp_err_t i2c_read_regs(i2c_master_dev_handle_t dev_handle,
                               uint8_t start_reg, uint8_t *buffer,
                               size_t length) {
  return i2c_master_transmit_receive(dev_handle, &start_reg, 1, buffer, length,
                                     I2C_TIMEOUT_MS);
}

static void scan_i2c_bus(i2c_master_bus_handle_t bus_handle,
                         const char *label) {
  ESP_LOGI(TAG, "Scanning %s...", label);
  bool found = false;

  for (uint8_t addr = 1; addr < 0x78; ++addr) {
    if (i2c_master_probe(bus_handle, addr, I2C_TIMEOUT_MS) == ESP_OK) {
      ESP_LOGI(TAG, "%s found device at 0x%02X", label, addr);
      found = true;
    }
  }

  if (!found) {
    ESP_LOGW(TAG, "%s scan found no devices", label);
  }
}

static esp_err_t tof_set_shutdown_pin(gpio_num_t pin, uint32_t level) {
  return gpio_set_level(pin, level);
}

static esp_err_t tof_shutdown_pins_init(void) {
  const gpio_config_t cfg = {
      .pin_bit_mask = (1ULL << X_SHUT_TOF_2) | (1ULL << X_SHUT_TOF_3),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };

  RETURN_ON_ERROR(gpio_config(&cfg), TAG, "xshut pin init failed");
  RETURN_ON_ERROR(tof_set_shutdown_pin(X_SHUT_TOF_2, 0), TAG,
                  "failed to hold TOF2 in reset");
  RETURN_ON_ERROR(tof_set_shutdown_pin(X_SHUT_TOF_3, 0), TAG,
                  "failed to hold TOF3 in reset");
  vTaskDelay(pdMS_TO_TICKS(20));
  return ESP_OK;
}
static esp_err_t vl53l0x_check_identity(i2c_master_dev_handle_t dev_handle,
                                        uint8_t address) {
  uint8_t model_id = 0;
  RETURN_ON_ERROR(
      i2c_read_reg(dev_handle, REG_IDENTIFICATION_MODEL_ID, &model_id), TAG,
      "model ID read failed");

  if (model_id != 0xEE) {
    ESP_LOGE(TAG, "Unexpected model ID 0x%02X at 0x%02X", model_id, address);
    return ESP_FAIL;
  }

  return ESP_OK;
}

static esp_err_t vl53l0x_basic_init(i2c_master_dev_handle_t dev_handle,
                                    uint8_t address) {
  uint8_t value = 0;

  RETURN_ON_ERROR(vl53l0x_check_identity(dev_handle, address), TAG,
                  "VL53L0X identity check failed");
  RETURN_ON_ERROR(i2c_read_reg(dev_handle, REG_GPIO_HV_MUX_ACTIVE_HIGH, &value),
                  TAG, "GPIO mux read failed");
  RETURN_ON_ERROR(i2c_write_reg(dev_handle, REG_GPIO_HV_MUX_ACTIVE_HIGH,
                                value & (uint8_t)~0x10),
                  TAG, "GPIO mux write failed");
  RETURN_ON_ERROR(
      i2c_write_reg(dev_handle, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04), TAG,
      "interrupt config failed");
  RETURN_ON_ERROR(i2c_write_reg(dev_handle, REG_SYSTEM_INTERRUPT_CLEAR, 0x01),
                  TAG, "interrupt clear failed");

  return ESP_OK;
}

static esp_err_t vl53l0x_set_address(i2c_master_dev_handle_t dev_handle,
                                     uint8_t old_address, uint8_t new_address) {
  RETURN_ON_ERROR(i2c_write_reg(dev_handle, REG_I2C_SLAVE_DEVICE_ADDRESS,
                                new_address & 0x7F),
                  TAG, "set address failed");
  RETURN_ON_ERROR(
      i2c_master_device_change_address(dev_handle, new_address, I2C_TIMEOUT_MS),
      TAG, "device handle address update failed");
  vTaskDelay(pdMS_TO_TICKS(10));
  ESP_LOGI(TAG, "Device moved from 0x%02X to 0x%02X", old_address, new_address);
  return ESP_OK;
}

static esp_err_t vl53l0x_read_range_mm(i2c_master_dev_handle_t dev_handle,
                                       uint8_t address, uint16_t *range_mm) {
  TickType_t start = xTaskGetTickCount();
  uint8_t status = 0;
  uint8_t raw_range[2] = {0};

  RETURN_ON_ERROR(i2c_write_reg(dev_handle, REG_SYSRANGE_START, 0x01), TAG,
                  "failed to start range measurement");

  do {
    RETURN_ON_ERROR(
        i2c_read_reg(dev_handle, REG_RESULT_INTERRUPT_STATUS, &status), TAG,
        "failed to poll measurement status");
    if ((status & 0x07) != 0) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  } while ((xTaskGetTickCount() - start) <
           pdMS_TO_TICKS(RANGE_READ_TIMEOUT_MS));

  if ((status & 0x07) == 0) {
    ESP_LOGW(TAG, "Range read timed out at 0x%02X", address);
    return ESP_ERR_TIMEOUT;
  }

  RETURN_ON_ERROR(i2c_read_regs(dev_handle, REG_RESULT_RANGE_STATUS + 10,
                                raw_range, sizeof(raw_range)),
                  TAG, "failed to read range result");

  *range_mm = ((uint16_t)raw_range[0] << 8) | raw_range[1];

  return i2c_write_reg(dev_handle, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

static esp_err_t bring_up_tof_on_bus_2(gpio_num_t xshut_pin,
                                       uint8_t new_address, const char *label,
                                       i2c_master_dev_handle_t *dev_handle) {
  ESP_LOGI(TAG, "Enabling %s on bus 2", label);
  RETURN_ON_ERROR(tof_set_shutdown_pin(xshut_pin, 1), TAG,
                  "failed to release XSHUT");
  vTaskDelay(pdMS_TO_TICKS(50));

  scan_i2c_bus(s_bus_2, "I2C bus 2");
  RETURN_ON_ERROR(i2c_add_device(s_bus_2, VL53L0X_DEFAULT_ADDR, dev_handle),
                  TAG, "add default-address device failed");
  RETURN_ON_ERROR(vl53l0x_basic_init(*dev_handle, VL53L0X_DEFAULT_ADDR), TAG,
                  "basic init failed");
  RETURN_ON_ERROR(
      vl53l0x_set_address(*dev_handle, VL53L0X_DEFAULT_ADDR, new_address), TAG,
      "address assignment failed");
  RETURN_ON_ERROR(vl53l0x_basic_init(*dev_handle, new_address), TAG,
                  "re-init after address change failed");

  ESP_LOGI(TAG, "%s now responding at 0x%02X", label, new_address);
  scan_i2c_bus(s_bus_2, "I2C bus 2");
  return ESP_OK;
}

static esp_err_t init_all_sensors(tof_sensor_t *sensors, size_t sensor_count) {
  if (sensor_count < 3) {
    return ESP_ERR_INVALID_ARG;
  }

  RETURN_ON_ERROR(tof_shutdown_pins_init(), TAG, "shutdown pin setup failed");
  RETURN_ON_ERROR(
      i2c_bus_init(I2C_BUS_1, I2C_BUS_1_SDA, I2C_BUS_1_SCL, &s_bus_1), TAG,
      "I2C bus 1 init failed");
  RETURN_ON_ERROR(
      i2c_bus_init(I2C_BUS_2, I2C_BUS_2_SDA, I2C_BUS_2_SCL, &s_bus_2), TAG,
      "I2C bus 2 init failed");

  ESP_LOGI(TAG, "Stage 1: baseline scan with TOF2 and TOF3 held in reset");
  scan_i2c_bus(s_bus_1, "I2C bus 1");
  scan_i2c_bus(s_bus_2, "I2C bus 2");

  sensors[0].name = "TOF1";
  sensors[0].address = VL53L0X_DEFAULT_ADDR;
  RETURN_ON_ERROR(
      i2c_add_device(s_bus_1, sensors[0].address, &sensors[0].handle), TAG,
      "TOF1 device add failed");
  RETURN_ON_ERROR(vl53l0x_basic_init(sensors[0].handle, sensors[0].address),
                  TAG, "TOF1 init failed");
  ESP_LOGI(TAG, "TOF1 on bus 1 responding at 0x%02X", sensors[0].address);

  ESP_LOGI(
      TAG,
      "Stage 2: release TOF2 only; expect device at default address 0x%02X",
      VL53L0X_DEFAULT_ADDR);
  sensors[1].name = "TOF2";
  sensors[1].address = VL53L0X_SENSOR_2_ADDR;
  RETURN_ON_ERROR(bring_up_tof_on_bus_2(X_SHUT_TOF_2, sensors[1].address,
                                        sensors[1].name, &sensors[1].handle),
                  TAG, "TOF2 bring-up failed");

  ESP_LOGI(
      TAG,
      "Stage 3: release TOF3 only; expect 0x%02X and existing TOF2 at 0x%02X",
      VL53L0X_DEFAULT_ADDR, VL53L0X_SENSOR_2_ADDR);
  sensors[2].name = "TOF3";
  sensors[2].address = VL53L0X_SENSOR_3_ADDR;
  RETURN_ON_ERROR(bring_up_tof_on_bus_2(X_SHUT_TOF_3, sensors[2].address,
                                        sensors[2].name, &sensors[2].handle),
                  TAG, "TOF3 bring-up failed");

  ESP_LOGI(TAG, "Stage 4: final scan; expect TOF2 at 0x%02X and TOF3 at 0x%02X",
           VL53L0X_SENSOR_2_ADDR, VL53L0X_SENSOR_3_ADDR);
  scan_i2c_bus(s_bus_1, "I2C bus 1");
  scan_i2c_bus(s_bus_2, "I2C bus 2");
  return ESP_OK;
}

static void lidar_sampling_task(void *arg) {
  tof_sensor_t *sensors = (tof_sensor_t *)arg;
  const size_t sensor_count = 3;

  while (true) {
    for (size_t i = 0; i < sensor_count; ++i) {
      uint16_t range_mm = 0;
      esp_err_t err = vl53l0x_read_range_mm(sensors[i].handle,
                                            sensors[i].address, &range_mm);

      if (err == ESP_OK) {
        update_lidar_reading(i, range_mm, true);
      } else {
        update_lidar_reading(i, 0, false);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(120));
  }
}

static void motor_control_task(void *arg) {
  (void)arg;
  motion_cmd_t last_applied = MOTION_STOP;
  bool was_blocked = false;

  while (true) {
    uint16_t mm[3] = {0};
    bool valid[3] = {false, false, false};
    motion_cmd_t desired = get_motion_command();
    bool blocked = false;

    get_lidar_snapshot(mm, valid);

    if (s_motion_inhibit) {
      desired = MOTION_STOP;
    } else if (desired == MOTION_FORWARD) {
      bool front_1_close = valid[0] && mm[0] <= OBSTACLE_STOP_MM;
      bool front_2_close = valid[1] && mm[1] <= OBSTACLE_STOP_MM;
      blocked = front_1_close || front_2_close;
    } else if (desired == MOTION_BACKWARD) {
      bool rear_close = valid[2] && mm[2] <= OBSTACLE_STOP_MM;
      blocked = rear_close;
    }

    motion_cmd_t applied = blocked ? MOTION_STOP : desired;
    if (applied != last_applied) {
      motor_apply_command(applied);
      last_applied = applied;
      ESP_LOGI(TAG, "Motor state -> %s", motion_cmd_to_string(applied));
    }

    if (blocked && !was_blocked) {
      ESP_LOGW(TAG, "Safety stop: cmd=%s, S1=%u(%d) S2=%u(%d) S3=%u(%d)",
               motion_cmd_to_string(desired), mm[0], valid[0], mm[1], valid[1],
               mm[2], valid[2]);
    }
    was_blocked = blocked;

    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(40));
  }
}

static void bluetooth_receiver_task(void *arg) {
  (void)arg;
  if (bluetooth_init() != ESP_OK) {
    ESP_LOGE(TAG, "Bluetooth init failed");
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "BLE receiver ready: write left/right/forward/backward/stop");

  while (true) {
    bt_rx_msg_t msg = {0};
    if (xQueueReceive(s_bt_rx_queue, &msg, portMAX_DELAY) == pdTRUE) {
      if (!s_ble_command_link_ready) {
        continue;
      }

      motion_cmd_t cmd = MOTION_STOP;
      if (parse_motion_command(msg.text, &cmd)) {
        if (!s_ble_command_link_ready) {
          continue;
        }
        set_motion_inhibit(false);
        set_motion_command(cmd);
        if (s_motor_task_handle != NULL) {
          xTaskNotifyGive(s_motor_task_handle);
        }
        ESP_LOGI(TAG, "Bluetooth command -> %s", motion_cmd_to_string(cmd));
      } else {
        ESP_LOGW(TAG, "Ignoring unknown command: '%s'", msg.text);
      }
    }
  }
}

void app_main(void) {
  tof_sensor_t sensors[3] = {0};

  s_state_mutex = xSemaphoreCreateMutex();
  if (s_state_mutex == NULL) {
    ESP_LOGE(TAG, "Failed to create state mutex");
    return;
  }

  s_bt_rx_queue = xQueueCreate(10, sizeof(bt_rx_msg_t));
  if (s_bt_rx_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create bluetooth queue");
    return;
  }

  if (init_all_sensors(sensors, sizeof(sensors) / sizeof(sensors[0])) !=
      ESP_OK) {
    ESP_LOGE(TAG, "Sensor initialization failed. Check wiring, pullups, XSHUT "
                  "pins, and power.");
    return;
  }

  if (motor_driver_init() != ESP_OK) {
    ESP_LOGE(TAG, "Motor driver initialization failed");
    return;
  }

  motor_stop();

  xTaskCreatePinnedToCore(motor_control_task, "motor_ctrl", 4096, NULL, 10,
                          &s_motor_task_handle, 0);
  xTaskCreate(lidar_sampling_task, "lidar_task", 6144, sensors, 9, NULL);
  xTaskCreatePinnedToCore(bluetooth_receiver_task, "bt_rx_task", 8192, NULL, 8,
                          NULL, 1);

  ESP_LOGI(TAG, "System started: core0 motor control + lidar safety + "
                "BLE command receiver on core1");

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
