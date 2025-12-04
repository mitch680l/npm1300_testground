#include "config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>
#include <zephyr/net/tls_credentials.h>
#include <zephyr/random/random.h>
#include <nrf_modem_at.h>
#include <modem/modem_key_mgmt.h>
#include <zephyr/sys/crc.h>
#include <zephyr/storage/flash_map.h>

LOG_MODULE_REGISTER(configuration, LOG_LEVEL_DBG);
char json_payload[JSON_BUF_SIZE] = "NO PVT";
char sensor_payload[JSON_BUF_SIZE] = "NO SENSOR DATA";
char json_payload_lte[JSON_BUF_SIZE] = "NO LTE";
char json_bmp390[JSON_BUF_SIZE] = "NO BMP390 DATA";
char json_iis2mdc[JSON_BUF_SIZE] = "NO IIS2MDC DATA";
char json_npm1300[JSON_BUF_SIZE_SMALL] = "NO NPM1300 DATA";
char json_icm42688[JSON_BUF_SIZE] = "NO ICM42688 DATA";
char topic_gps[DEFAULT_LEN] = "/gps";
char topic_pwr[DEFAULT_LEN] = "/power";
char topic_sensor[DEFAULT_LEN] = "/sensor";
char topic_lte[DEFAULT_LEN] = "/lte";
char pwd[64] = "Kalscott123";
char usr[64] = "admin";
char firmware_filename[FOTA_FILE_NAME_MAX_LEN] = "firmware.bin";
struct mqtt_utf8 struct_pass;
struct mqtt_utf8 struct_user;
system_enable_t sys_enable_config;
enum fota_state current_state = FOTA_IDLE;
mqtt_config_t mqtt_config;
ota_config_t ota_config;
hardware_info_t hw_info;
modem_info_t modem_info;
sensor_config_t sensor_config;
customer_info_t customer_info;
message_settings_t msg_settings;
bool modem_info_update_needed = false;
K_MUTEX_DEFINE(json_mutex);


gnss_config_t gnss_config = {
    .update_rate = 10,
    .version = "1.0.0",
    .constellation_mask = 0x01,
    .accuracy_threshold = 5
};
