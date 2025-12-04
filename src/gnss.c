#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include "gnss.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <errno.h>
#include "config.h"
#include <math.h>

struct gps_rate_config {
    uint8_t rate_hz;
    uint16_t meas_rate_ms;
    uint16_t nav_rate_cycles;
};

static struct {
    bool initialized;
    uint32_t iTOW;
    int32_t lat;
    int32_t lon;
    int32_t gSpeed;
} last_valid_fix = {0};

static const struct gps_rate_config rate_configs[] = {
    {1,  1000, 1},
    {5,  200,  1},
    {10, 100,  1},
    {15, 66,  1},
    {20, 50,   1},
    {25, 40,   1},
};

static uint8_t cfg_rate_template[] = {
    0xB5, 0x62,           
    0x06, 0x08,           
    0x06, 0x00,           
    0x00, 0x00,           
    0x01, 0x00,           
    0x01, 0x00,           
    0x00, 0x00            
};

static uint8_t cfg_msg_nav_pvt[] = {
    0xB5, 0x62,           
    0x06, 0x01,           
    0x03, 0x00,           
    0x01, 0x07,           
    0x01,                 
    0x00, 0x00            
};



static uint8_t rxbuf[GPS_READ_BUFFER_SIZE];
static uint8_t partial_buffer[GPS_READ_BUFFER_SIZE * 2];
static size_t partial_buffer_len = 0;
static struct ubx_nav_pvt_t pvt;
static uint32_t last_tow = 0;
static int pvt_count = 0;
static int read_count = 0;
static int fix_count_in_period = 0;
static uint32_t fix_rate_start_time = 0;
static bool gps_configured = false;
static uint32_t last_read_time = 0;
static double actual_rate = 0.0;
LOG_MODULE_REGISTER(gnss_opt);

const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

/**
 * Calculate UBX checksum for message validation
 * @param data - data buffer to calculate checksum for
 * @param len - length of data buffer
 * @param ck_a - pointer to store first checksum byte
 * @param ck_b - pointer to store second checksum byte
 * @return void - fills checksum output parameters
 */
inline void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = 0;
    *ck_b = 0;
    for (size_t i = 0; i < len; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}

/**
 * Find GPS rate configuration for target frequency
 * @param target_hz - desired GPS update rate in Hz
 * @return pointer to matching rate configuration or default 10Hz config
 */
const struct gps_rate_config* get_rate_config(uint8_t target_hz) {
    for (int i = 0; i < ARRAY_SIZE(rate_configs); i++) {
        if (rate_configs[i].rate_hz == target_hz) {
            return &rate_configs[i];
        }
    }
    return &rate_configs[2];
}




/**
 * Format NAV-PVT data into JSON string
 * @param pvt - pointer to UBX NAV-PVT structure
 * @return void - updates global json_payload with formatted GPS data
 */

void format_nav_pvt_json(const struct ubx_nav_pvt_t *pvt)
{
    double lat     = (double)pvt->lat / 1e7; 
    double lon     = (double)pvt->lon / 1e7;       
    double alt     = (double)pvt->hMSL / 1000.0;   
    double speed   = (double)pvt->gSpeed / 1000.0; 
    double heading = (double)pvt->heading / 1e5;   
    double vel_n   = (double)pvt->velN / 1000.0;   
    double vel_e   = (double)pvt->velE / 1000.0;   
    double vel_d   = (double)pvt->velD / 1000.0;   
    double h_acc   = (double)pvt->hAcc / 1000.0;   
    double v_acc   = (double)pvt->vAcc / 1000.0;   

    if (!isfinite(lat)) lat = 0.0;
    if (!isfinite(lon)) lon = 0.0;
    if (!isfinite(alt)) alt = 0.0;
    if (!isfinite(speed)) speed = 0.0;
    if (!isfinite(heading)) heading = 0.0;
    if (!isfinite(vel_n)) vel_n = 0.0;
    if (!isfinite(vel_e)) vel_e = 0.0;
    if (!isfinite(vel_d)) vel_d = 0.0;
    if (!isfinite(h_acc)) h_acc = 0.0;
    if (!isfinite(v_acc)) v_acc = 0.0;
    double rate_snapshot = actual_rate;
    if (!isfinite(rate_snapshot)) rate_snapshot = 0.0;

    k_mutex_lock(&json_mutex, K_FOREVER);

    int len = snprintf(json_payload, sizeof(json_payload),
        "{"
            "\"time\":\"%04d-%02d-%02dT%02d:%02d:%02dZ\","
            "\"lat\":%.7f,"
            "\"lon\":%.7f,"
            "\"fixType\":%d,"
            "\"numSV\":%d,"
            "\"alt_m\":%.1f,"
            "\"gSpeed\":%.1f,"
            "\"velN\":%.1f,"
            "\"velE\":%.1f,"
            "\"velD\":%.1f,"
            "\"heading\":%.1f,"
            "\"hAcc\":%.1f,"
            "\"vAcc\":%.1f,"
            "\"rate\":%.1f"
        "}",
        pvt->year, pvt->month, pvt->day,
        pvt->hour, pvt->min, pvt->sec,
        lat, lon,
        pvt->fixType, pvt->numSV,
        alt, speed, vel_n, vel_e, vel_d,
        heading, h_acc, v_acc, rate_snapshot);

    if (len < 0 || len >= sizeof(json_payload)) {
        LOG_WRN("JSON payload truncated (%d bytes)", len);
    }

    k_mutex_unlock(&json_mutex);
}
/**
 * Parse UBX NAV-PVT message from buffer
 * @param buf - data buffer to parse
 * @param len - buffer length
 * @param out - output structure to fill with parsed data
 * @return true if valid NAV-PVT message parsed, false otherwise
 */
bool parse_nav_pvt(const uint8_t *buf, size_t len, struct ubx_nav_pvt_t *out) {
    if (len < 8 + NAV_PVT_LEN) return false;
    
    for (size_t i = 0; i <= len - (8 + NAV_PVT_LEN); i++) {
        if (buf[i] == 0xB5 && buf[i+1] == 0x62 && buf[i+2] == 0x01 && buf[i+3] == 0x07) {
            uint16_t payload_len = buf[i+4] | (buf[i+5] << 8);
            if (payload_len != NAV_PVT_LEN) continue;
            
            if (i + 8 + payload_len > len) {
                break;
            }
            
            memcpy(out, &buf[i+6], NAV_PVT_LEN);
            
            #ifdef VERIFY_CHECKSUM
            uint8_t ck_a, ck_b;
            ubx_checksum(&buf[i+2], 4 + payload_len, &ck_a, &ck_b);
            if (buf[i+6+payload_len] != ck_a || buf[i+7+payload_len] != ck_b) {
                continue;
            }
            #endif
            
            return true;
        }
    }
    return false;
}

/**
 * Process incoming GPS data with partial message handling
 * @param new_data - incoming data buffer
 * @param new_len - data length
 * @return number of bytes processed, updates global pvt structure if valid message found
 */
size_t process_gps_data(const uint8_t *new_data, size_t new_len) {
    size_t processed = 0;
    
    if (partial_buffer_len > 0) {
        size_t copy_len = MIN(new_len, sizeof(partial_buffer) - partial_buffer_len);
        memcpy(partial_buffer + partial_buffer_len, new_data, copy_len);
        partial_buffer_len += copy_len;
        
        if (parse_nav_pvt(partial_buffer, partial_buffer_len, &pvt)) {
            partial_buffer_len = 0;
            processed = copy_len;
            return processed;
        }
        
        if (partial_buffer_len > GPS_READ_BUFFER_SIZE) {
            LOG_WRN("Partial buffer overflow, resetting");
            partial_buffer_len = 0;
        }
    }
    
    if (parse_nav_pvt(new_data, new_len, &pvt)) {
        processed = new_len;
        return processed;
    }
    
    for (int i = new_len - 1; i >= 0; i--) {
        if (new_data[i] == 0xB5 && i + 1 < new_len && new_data[i+1] == 0x62) {
            size_t partial_len = new_len - i;
            if (partial_len < 8 + NAV_PVT_LEN) {
                memcpy(partial_buffer, new_data + i, partial_len);
                partial_buffer_len = partial_len;
                processed = i;
                LOG_DBG("Stored partial UBX message (%zu bytes)", partial_len);
                break;
            }
        }
    }
    
    return processed;
}

/**
 * Parse UBX ACK/NAK message from buffer with detailed NACK decoding
 * @param buf - data buffer to parse
 * @param len - buffer length
 * @param cls - message class to match
 * @param id - message ID to match
 * @return 1 if ACK received, -1 if NAK received, 0 if not found
 */
int parse_ack(const uint8_t *buf, size_t len, uint8_t cls, uint8_t id) {
    if (len < 10) return 0;

    for (size_t i = 0; i <= len - 10; i++) {
        if (buf[i] == 0xB5 && buf[i+1] == 0x62 && buf[i+2] == 0x05) {
            if (buf[i+3] == 0x01 && buf[i+6] == cls && buf[i+7] == id) {
                LOG_INF("ACK-ACK received for Class 0x%02X, ID 0x%02X", cls, id);
                return 1;
            } else if (buf[i+3] == 0x00 && buf[i+6] == cls && buf[i+7] == id) {
                LOG_ERR("ACK-NAK received for Class 0x%02X, ID 0x%02X", cls, id);
                LOG_ERR("NACK Reason: Configuration rejected by receiver");
                LOG_HEXDUMP_ERR(&buf[i], 10, "NACK Packet");
                return -1;
            }
        }
    }
    return 0;
}

/**
 * Send UBX message to GPS and wait for acknowledgment
 * @param msg - message buffer to send
 * @param len - message length
 * @param desc - description for logging
 * @return true if message sent and ACK received, false otherwise
 */
bool send_ubx_message(uint8_t *msg, size_t len, const char *desc) {
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready in send_ubx_message");
        return false;
    }

    uint8_t ck_a, ck_b;
    ubx_checksum(&msg[2], len - 4, &ck_a, &ck_b);
    msg[len - 2] = ck_a;
    msg[len - 1] = ck_b;

    int ret = i2c_write(i2c_dev, msg, len, M10S_ADDR);
    if (ret != 0) {
        LOG_ERR("Failed to send %s: %d", desc, ret);
        return false;
    }


    uint32_t start_time = k_uptime_get_32();
    while ((k_uptime_get_32() - start_time) < GPS_CONFIG_TIMEOUT_MS) {
        k_sleep(K_MSEC(50));

        ret = i2c_burst_read(i2c_dev, M10S_ADDR, 0xFF, rxbuf, 256);
        if (ret == 0) {
            int ack_result = parse_ack(rxbuf, 256, msg[2], msg[3]);
            if (ack_result == 1) {
                LOG_INF("%s: ACK received", desc);
                return true;
            } else if (ack_result == -1) {
                LOG_ERR("%s: NACK received - configuration rejected", desc);
                return false;
            }
        }
    }

    LOG_WRN("%s: No ACK/NACK received within timeout", desc);
    return false;
}


/**
 * Configure GPS update rate using multiple UBX methods
 * @param target_hz - desired update rate in Hz
 * @return true if at least one configuration method succeeded
 */
bool configure_gps_rate(uint8_t target_hz) {
    const struct gps_rate_config *config = get_rate_config(target_hz);
    bool success = false;
    
    LOG_INF("Configuring GPS for %d Hz (meas_rate: %d ms)", target_hz, config->meas_rate_ms);
    
    uint8_t cfg_rate_msg[sizeof(cfg_rate_template)];
    memcpy(cfg_rate_msg, cfg_rate_template, sizeof(cfg_rate_template));
    cfg_rate_msg[6] = config->meas_rate_ms & 0xFF;
    cfg_rate_msg[7] = (config->meas_rate_ms >> 8) & 0xFF;
    
    if (send_ubx_message(cfg_rate_msg, sizeof(cfg_rate_msg), "CFG-RATE")) {
        success = true;
    }
    
    k_sleep(K_MSEC(100));

    if (send_ubx_message(cfg_msg_nav_pvt, sizeof(cfg_msg_nav_pvt), "CFG-MSG NAV-PVT")) {
        success = true;
    }

    return success;
}

/**
 * Initialize GNSS system - power on, configure I2C, setup GPS
 * @return void - initializes GPS hardware and configuration
 */
void gnss_int(void) {
    LOG_INF("Starting MAX-M10S GPS at %d Hz", gnss_config.update_rate);
    k_sleep(K_MSEC(GPS_STARTUP_DELAY_MS-13));
    
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return;
    }
    
    for (int attempt = 0; attempt < GPS_MAX_CONFIG_ATTEMPTS; attempt++) {
        LOG_INF("Configuration attempt %d/%d", attempt + 1, GPS_MAX_CONFIG_ATTEMPTS);
        if (configure_gps_rate(gnss_config.update_rate)) {
            gps_configured = true;
            break;
        }
        k_sleep(K_MSEC(500));
    }

    if (!gps_configured) {
        LOG_WRN("GPS configuration failed, continuing with default settings");
    }

    // Configure GPS-only constellation
    k_sleep(K_MSEC(500));
    set_gps_only();

    LOG_INF("GPS initialization complete, starting data collection...");
    last_read_time = k_uptime_get_32();
}

/**
 * Main GNSS processing loop - reads and processes GPS data
 * @return void - continuously processes GPS data and updates position info
 */
void gnss_main_loop(void)
{
    if (gnss_config.update_rate == 0) {
        LOG_WRN("update_rate is 0; skipping read");
        k_sleep(K_MSEC(250));
        return;
    }

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready in gnss_main_loop");
        k_sleep(K_MSEC(250));
        return;
    }


    uint8_t current_rate = gnss_config.update_rate;
    if (current_rate == 0) {
        LOG_ERR("update_rate became 0, using default 250ms");
        k_sleep(K_MSEC(250));
        return;
    }

    k_sleep(K_MSEC((1000 / current_rate) - 13));

    uint32_t current_time = k_uptime_get_32();
    last_read_time = current_time;

    memset(rxbuf, 0, GPS_READ_BUFFER_SIZE);
    int ret = i2c_burst_read(i2c_dev, M10S_ADDR, 0xFF, rxbuf, GPS_READ_BUFFER_SIZE);
    read_count++;

    if (ret != 0) {
        if ((read_count % 100) == 0) {
            LOG_WRN("I2C read failed: %d (count: %d)", ret, read_count);
        }
        return;
    }


    size_t actual_data_len = 0;
    for (int i = GPS_READ_BUFFER_SIZE - 1; i >= 0; i--) {
        if (rxbuf[i] != 0) { actual_data_len = i + 1; break; }
    }
    if (actual_data_len == 0) { return; }

    process_gps_data(rxbuf, actual_data_len);

    if (pvt.numSV == 0 || pvt.fixType <= 1 || pvt.fixType > 4) {


    } 
    else if (pvt.fixType == 2) {

    } 
    else if (pvt.fixType == 3 || pvt.fixType == 4) {

    }

    if (pvt.iTOW != last_tow) {
        last_tow = pvt.iTOW;
        pvt_count++;

        if (pvt.fixType >= 2) {
            bool valid = validate_gps_fix(&pvt);

            last_valid_fix.initialized = true;
            last_valid_fix.iTOW   = pvt.iTOW;
            last_valid_fix.lat    = pvt.lat;
            last_valid_fix.lon    = pvt.lon;
            last_valid_fix.gSpeed = pvt.gSpeed;

            if (!valid) {
                return;
            }

            if (fix_rate_start_time == 0) {
                fix_rate_start_time = current_time;
                fix_count_in_period = 0;
            }

            format_nav_pvt_json(&pvt);
            fix_count_in_period++;

            if (fix_rate_start_time > 0 && (current_time - fix_rate_start_time) >= 10000U) {
                actual_rate = fix_count_in_period / 10.0;
                LOG_INF("GPS Rate: %.1f Hz (target: %d Hz, fixes: %d, total: %d)",
                        actual_rate, gnss_config.update_rate, fix_count_in_period, pvt_count);
                pvt_count = 0;
                fix_rate_start_time = current_time;
                fix_count_in_period = 0;
            }
        } else {
            if ((pvt_count % 50) == 0) {
                LOG_INF("Acquiring fix... (type: %d, sats: %d)", pvt.fixType, pvt.numSV);
            }
        }
    }

    

    if ((read_count % 500) == 0) {
        LOG_DBG("Read stats: %d reads, %d PVT messages, rate: %d Hz, partial: %zu bytes",
                read_count, pvt_count, gnss_config.update_rate, partial_buffer_len);
    }
}





/**
 * Validate GPS fix for sanity (time, speed, position)
 * @param pvt - pointer to current UBX NAV-PVT structure
 * @return true if fix is valid, false if should be rejected
 */
bool validate_gps_fix(const struct ubx_nav_pvt_t *pvt) {
    if (pvt->gSpeed < 0) {
        LOG_WRN("Rejecting fix: negative speed (%d mm/s)", pvt->gSpeed);
        return false;
    }
    
    double speed_mps = pvt->gSpeed / 1000.0;
    if (speed_mps > GPS_MAX_SPEED_MPS) {
        LOG_WRN("Rejecting fix: excessive speed (%.1f m/s > %.1f m/s)", 
                speed_mps, GPS_MAX_SPEED_MPS);
        return false;
    }
    
    if (last_valid_fix.initialized) {
        int32_t time_diff = (int32_t)(pvt->iTOW - last_valid_fix.iTOW);

        if (time_diff < -600000000) {
            time_diff += 604800000;
        }

        if (time_diff < -GPS_MAX_TIME_BACKWARDS_MS) {
            LOG_WRN("Rejecting fix: time went backwards (iTOW: %u -> %u, diff: %d ms)",
                    last_valid_fix.iTOW, pvt->iTOW, time_diff);
            return false;
        }


        int32_t lat_diff = pvt->lat - last_valid_fix.lat;
        int32_t lon_diff = pvt->lon - last_valid_fix.lon;
        uint32_t lat_jump = (lat_diff < 0) ? (uint32_t)(-lat_diff) : (uint32_t)lat_diff;
        uint32_t lon_jump = (lon_diff < 0) ? (uint32_t)(-lon_diff) : (uint32_t)lon_diff;
        
        if (lat_jump > GPS_MAX_LAT_JUMP) {
            LOG_WRN("Rejecting fix: excessive latitude jump (%d > %d)",
                    lat_jump, GPS_MAX_LAT_JUMP);
            return false;
        }
        
        if (lon_jump > GPS_MAX_LON_JUMP) {
            LOG_WRN("Rejecting fix: excessive longitude jump (%d > %d)",
                    lon_jump, GPS_MAX_LON_JUMP);
            return false;
        }
    }
    
    return true;
}

/**
 * Parse constellation configuration from VALGET response buffer
 * @param buf - buffer containing VALGET response (0xB5 0x62 0x06 0x8B)
 * @param len - buffer length
 * @return true if successfully parsed, false otherwise
 */
bool parse_constellation_val_get(const uint8_t *buf, size_t len) {
    // Find VALGET response header
    for (size_t i = 0; i < len - 10; i++) {
        if (buf[i] == 0xB5 && buf[i+1] == 0x62 && buf[i+2] == 0x06 && buf[i+3] == 0x8B) {
            LOG_INF("Found VALGET response at offset %d", i);

            uint16_t resp_len = buf[i+4] | (buf[i+5] << 8);
            LOG_INF("VALGET payload length: %d bytes", resp_len);

            if (i + 8 + resp_len > len) {
                LOG_WRN("VALGET response truncated");
                return false;
            }

            uint8_t version = buf[i+6];
            uint8_t layer = buf[i+7];
            LOG_INF("Version: 0x%02X, Layer: 0x%02X", version, layer);

            size_t cfg_offset = i + 10;
            size_t cfg_end = i + 6 + resp_len;

            while (cfg_offset + 5 <= cfg_end) {
                uint32_t key_id = buf[cfg_offset] |
                                 (buf[cfg_offset+1] << 8) |
                                 (buf[cfg_offset+2] << 16) |
                                 (buf[cfg_offset+3] << 24);
                uint8_t value = buf[cfg_offset+4];

                const char *name = "UNKNOWN";
                if (key_id == 0x1031001F) name = "GPS_ENA";
                else if (key_id == 0x10310021) name = "GPS_L1CA_ENA";
                else if (key_id == 0x10310022) name = "SBAS_ENA";
                else if (key_id == 0x10310024) name = "GAL_ENA";
                else if (key_id == 0x10310025) name = "BDS_ENA";
                else if (key_id == 0x10310026) name = "QZSS_ENA";

                LOG_INF("  CFG-SIGNAL-%s (0x%08X) = %s (0x%02X)",
                       name, key_id, value ? "ENABLED" : "DISABLED", value);

                cfg_offset += 5;
            }

            return true;
        }
    }

    LOG_WRN("VALGET response not found in buffer");
    return false;
}

/**
 * Configure GPS to use GPS constellation only
 * Sends VALSET command, waits for ACK, then sends VALGET to verify
 * @return true if configuration successful and verified
 */
bool set_gps_only(void) {
    static uint8_t cfg_valset[] = {0xB5, 0x62, 0x06, 0x8A, 0x1D, 0x00, 0x00, 0x01, 0x00, 0x00, 0x1F, 0x00, 0x31,
        0x10, 0x01, 0x21, 0x00, 0x31, 0x10, 0x00, 0x22, 0x00, 0x31, 0x10, 0x00, 0x24, 0x00, 0x31, 0x10,
        0x00, 0x25, 0x00, 0x31, 0x10, 0x00, 0x9F, 0xA5};

    static uint8_t valget[] = {0xB5, 0x62, 0x06, 0x8B, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x31,
        0x10, 0x21, 0x00, 0x31, 0x10, 0x22, 0x00, 0x31, 0x10, 0x24, 0x00, 0x31, 0x10, 0x25, 0x00, 0x31,
        0x10, 0x99, 0xEB};

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return false;
    }

    LOG_INF("Configuring GPS-only mode...");

    // Step 1: Send VALSET command with retry
    bool ack_received = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        LOG_INF("Sending VALSET (attempt %d/3)...", attempt + 1);

        uint16_t payload_len = cfg_valset[4] | (cfg_valset[5] << 8);
        size_t len = payload_len + 8;

        int ret = i2c_write(i2c_dev, cfg_valset, len, M10S_ADDR);
        if (ret != 0) {
            LOG_ERR("I2C Write Failed: %d", ret);
            continue;
        }

        uint32_t start_time = k_uptime_get_32();
        static uint8_t rx_buf[128];

        while ((k_uptime_get_32() - start_time) < GPS_CONFIG_TIMEOUT_MS) {
            k_sleep(K_MSEC(50));

            memset(rx_buf, 0, sizeof(rx_buf));
            ret = i2c_burst_read(i2c_dev, M10S_ADDR, 0xFF, rx_buf, sizeof(rx_buf));

            if (ret == 0) {
                int ack_result = parse_ack(rx_buf, sizeof(rx_buf), 0x06, 0x8A);

                if (ack_result == 1) {
                    LOG_INF("VALSET ACK received");
                    ack_received = true;
                    break;
                } else if (ack_result == -1) {
                    LOG_ERR("VALSET NAK received");
                    break;
                }
            }
        }

        if (ack_received) {
            break;
        }

        LOG_WRN("No ACK received, retrying...");
        k_sleep(K_MSEC(200));
    }

    if (!ack_received) {
        LOG_ERR("Failed to configure GPS after 3 attempts");
        return false;
    }

    // Step 2: Send VALGET to verify configuration
    k_sleep(K_MSEC(500));
    LOG_INF("Sending VALGET to verify configuration...");

    uint16_t valget_len = valget[4] | (valget[5] << 8);
    size_t len = valget_len + 8;

    int ret = i2c_write(i2c_dev, valget, len, M10S_ADDR);
    if (ret != 0) {
        LOG_ERR("VALGET write failed: %d", ret);
        return false;
    }

    // Step 3: Read much more data to catch VALGET response among NAV-PVT spam
    uint32_t start_time = k_uptime_get_32();
    uint8_t accumulator[2048];  // Much larger buffer for GPS spam
    size_t accum_len = 0;
    bool response_found = false;
    int read_attempts = 0;
    const int MAX_READ_ATTEMPTS = 50;  // Read many times to catch response

    while ((k_uptime_get_32() - start_time) < GPS_CONFIG_TIMEOUT_MS &&
           !response_found &&
           read_attempts < MAX_READ_ATTEMPTS) {

        k_sleep(K_MSEC(25));  // Faster polling

        static uint8_t rx_buf[256];
        memset(rx_buf, 0, sizeof(rx_buf));
        ret = i2c_burst_read(i2c_dev, M10S_ADDR, 0xFF, rx_buf, sizeof(rx_buf));
        read_attempts++;

        if (ret == 0) {
            // Find actual data length
            size_t data_len = 0;
            for (int i = sizeof(rx_buf) - 1; i >= 0; i--) {
                if (rx_buf[i] != 0) {
                    data_len = i + 1;
                    break;
                }
            }

            if (data_len > 0) {
                // Accumulate data if space available
                if (accum_len + data_len < sizeof(accumulator)) {
                    memcpy(accumulator + accum_len, rx_buf, data_len);
                    accum_len += data_len;
                }

                // Search accumulated buffer for VALGET response
                for (size_t i = 0; i < accum_len - 10; i++) {
                    if (accumulator[i] == 0xB5 && accumulator[i+1] == 0x62 &&
                        accumulator[i+2] == 0x06 && accumulator[i+3] == 0x8B) {

                        uint16_t msg_len = accumulator[i+4] | (accumulator[i+5] << 8);
                        size_t total_msg_len = msg_len + 8;

                        // Check if we have the complete message
                        if (i + total_msg_len <= accum_len) {
                            LOG_INF("VALGET response found (attempt %d, offset %d, total %d bytes)",
                                   read_attempts, i, accum_len);
                            response_found = true;
                            parse_constellation_val_get(accumulator + i, accum_len - i);
                            break;
                        }
                    }
                }
            }
        }
    }

    LOG_INF("Total read attempts: %d, accumulated: %d bytes", read_attempts, accum_len);

    if (!response_found) {
        LOG_WRN("VALGET response not received");
        if (accum_len > 0) {
            LOG_HEXDUMP_WRN(accumulator, MIN(accum_len, 128), "Accumulated data (first 128 bytes)");
        }
        return false;
    }

    LOG_INF("GPS-only configuration complete and verified");
    return true;
}

/**
 * Test function to write a specific UBX message and wait for ACK/NAK response.
 * Useful for debugging configuration commands.
 *
 * This version automatically calculates length from the UBX header and uses parse_ack.
 * @param msg - Pointer to the UBX message buffer (Must start with 0xB5 0x62)
 */
void test_configure(uint8_t *msg)
{
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return;
    }

    if (msg == NULL || msg[0] != 0xB5 || msg[1] != 0x62) {
        LOG_ERR("Invalid UBX message header");
        return;
    }

    // --- 0. Calculate Length ---
    // UBX Structure: [0xB5 0x62] [CLS] [ID] [LEN_LSB] [LEN_MSB] [PAYLOAD...] [CK_A] [CK_B]
    // Payload length is at index 4 and 5
    uint16_t payload_len = msg[4] | (msg[5] << 8);
    size_t len = payload_len + 8; // +8 covers Header(2), Class(1), ID(1), Len(2), Checksum(2)

    // Extract class and ID for ACK matching
    uint8_t msg_class = msg[2];
    uint8_t msg_id = msg[3];

    // --- 1. Write and Log ---
    LOG_INF("--- TEST CONFIGURE START ---");
    LOG_INF("Writing %d bytes to address 0x%02X (Class: 0x%02X, ID: 0x%02X):",
            len, M10S_ADDR, msg_class, msg_id);

    // Hexdump the TX buffer to console
    LOG_HEXDUMP_INF(msg, len, "TX Data");

    int ret = i2c_write(i2c_dev, msg, len, M10S_ADDR);
    if (ret != 0) {
        LOG_ERR("I2C Write Failed: %d", ret);
        return;
    }

    LOG_INF("Waiting for response (Class: 0x%02X, ID: 0x%02X)...", msg_class, msg_id);

    uint32_t start_time = k_uptime_get_32();
    static uint8_t test_rx_buf[256];
    bool response_received = false;
    bool is_poll = (msg_id == 0x8B); 

    while ((k_uptime_get_32() - start_time) < GPS_CONFIG_TIMEOUT_MS) {
        k_sleep(K_MSEC(50));

        memset(test_rx_buf, 0, sizeof(test_rx_buf));
        ret = i2c_burst_read(i2c_dev, M10S_ADDR, 0xFF, test_rx_buf, 128);

        if (ret == 0) {
            size_t data_len = 0;
            for (int i = 127; i >= 0; i--) {
                if (test_rx_buf[i] != 0) {
                    data_len = i + 1;
                    break;
                }
            }

            if (data_len > 0) {
                LOG_INF("Received %d bytes:", data_len);
                LOG_HEXDUMP_INF(test_rx_buf, data_len, "RX Data");

                // Check if this is a VALGET response
                if (is_poll) {
                    for (size_t i = 0; i < data_len - 4; i++) {
                        if (test_rx_buf[i] == 0xB5 && test_rx_buf[i+1] == 0x62 &&
                            test_rx_buf[i+2] == msg_class && test_rx_buf[i+3] == msg_id) {
                            LOG_INF("Found VALGET response at offset %d", i);

                            // Parse the VALGET response
                            // Structure: [B5 62] [06 8B] [LEN_L LEN_H] [VERSION] [LAYER] [RESERVED] [RESERVED] [CFG_DATA...] [CK_A CK_B]
                            uint16_t resp_len = test_rx_buf[i+4] | (test_rx_buf[i+5] << 8);
                            LOG_INF("VALGET payload length: %d bytes", resp_len);

                            if (i + 8 + resp_len <= data_len) {
                                uint8_t version = test_rx_buf[i+6];
                                uint8_t layer = test_rx_buf[i+7];
                                LOG_INF("Version: 0x%02X, Layer: 0x%02X", version, layer);

                                // Parse configuration items (starting at offset i+10)
                                size_t cfg_offset = i + 10;
                                size_t cfg_end = i + 6 + resp_len;

                                while (cfg_offset + 5 <= cfg_end) {
                                    uint32_t key_id = test_rx_buf[cfg_offset] |
                                                     (test_rx_buf[cfg_offset+1] << 8) |
                                                     (test_rx_buf[cfg_offset+2] << 16) |
                                                     (test_rx_buf[cfg_offset+3] << 24);
                                    uint8_t value = test_rx_buf[cfg_offset+4];

                                    // Decode constellation settings
                                    const char *name = "UNKNOWN";
                                    if (key_id == 0x1031001F) name = "GPS_ENA";
                                    else if (key_id == 0x10310021) name = "GPS_L1CA_ENA";
                                    else if (key_id == 0x10310022) name = "SBAS_ENA";
                                    else if (key_id == 0x10310024) name = "GAL_ENA";
                                    else if (key_id == 0x10310025) name = "BDS_ENA";
                                    else if (key_id == 0x10310026) name = "QZSS_ENA";

                                    LOG_INF("  CFG-SIGNAL-%s (0x%08X) = %s (0x%02X)",
                                           name, key_id, value ? "ENABLED" : "DISABLED", value);

                                    cfg_offset += 5;
                                }
                            } else {
                                LOG_WRN("VALGET response truncated");
                            }

                            response_received = true;
                            break;
                        }
                    }
                }
            }


            if (!is_poll) {
                int ack_result = parse_ack(test_rx_buf, 128, msg_class, msg_id);

                if (ack_result == 1) {
                    LOG_INF("Configuration successful - ACK received");
                    response_received = true;
                    break;
                } else if (ack_result == -1) {
                    LOG_ERR("Configuration failed - NAK received");
                    response_received = true;
                    break;
                }
            } else if (response_received) {
                break;
            }
        } else {
            LOG_WRN("I2C Read error: %d", ret);
        }
    }

    if (!response_received) {
        LOG_WRN("No response received within timeout (%d ms)", GPS_CONFIG_TIMEOUT_MS);
    }

    LOG_INF("--- TEST CONFIGURE COMPLETE ---");
}