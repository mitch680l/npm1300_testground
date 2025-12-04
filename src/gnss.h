#ifndef GNSS_H
#define GNSS_H

#define I2C_NODE DT_NODELABEL(i2c2)
#define QWIIC_EN_NODE DT_ALIAS(qwiic_en)


extern const struct device *i2c_dev;


struct __attribute__((packed)) ubx_nav_pvt_t {
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t heading;
    uint32_t sAcc;
    uint32_t headingAcc;
    uint16_t pDOP;
    uint8_t reserved[6];
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
};


bool validate_gps_fix(const struct ubx_nav_pvt_t *pvt); 

void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b);

void print_nav_pvt_json(const struct ubx_nav_pvt_t *pvt);

bool parse_nav_pvt(const uint8_t *buf, size_t len, struct ubx_nav_pvt_t *out);

int parse_ack(const uint8_t *buf, size_t len, uint8_t cls, uint8_t id);

bool send_ubx_message(uint8_t *msg, size_t len, const char *desc);

bool configure_gps_10hz(void);

bool configure_constellation(void);

void gnss_main_loop();
void gnss_int();

void test_configure(uint8_t *msg);
bool parse_constellation_val_get(const uint8_t *buf, size_t len);
bool set_gps_only(void);
extern struct k_mutex json_mutex;


#define RAM_LAYER 0x00

#define SIGNAL_ON 0x01
#define SIGNAL_OFF 0x00

#define GPS_MAX_SPEED_MPS 100.0
#define GPS_MAX_LAT_JUMP 5000000
#define GPS_MAX_LON_JUMP 5000000
#define GPS_MAX_TIME_BACKWARDS_MS 100

#define GPS_EN 0x1031001f
#define GPS_L1CA_ENA 0x10310001

#define SBAS_ENA 0x10310020
#define SBAS_L1CA_ENA 0x10310005

#define GAL_ENA 0x10310021
#define GAL_E1B_ENA 0x10310007

#define BDS_ENA 0x10310022
#define BDS_B1_ENA 0x1031000d
#define BDS_B1C_ENA 0x1031000f

#define QZSS_ENA 0x10310024
#define QZSS_L1CA_ENA 0x10310012
#define QZSS_L1BC_ENA 0x10310039
#define QZSS_L1S_ENA 0x10310014

#define GLO_ENA 0x10310025
#define GLO_L1_ENA 0x10310018

#define NAVIC_ENA 0x10310026


#endif