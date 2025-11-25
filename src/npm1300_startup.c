#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <stdio.h>

char json_npm1300[500] = "NO NPM1300 DATA";
LOG_MODULE_REGISTER(npm1300_ctrl, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------- */
/* nPM1300 I2C and Register Definitions                                       */
/* -------------------------------------------------------------------------- */

#define NPM1300_I2C_ADDR        0x6B
#define NPM1300_I2C_DEV         DT_NODELABEL(i2c2)


#define BAT_TEMP_CONF_REG_MSB   0x03
#define BAT_TEMP_CONF_REG_LSB   0x06
#define TEMP_MONITOR_DISABLE    0x02
#define TEMP_MONITOR_ENABLE     0x00

#define BCHGISE_BASE 0x03
#define BCHGISETDISCHARGE_MSB  0x0A
#define BCHGISETDISCHARGE_MSB_FULL 0x030A
#define BCHGISETDISCHARGE_LSB   0x0B
#define BCHGISETDISCHARGE_LSB_FULL 0x030B

#define BCHGISETCHARGE_MSB  0x08
#define BCHGISETCHARGE_MSB_FULL 0x0308
#define BCHGISETCHARGE_LSB      0x09
#define BCHGISETCHARGE_LSB_FULL 0x0309

#define BCHARGER_BASE 0x05
#define BCHARGER_MODE 0x10
#define BCHARGER_MODE_FULL 0x0510

#define BCHARGER_MODE_BITMASK 0b00001000


#define BCHGISETCHARGE_MSB_VALUE 0x26
#define BCHGISETCHARGE_LSB_VALUE 0x00

#define TASK_VBAT_MEASURE          0x0500
#define TASK_NTC_MEASURE           0x0501
#define TASK_TEMP_MEASURE          0x0502
#define TASK_VSYS_MEASURE          0x0503
#define TASK_IBAT_MEASURE          0x0506
#define TASK_VBUS_MEASURE          0x0507
#define TASK_DELAYED_VBAT_MEASURE  0x0508


#define ADC_IBAT_STATUS_MSB  0x0510
#define ADC_IBAT_STATUS_LSB  0x0511
#define ADC_VBAT_MSB         0x0511
#define ADC_NTC_MSB          0x0512
#define ADC_TEMP_MSB         0x0513
#define ADC_VSYS_MSB         0x0514
#define ADC_GP0_LSB          0x0515
#define ADC_VBUS_MSB         0x0519
#define ADC_GP1_LSB          0x051A


#define VFS_VBAT 5.0f
#define VFS_VSYS 6.375f
#define VFS_VBUS 7.0f
#define IFS_IBAT_HEAD_ROOM_DISCHARGE 1.12f
#define IFS_IBAT_HEAD_ROOM_CHARGE 1.25f

#define IBAT_LSB_A 0.00015625f   
#define TEMP_LSB_C 0.125f
#define TEMP_OFFSET_C -40.0f

#define BETA_NTC   3435.0f
#define T0_K       298.15f

static const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c2));
static struct k_work_delayable npm1300_work;

/* -------------------------------------------------------------------------- */
/* Utility Helpers                                                            */
/* -------------------------------------------------------------------------- */

static int npm1300_write_reg16(const struct device *i2c_dev,
                               uint8_t msb, uint8_t lsb, uint8_t val)
{
    uint8_t tx_buf[3] = { msb, lsb, val };
    return i2c_write(i2c_dev, tx_buf, sizeof(tx_buf), NPM1300_I2C_ADDR);
}


static int npm1300_write_task(uint16_t reg)
{
    uint8_t addr[2] = {
        (uint8_t)((reg >> 8) & 0xFF),
        (uint8_t)(reg & 0xFF)
    };
    uint8_t val = 0x01;
    uint8_t tx[3] = { addr[0], addr[1], val };
    return i2c_write(i2c_dev, tx, sizeof(tx), NPM1300_I2C_ADDR);
}


static int npm1300_read8(uint16_t reg, uint8_t *out)
{
    uint8_t addr[2] = {
        (uint8_t)((reg >> 8) & 0xFF),
        (uint8_t)(reg & 0xFF)
    };
    return i2c_write_read(i2c_dev, NPM1300_I2C_ADDR,
                          addr, sizeof(addr), out, 1);
}


static int npm1300_read16(uint16_t reg_msb, uint16_t reg_lsb, uint16_t *out)
{
    uint8_t addr_msb[2] = { (reg_msb >> 8) & 0xFF, reg_msb & 0xFF };
    uint8_t addr_lsb[2] = { (reg_lsb >> 8) & 0xFF, reg_lsb & 0xFF };
    uint8_t msb, lsb;

    if (i2c_write_read(i2c_dev, NPM1300_I2C_ADDR, addr_msb, 2, &msb, 1) < 0) return -1;
    if (i2c_write_read(i2c_dev, NPM1300_I2C_ADDR, addr_lsb, 2, &lsb, 1) < 0) return -1;

    *out = ((uint16_t)msb << 8) | lsb;
    return 0;
}


static float calc_tbat_c(uint16_t raw_ntc)
{
    if (raw_ntc == 0 || raw_ntc >= 1024) return 0.0f;
    float tbat_k = 1.0f / ((1.0f / T0_K) -
                   (1.0f / BETA_NTC) * logf((1024.0f / raw_ntc) - 1.0f));
    return tbat_k - 273.15f;
}

/* -------------------------------------------------------------------------- */
/* Temperature-monitor control                                                */
/* -------------------------------------------------------------------------- */

static int npm1300_disable_temp_monitor(void)
{
    if (!device_is_ready(i2c_dev)) return -ENODEV;
    int ret = npm1300_write_reg16(i2c_dev,
                                  BAT_TEMP_CONF_REG_MSB,
                                  BAT_TEMP_CONF_REG_LSB,
                                  TEMP_MONITOR_DISABLE);
    if (ret == 0)
        LOG_INF("NPM1300 temperature monitoring disabled");
    else
        LOG_WRN("Failed to disable temp monitor: %d", ret);
    return ret;
}

static int npm1300_enable_temp_monitor(void)
{
    if (!device_is_ready(i2c_dev)) return -ENODEV;
    int ret = npm1300_write_reg16(i2c_dev,
                                  BAT_TEMP_CONF_REG_MSB,
                                  BAT_TEMP_CONF_REG_LSB,
                                  TEMP_MONITOR_ENABLE);
    if (ret == 0)
        LOG_INF("NPM1300 temperature monitoring re-enabled");
    else
        LOG_WRN("Failed to enable temp monitor: %d", ret);
    return ret;
}

static int npm1300_temp_monitor_init(void)
{
    return npm1300_disable_temp_monitor();
}

/* -------------------------------------------------------------------------- */
/* ADC Read Workqueue                                                         */
/* -------------------------------------------------------------------------- */

static void npm1300_read_work(struct k_work *work)
{
    uint8_t vbat_m=0, ntc_m=0, temp_m=0, vsys_m=0, vbus_m=0;
    uint8_t gp0_lsb=0, gp1_lsb=0;
    uint16_t raw_vbat=0, raw_ntc=0, raw_temp=0, raw_vsys=0, raw_vbus=0;

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready in work handler");
        k_work_schedule(&npm1300_work, K_SECONDS(30));
        return;
    }

    npm1300_write_task(TASK_VBAT_MEASURE);
    k_msleep(10);
    npm1300_read8(ADC_VBAT_MSB, &vbat_m);

    npm1300_write_task(TASK_NTC_MEASURE);
    k_msleep(10);
    npm1300_read8(ADC_NTC_MSB, &ntc_m);

    npm1300_write_task(TASK_TEMP_MEASURE);
    k_msleep(10);
    npm1300_read8(ADC_TEMP_MSB, &temp_m);

    npm1300_write_task(TASK_VSYS_MEASURE);
    k_msleep(10);
    npm1300_read8(ADC_VSYS_MSB, &vsys_m);

    npm1300_read8(ADC_GP0_LSB, &gp0_lsb);

    npm1300_write_task(TASK_VBUS_MEASURE);
    k_msleep(10);
    npm1300_read8(ADC_VBUS_MSB, &vbus_m);
    npm1300_read8(ADC_GP1_LSB, &gp1_lsb);

    npm1300_write_task(TASK_IBAT_MEASURE);
    k_msleep(10);
    uint16_t raw_ibat_u16 = 0;
    npm1300_read16(ADC_IBAT_STATUS_MSB, ADC_IBAT_STATUS_LSB, &raw_ibat_u16);
    int16_t raw_ibat = (int16_t)raw_ibat_u16;
    
    raw_vbat = ((uint16_t)vbat_m << 2) | ((gp0_lsb >> 0) & 0x03);
    raw_ntc  = ((uint16_t)ntc_m  << 2) | ((gp0_lsb >> 2) & 0x03);
    raw_temp = ((uint16_t)temp_m << 2) | ((gp0_lsb >> 4) & 0x03);
    raw_vsys = ((uint16_t)vsys_m << 2) | ((gp0_lsb >> 6) & 0x03);
    raw_vbus = ((uint16_t)vbus_m << 2) | ((gp1_lsb >> 0) & 0x03);
    uint8_t charge_mode=0;

    npm1300_read8(BCHARGER_MODE_FULL, &charge_mode);

    uint8_t raw_battery_charge_msb = 0;
    uint8_t raw_battery_charge_lsb = 0;
    float raw_battery_charge_total = 0.0;
    float ibat_fs = 0.0;
    uint8_t after_mask = charge_mode & BCHARGER_MODE_BITMASK;
    if (after_mask > 0) {
        npm1300_read8(BCHGISETCHARGE_MSB_FULL, &raw_battery_charge_msb);
        npm1300_read8(BCHGISETCHARGE_LSB_FULL, &raw_battery_charge_lsb);
        raw_battery_charge_total = (float)raw_battery_charge_msb*4.0f + (float)raw_battery_charge_lsb*2.0f;
        ibat_fs = raw_battery_charge_total * IFS_IBAT_HEAD_ROOM_CHARGE;
    }
    else {
        npm1300_read8(BCHGISETDISCHARGE_MSB_FULL, &raw_battery_charge_msb);
        npm1300_read8(BCHGISETDISCHARGE_LSB_FULL, &raw_battery_charge_lsb);
        raw_battery_charge_total = (float)raw_battery_charge_msb*4.8f + (float)raw_battery_charge_lsb*2.4f;
        ibat_fs = raw_battery_charge_total * IFS_IBAT_HEAD_ROOM_DISCHARGE;
    }

    float ibat_a = ((float)raw_ibat / (float)3.2768e4) * ibat_fs;





    float vbat_v = ((float)raw_vbat / 1024.0f) * VFS_VBAT;
    float vsys_v = ((float)raw_vsys / 1024.0f) * VFS_VSYS;
    float vbus_v = ((float)raw_vbus / 1024.0f) * VFS_VBUS;
    float tbat_c = calc_tbat_c(raw_ntc);
    float temp_die_c = 394.67f - (0.7926f * raw_temp);

    snprintf(json_npm1300, sizeof(json_npm1300),
             "{\"vbat_v\":%.3f,\"vsys_v\":%.3f,\"vbus_v\":%.3f,"
             "\"ibat_a\":%.3f,\"temp_die_c\":%.1f,\"tbat_c\":%.1f}",
             vbat_v, vsys_v, vbus_v, ibat_a, temp_die_c, tbat_c);

    LOG_INF("%s", json_npm1300);
    k_work_schedule(&npm1300_work, K_SECONDS(30));
}

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

static int npm1300_init(void)
{
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }
    k_work_init_delayable(&npm1300_work, npm1300_read_work);
    k_work_schedule(&npm1300_work, K_NO_WAIT);
    return 0;
}

/* -------------------------------------------------------------------------- */
/* Shell Commands                                                             */
/* -------------------------------------------------------------------------- */

static int cmd_npm1300_disable_temp(const struct shell *shell,
                                    size_t argc, char **argv)
{
    int ret = npm1300_disable_temp_monitor();
    shell_print(shell, (ret == 0) ? "Temperature monitoring disabled" :
                                   "Disable failed: %d", ret);
    return ret;
}

static int cmd_npm1300_enable_temp(const struct shell *shell,
                                   size_t argc, char **argv)
{
    int ret = npm1300_enable_temp_monitor();
    shell_print(shell, (ret == 0) ? "Temperature monitoring enabled" :
                                   "Enable failed: %d", ret);
    return ret;
}

SHELL_CMD_REGISTER(disable_npm1300_temp, NULL,
                   "Disable temperature monitoring on NPM1300 PMIC",
                   cmd_npm1300_disable_temp);

SHELL_CMD_REGISTER(enable_npm1300_temp, NULL,
                   "Re-enable temperature monitoring on NPM1300 PMIC",
                   cmd_npm1300_enable_temp);

/* -------------------------------------------------------------------------- */

SYS_INIT(npm1300_init, APPLICATION, 3);
SYS_INIT(npm1300_temp_monitor_init, APPLICATION, 1);


static int npm1300_change_bat_current(void)
{
    if (!device_is_ready(i2c_dev)) return -ENODEV;

    int ret = 0;
    uint8_t config[3] = {BCHGISE_BASE,BCHGISETCHARGE_MSB,BCHGISETCHARGE_MSB_VALUE};
    ret = i2c_write(i2c_dev, config, sizeof(config), NPM1300_I2C_ADDR);
    if (ret < 0) {
        LOG_ERR("Failed to write BCHGISETDISCHARGE_MSB: %d", ret);
        return ret;
    }
    config[1] = BCHGISETCHARGE_LSB;
    config[0] = BCHGISETCHARGE_LSB_VALUE;

    ret = i2c_write(i2c_dev, config, sizeof(config), NPM1300_I2C_ADDR);
    if (ret < 0) {
        LOG_ERR("Failed to write BCHGISETCHARGE_LSB: %d", ret);
        return ret;
    }
    LOG_INF("NPM1300 battery current limits modified");
    return 0;

}

SYS_INIT(npm1300_change_bat_current, APPLICATION, 2);
