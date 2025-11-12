#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"
#include "driver/gpio.h"
#include "esp_log.h"

// I2C / PCA9685
#define I2C_PORT    I2C_NUM_0
#define I2C_SDA     21
#define I2C_SCL     22
#define PCA_ADDR    0x70

#define MODE1       0x00
#define MODE2       0x01
#define PRESCALE    0xFE
#define LED0_ON_L   0x06

// Joysticks (GPIOs) 
#define J1_X_GPIO   34  // Base (L/R)
#define J1_Y_GPIO   35  // Shoulder (U/D)
#define J2_X_GPIO   32  // Wrist (L/R)
#define J2_Y_GPIO   33  // Elbow (U/D)
#define J1_SW_GPIO  25
#define J2_SW_GPIO  26

// PCA9685 channels (your board: P# = ch+1) 
#define CH_GRIPPER   4  // P5
#define CH_WRIST     6  // P7
#define CH_ELBOW     7  // P8
#define CH_BASE      8  // P9
#define CH_SHOULDER  9  // P10

//  Servo parameters
#define SERVO_MIN_US    600
#define SERVO_MAX_US    2400
#define SERVO_HOME_US   1500
#define LOOP_MS         20

// Max speed at full stick (µs/s)
#define RATE_BASE_US_S      1200
#define RATE_SHOULDER_US_S  1200
#define RATE_ELBOW_US_S     1200
#define RATE_WRIST_US_S     1200

// Gripper
#define GRIPPER_HOME_US     1200   // OPEN (default)
#define GRIPPER_ACTIVE_US   2000   // CLOSED (set 1000 if inverted)

// Deadzone & calibration
#define DEADZONE_RAW   100
#define CAL_SAMPLES    200

//  PCA9685 helpers 
static esp_err_t w8(uint8_t reg, uint8_t val){
    uint8_t b[2] = {reg, val};
    return i2c_master_write_to_device(I2C_PORT, PCA_ADDR, b, 2, pdMS_TO_TICKS(200));
}
static esp_err_t r8(uint8_t reg, uint8_t *val){
    return i2c_master_write_read_device(I2C_PORT, PCA_ADDR, &reg, 1, val, 1, pdMS_TO_TICKS(200));
}
static void pca_pwm(uint8_t ch, uint16_t on, uint16_t off){
    uint8_t base = LED0_ON_L + 4U * ch;
    uint8_t buf[5] = { base, (uint8_t)(on & 0xFF), (uint8_t)(on >> 8),
                              (uint8_t)(off & 0xFF), (uint8_t)(off >> 8) };
    (void)i2c_master_write_to_device(I2C_PORT, PCA_ADDR, buf, 5, pdMS_TO_TICKS(200));
}
static void pca_us(uint8_t ch, int us){
    if (us < SERVO_MIN_US) {
        us = SERVO_MIN_US;
    } else if (us > SERVO_MAX_US) {
        us = SERVO_MAX_US;
    }
    uint16_t ticks = (uint16_t)((us / 20000.0f) * 4096.0f); // 50 Hz → 20 ms
    pca_pwm(ch, 0, ticks);
}
static void pca_init_50hz(void){
    (void)w8(MODE2, 0x04); 
    uint8_t old = 0; (void)r8(MODE1, &old);
    (void)w8(MODE1, (uint8_t)((old & 0x7F) | 0x10)); 
    float prescale = (25000000.0f / (4096.0f * 50.0f)) - 1.0f;
    (void)w8(PRESCALE, (uint8_t)(prescale + 0.5f));
    (void)w8(MODE1, (uint8_t)(old & ~0x10));         
    vTaskDelay(pdMS_TO_TICKS(5));
    (void)w8(MODE1, (uint8_t)((old & ~0x10) | 0xA1)); 
}

// ---------- ADC One-Shot ----------
static adc_oneshot_unit_handle_t adc1;
static const adc_channel_t CH32_ADC = ADC_CHANNEL_4; // GPIO32
static const adc_channel_t CH33_ADC = ADC_CHANNEL_5; // GPIO33
static const adc_channel_t CH34_ADC = ADC_CHANNEL_6; // GPIO34
static const adc_channel_t CH35_ADC = ADC_CHANNEL_7; // GPIO35

static void adc_init(void){
    adc_oneshot_unit_init_cfg_t u = { .unit_id = ADC_UNIT_1, .ulp_mode = ADC_ULP_MODE_DISABLE };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&u, &adc1));
    adc_oneshot_chan_cfg_t c = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, CH32_ADC, &c));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, CH33_ADC, &c));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, CH34_ADC, &c));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1, CH35_ADC, &c));
}
static inline int adc_read(adc_channel_t ch){
    int v = 0; ESP_ERROR_CHECK(adc_oneshot_read(adc1, ch, &v)); return v;
}


static float axis_norm(int raw, int center, bool invert){
    int v = raw - center;
    if (v > -DEADZONE_RAW && v < DEADZONE_RAW) v = 0;
    float n = (float)v / 2048.0f;
    if (n > 1.0f) n = 1.0f; else if (n < -1.0f) n = -1.0f;
    return invert ? -n : n;
}

void app_main(void){
    // I2C + PCA
    i2c_config_t ic = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA, .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &ic));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
    pca_init_50hz();

    // Buttons
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<J1_SW_GPIO) | (1ULL<<J2_SW_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    // ADC + center calibration
    adc_init();
    int cJ1X=0, cJ1Y=0, cJ2X=0, cJ2Y=0;
    for (int i=0;i<CAL_SAMPLES;i++){
        cJ1X += adc_read(CH34_ADC); // J1X
        cJ1Y += adc_read(CH35_ADC); // J1Y
        cJ2X += adc_read(CH33_ADC); // J2X
        cJ2Y += adc_read(CH32_ADC); // J2Y
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    cJ1X/=CAL_SAMPLES; cJ1Y/=CAL_SAMPLES; cJ2X/=CAL_SAMPLES; cJ2Y/=CAL_SAMPLES;

    // Start positions (hold)
    float base_us=SERVO_HOME_US, shoulder_us=2200, elbow_us=1200, wrist_us=SERVO_HOME_US;
    pca_us(CH_BASE, base_us); pca_us(CH_SHOULDER, shoulder_us);
    pca_us(CH_ELBOW, elbow_us); pca_us(CH_WRIST, wrist_us);

    //  Gripper toggle: default OPEN, toggle on both-buttons rising edge 
    bool gripper_closed = false;      // false=open, true=closed
    bool prev_both = false;
    int  gripper_us = GRIPPER_HOME_US;
    pca_us(CH_GRIPPER, gripper_us);

    const float dt = (float)LOOP_MS / 1000.0f;

    while(1){
        // Read sticks (invert Y so up=+)
        float j1x = axis_norm(adc_read(CH34_ADC), cJ1X, true);  // Base
        float j1y = axis_norm(adc_read(CH35_ADC), cJ1Y, true);  // Shoulder
        float j2x = axis_norm(adc_read(CH32_ADC), cJ2X, true);  // Wrist
        float j2y = axis_norm(adc_read(CH33_ADC), cJ2Y, true);  // Elbow

        bool b1 = (gpio_get_level(J1_SW_GPIO)==0);
        bool b2 = (gpio_get_level(J2_SW_GPIO)==0);
        bool both_pressed = b1 && b2;

        // Toggle gripper on rising edge (press & release both buttons)
        if (both_pressed && !prev_both) {
            gripper_closed = !gripper_closed;
        }
        prev_both = both_pressed;
        gripper_us = gripper_closed ? GRIPPER_ACTIVE_US : GRIPPER_HOME_US;

        // Integrate (position-hold)
        base_us     += j1x * RATE_BASE_US_S     * dt;
        shoulder_us += j1y * RATE_SHOULDER_US_S * dt;
        elbow_us    += j2y * RATE_ELBOW_US_S    * dt;
        wrist_us    += j2x * RATE_WRIST_US_S    * dt;

        // Clamp
        if (base_us < SERVO_MIN_US) { base_us = SERVO_MIN_US; }
        else if (base_us > SERVO_MAX_US) { base_us = SERVO_MAX_US; }

        if (shoulder_us < SERVO_MIN_US) { shoulder_us = SERVO_MIN_US; }
        else if (shoulder_us > SERVO_MAX_US) { shoulder_us = SERVO_MAX_US; }

        if (elbow_us < SERVO_MIN_US) { elbow_us = SERVO_MIN_US; }
        else if (elbow_us > SERVO_MAX_US) { elbow_us = SERVO_MAX_US; }

        if (wrist_us < SERVO_MIN_US) { wrist_us = SERVO_MIN_US; }
        else if (wrist_us > SERVO_MAX_US) { wrist_us = SERVO_MAX_US; }

        // Drive
        pca_us(CH_BASE,     (int)base_us);
        pca_us(CH_SHOULDER, (int)shoulder_us);
        pca_us(CH_ELBOW,    (int)elbow_us);
        pca_us(CH_WRIST,    (int)wrist_us);
        pca_us(CH_GRIPPER,  (int)gripper_us);

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}
