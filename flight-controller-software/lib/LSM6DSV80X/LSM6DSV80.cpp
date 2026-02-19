#include "LSM6DSV80X.h"

#define CLEAR        0x00  // To clear registers before writing

#define WHOAMI_REG   0x0F

#define ctrl1_xl     0x10
#define ctrl2_g      0x11
#define ctrl3        0x12  // SW reset at pin 1 and BDU at pin 6 (need BDU on)
#define ctrl6_g      0x15  // bw and full rate | BIT 4 NEEDS TO BE 1
#define ctrl8_xl     0x17  // bw and full rate
#define ctrl9_xl     0x18  // xl filtering and cal offset weighting

#define OUT_TEMP_L   0x20  // start of 14 byte output register sequence
#define OUTX_L_G     0x22  // start of gyro xlh>ylh>zlh
#define OUTX_L_A     0x28  // start of acc xlh>ylh>zlh



// ctrl1_xl
typedef enum
{
    LSM6DSV80X_XL_HIGH_PERFORM   = 0b0000000,  // default
    LSM6DSV80X_XL_HIGH_ACCURACY  = 0b0001000, 
    LSM6DSV80X_XL_NORMAL         = 0b0111000
} lsm6dsv80x_xl_op_mode_t;

// ctrl1_xl
typedef enum xl_odr
{
    LSM6DSV80X_XL_POWER_DOWN     = 0b00000000, // default
    LSM6DSV80X_XL_240_ODR        = 0b00000111,
    LSM6DSV80X_XL_480_ODR        = 0b00001000,
    LSM6DSV80X_XL_960_ORD        = 0b00001001
} lsm6dsv80x_xl_odr_t;

// ctrl2_g
enum class g_op_mode : uint8_t
{
    high_perform   = 0b00000000, // default
    high_accuracy  = 0b00010000
};

// ctrl2_g
enum class g_odr : uint8_t
{
    power_down     = 0b00000000, // default
    odr_240        = 0b00000111,
    odr_480        = 0b00001000,
    odr_960        = 0b00001001
};

// ctrl3
enum class ctrl3_enum : uint8_t
{
    bdu_en         = 0b01000000, // Only update low and high byte after both are read
    sw_reset       = 0b00000001  // Reset everything to default settings
};

// ctrl6_g
enum class lpf1_g : uint8_t
{
    bw_low         = 0b01011000, // 58  Hz at 960 ODR
    bw_med         = 0b01001000, // 100 Hz at 960 ODR  <--- Choosing This Default
    bw_high        = 0b00011000  // 310 Hz at 960 ODR
};

// ctrl6_g
enum class fs_g : uint8_t
{
    dps_250        = 0b00001001,
    dps_500        = 0b00001010,
    dps_1000       = 0b00001011,
    dps_2000       = 0b00001100,
    dps_4000       = 0b00001101
};

// ctrl8_xl - assumes LPF2_XL_EN enabled.
enum class lpf1_xl : uint8_t
{
    bw_ODR4         = 0b00000000,
    bw_ODR10        = 0b00100000,
    bw_ODR20        = 0b01000000,
    bw_ODR45        = 0b01100000,
    bw_ODR100       = 0b10000000,
    bw_ODR200       = 0b10100000,
    bw_ODR400       = 0b11000000,
    bw_ODR800       = 0b11100000
};

// ctrl8_xl
enum class fs_xl : uint8_t
{
    _2g            = 0b00000000,
    _4g            = 0b00000001,
    _8g            = 0b00000010,
    _16g           = 0b00000011
};

// ctrl9_xl
enum class ctrl9 : uint8_t
{
    lpf2_xl_en     = 0b00001000,
    usr_off_en     = 0b00000010  // FOR HG_XL | 0: 2e-10 g/LSB |  1: 2e-6 g/LSB 
};

//


bool LSM6DSV80X::begin() {

    _bus.begin(); // I2C or SPI depending on implementation

    // Reset Sensor Fully, Start Fresh, block for 1000 us
    uint8_t reset = (uint8_t)ctrl3_enum::sw_reset;
    _bus.write(ctrl3, reset);
    _time.delay_us(1000);


    uint8_t temp_reg = 0;
    uint8_t write_byte = 0;

    // Config Acc
    _bus.write(ctrl1_xl, (xl_op_mode::high_accuracy)|(xl_odr::odr_240));

    // Config Gyro 
    _bus.read();


    // need to configure default ranges and odr
    // GYRO Want 960 Hz ODR if SPI, 480 Hz ODR if I2C
    // Want ODR/2 Bandwidth for noise reduction
    // Gyro Range +/- 2000 dps
    // Accel Range of +/- 16g
}

// generic read, pass by ref IMU_Data ref for 3a,3g,1temp
bool LSM6DSV80X::read(IMU_Data& data) {

    const uint8_t BUF_LEN = 14;
    uint8_t buf[BUF_LEN] = {};

    // Get Timestamp
    uint32_t time = _time.now_us();

    // Read Data to Buffer
    _bus.read(OUT_TEMP_L, buf, BUF_LEN);

    // Assign data to struct
    data.temperature = (buf[1] << 8)  | buf[0];
    data.gx          = (buf[3] << 8)  | buf[2];
    data.gy          = (buf[5] << 8)  | buf[4];
    data.gz          = (buf[7] << 8)  | buf[6];
    data.ax          = (buf[9] << 8)  | buf[8];
    data.ay          = (buf[11] << 8) | buf[10];
    data.az          = (buf[13] << 8) | buf[12];
    data.time        = time;
}