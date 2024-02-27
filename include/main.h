#ifndef MASTERTHESIS_MAIN_H
#define MASTERTHESIS_MAIN_H


#define LIS3DH_ADDR     0x18
#define LIS3DH_REG_CTRL1 0x20
#define LIS3DH_REG_CTRL2 0x21
#define LIS3DH_REG_CTRL3 0x22
#define LIS3DH_REG_CTRL4 0x23
#define LIS3DH_REG_CTRL5 0x24
#define LIS3DH_REG_REFERENCE 0x26
#define LIS3DH_REG_OUT_X_L 0x28
#define LIS3DH_REG_INT1_CFG 0x30
#define LIS3DH_REG_INT1_SRC 0x31
#define LIS3DH_REG_INT1_THS 0x32
#define LIS3DH_REG_INT1_DUR 0x33

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     22
#define I2C_SCL_PIN     20

typedef struct Acceleration {
    float x;
    float y;
    float z;
} Acceleration;


#endif //MASTERTHESIS_MAIN_H
