#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#include <Arduino.h>

#define ULTRASOUND_I2C_ADDR 0x77 

//寄存器
#define DISDENCE_L    0//距离低8位，单位mm
#define DISDENCE_H    1

#define RGB_BRIGHTNESS  50//0-255

#define RGB_WORK_MODE 2//RGB灯模式，0：用户自定义模式   1：呼吸灯模式  默认0

#define RGB1_R      3//1号探头的R值，0~255，默认0
#define RGB1_G      4//默认0
#define RGB1_B      5//默认255

#define RGB2_R      6//2号探头的R值，0~255，默认0
#define RGB2_G      7//默认0
#define RGB2_B      8//默认255

#define RGB1_R_BREATHING_CYCLE      9 //呼吸灯模式时，1号探头的R的呼吸周期，单位100ms 默认0，
                                      //如果设置周期3000ms，则此值为30
#define RGB1_G_BREATHING_CYCLE      10
#define RGB1_B_BREATHING_CYCLE      11

#define RGB2_R_BREATHING_CYCLE      12//2号探头
#define RGB2_G_BREATHING_CYCLE      13
#define RGB2_B_BREATHING_CYCLE      14

#define RGB_WORK_SIMPLE_MODE    0
#define RGB_WORK_BREATHING_MODE   1

class Ultrasound {
  public:
    Ultrasound();
    bool wireWriteByte(uint8_t addr, uint8_t val);
    bool wireWriteDataArray(uint8_t addr, uint8_t reg,uint8_t *val,unsigned int len);
    int wireReadDataArray(uint8_t addr, uint8_t reg, uint8_t *val, unsigned int len);
    
    void Breathing(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2);
    void Color(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2);
    int Filter();
    u16 GetDistance();
};
#endif
