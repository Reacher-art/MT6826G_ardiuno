#include <Arduino.h>
#include <SPI.h>

// 定义MT6826S的SPI引脚
#define MT6826S_SCK 18
#define MT6826S_MISO 19
#define MT6826S_MOSI 23
#define MT6826S_CS 5
#define _2PI 6.28318530719f

// 定义MT6826S的寄存器地址
#define MT6826S_ANGLE_REG_START 0x03
#define MT6826S_ANGLE_REG_END 0x06

//CRC校验
uint8_t crc8_24bit(uint8_t b0, uint8_t b1, uint8_t b2) {
  const uint8_t poly = 0x07;
  uint8_t crc = 0x00;                // 初始值
  uint8_t data[3] = {b0, b1, b2};    // 0x003, 0x004, 0x005 的顺序
  for (uint8_t i = 0; i < 3; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 0x80)
        crc = (crc << 1) ^ poly;
      else
        crc <<= 1;
    }
  }
  return crc;   // 最终 CRC 值
}

// 定义SPI设置
const int SPI_CLOCK_SPEED = 1000000;  // 降低时钟速度
const int SPI_MODE = SPI_MODE3;  // 使用SPI模式3


void printBinary(uint16_t value) {
  int numBits = 16; // 假设整数为32位
  for (int i = numBits - 1; i >= 0; i--) {
    Serial.print((value >> i) & 0x01); // 打印每一位的二进制值
    if (i % 4 == 0) Serial.print(' '); // 每四位添加一个空格，便于阅读
  }
  Serial.println(); // 打印换行符
}

// 初始化SPI
void initSPI() {
    SPI.begin(MT6826S_SCK, MT6826S_MISO, MT6826S_MOSI, MT6826S_CS);
    SPI.beginTransaction(SPISettings(SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE));
}

// 发送SPI命令并读取数据
uint8_t spiTransfer(uint8_t command, uint8_t addr) {
    
    uint8_t data = 0;
    uint16_t temp = (command << 8) | addr;
    digitalWrite(MT6826S_CS, LOW);
    SPI.transfer16(temp);
    data = SPI.transfer(addr);
    digitalWrite(MT6826S_CS, HIGH);
    
    return data;
}

// 读取MT6826S的角度数据
uint16_t readAngleMT6826S() {
    uint8_t angleData[4];
    uint8_t command = 0x30;
    uint8_t crc = 0X00;
    // 读取角度寄存器0x03到0x06的数据
    //do {
      for (uint8_t i = 0; i < 4; i++) {
          //uint8_t command = 0x30;
          uint8_t addr = (MT6826S_ANGLE_REG_START + i); // 构造读取命令
          //printBinary(command);
          digitalWrite(MT6826S_CS, HIGH);
          angleData[i] = spiTransfer(command, addr);
      }      
    //} while (crc8_24bit(angleData[0], angleData[1], angleData[2]) != angleData[3]);
    //Serial.print("2\n"); 
    // 计算角度
    uint16_t rawAngle = 0;
    rawAngle = (angleData[0] << 8) | (angleData[1]);
    rawAngle >>= 1;
    printBinary(rawAngle);

    return rawAngle;
}

void setup() {
    Serial.begin(115200); 
    pinMode(MT6826S_CS, OUTPUT);
    digitalWrite(MT6826S_CS, HIGH);
    initSPI();
}

void loop() {
    float angle = (readAngleMT6826S()* _2PI) / 32767;
    //Serial.print("Angle: ");
    Serial.println(angle, 2); // 保留一位小数
    delay(100); // 每秒读取一次
}
