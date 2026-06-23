#include "mpu6050.h"
#include <math.h>

// 微秒級延遲 (粗略模擬，72MHz 主頻下約略估算)
static void I2C_Delay(void)
{
    volatile int i = 15;
    while(i--);
}

// 軟體 I2C 初始化
void MPU6050_I2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // 配置 PB8(SCL) 與 PB9(SDA) 為開漏輸出 (Open-Drain)
    GPIO_InitStruct.Pin = SCL_PIN | SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // [新增] I2C 匯流排復位邏輯
    // 如果 SDA 被 Slave 拉低（卡死），傳送 9 個時鐘脈衝通常能強迫 Slave 釋放 SDA
    I2C_SDA_HIGH();
    for (int i = 0; i < 9; i++) {
        I2C_SCL_LOW();
        I2C_Delay(); 
        I2C_SCL_HIGH();
        I2C_Delay();
    }

    I2C_SCL_HIGH();
    I2C_SDA_HIGH();
}

// I2C 起始訊號
static void I2C_Start(void)
{
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
    I2C_Delay();
    I2C_SDA_LOW();
    I2C_Delay();
    I2C_SCL_LOW();
    I2C_Delay();
}

// I2C 停止訊號
static void I2C_Stop(void)
{
    I2C_SDA_LOW();
    I2C_SCL_HIGH();
    I2C_Delay();
    I2C_SDA_HIGH();
    I2C_Delay();
}

// I2C 發送應答 (Ack)
static void I2C_SendAck(uint8_t ack)
{
    if (ack) I2C_SDA_HIGH(); // NACK
    else I2C_SDA_LOW();      // ACK
    I2C_Delay();
    I2C_SCL_HIGH();
    I2C_Delay();
    I2C_SCL_LOW();
    I2C_Delay();
}

// I2C 等待應答
static uint8_t I2C_WaitAck(void)
{
    uint8_t retry = 0;
    I2C_SDA_HIGH();
    I2C_Delay();
    I2C_SCL_HIGH();
    I2C_Delay();
    while (I2C_SDA_READ()) {
        retry++;
        if (retry > 250) {
            I2C_Stop();
            return 1; // 超時無應答
        }
    }
    I2C_SCL_LOW();
    I2C_Delay();
    return 0;
}

// I2C 發送一個位元組
static void I2C_WriteByte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80) I2C_SDA_HIGH();
        else I2C_SDA_LOW();
        byte <<= 1;
        I2C_Delay();
        I2C_SCL_HIGH();
        I2C_Delay();
        I2C_SCL_LOW();
        I2C_Delay();
    }
}

// I2C 讀取一個位元組
static uint8_t I2C_ReadByte(void)
{
    uint8_t byte = 0;
    I2C_SDA_HIGH();
    for (uint8_t i = 0; i < 8; i++) {
        byte <<= 1;
        I2C_SCL_HIGH();
        I2C_Delay();
        if (I2C_SDA_READ()) byte |= 0x01;
        I2C_SCL_LOW();
        I2C_Delay();
    }
    return byte;
}

// 寫入 MPU6050 暫存器
uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    I2C_Start();
    I2C_WriteByte(MPU6050_ADDR);
    if (I2C_WaitAck()) return 1;
    I2C_WriteByte(reg);
    if (I2C_WaitAck()) return 1;
    I2C_WriteByte(data);
    if (I2C_WaitAck()) return 1;
    I2C_Stop();
    return 0;
}

// 讀取 MPU6050 暫存器
uint8_t MPU6050_ReadReg(uint8_t reg)
{
    uint8_t data = 0;
    I2C_Start();
    I2C_WriteByte(MPU6050_ADDR);
    if (I2C_WaitAck()) return 0xFF; // 出錯返回 0xFF 以區別合法數據
    I2C_WriteByte(reg);
    if (I2C_WaitAck()) return 0xFF;
    
    I2C_Start(); // 重複起始訊號
    I2C_WriteByte(MPU6050_ADDR | 0x01); // 讀取指令
    if (I2C_WaitAck()) return 0xFF;
    data = I2C_ReadByte();
    I2C_SendAck(1); // 發送 NACK 結束
    I2C_Stop();
    return data;
}

// MPU6050 初始化
uint8_t MPU6050_Init(void)
{
    uint8_t id = 0;
    MPU6050_I2C_Init();
    
    // [修改] 增加電源穩定等待時間
    HAL_Delay(500);
    
    for (int retry = 0; retry < 5; retry++) {
        id = MPU6050_ReadReg(MPU6050_WHO_AM_I);
        // 放寬檢查：只要不是 0xFF (通訊失敗) 且不是 0 (沒讀到)，就試著啟動
        if (id != 0xFF && id != 0x00) {
            MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);
            MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07);
            MPU6050_WriteReg(MPU6050_CONFIG, 0x03);
            MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
            MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);
            
            HAL_Delay(50);
            return 0; 
        }
        HAL_Delay(100);
    }
    
    return 1;
}

// 批次讀取加速度計與陀螺儀原始資料
void MPU6050_Read_Data(short *accel, short *gyro)
{
    uint8_t buf[14];
    
    I2C_Start();
    I2C_WriteByte(MPU6050_ADDR);
    I2C_WaitAck();
    I2C_WriteByte(MPU6050_ACCEL_XOUT_H);
    I2C_WaitAck();
    
    I2C_Start();
    I2C_WriteByte(MPU6050_ADDR | 0x01);
    I2C_WaitAck();
    
    for (int i = 0; i < 13; i++) {
        buf[i] = I2C_ReadByte();
        I2C_SendAck(0); // ACK
    }
    buf[13] = I2C_ReadByte();
    I2C_SendAck(1); // NACK
    I2C_Stop();
    
    // 組合高低位元組
    accel[0] = (buf[0] << 8) | buf[1];    // Accel X
    accel[1] = (buf[2] << 8) | buf[3];    // Accel Y
    accel[2] = (buf[4] << 8) | buf[5];    // Accel Z
    
    gyro[0] = (buf[8] << 8) | buf[9];     // Gyro X
    gyro[1] = (buf[10] << 8) | buf[11];   // Gyro Y
    gyro[2] = (buf[12] << 8) | buf[13];   // Gyro Z
}

// 互補濾波演算法獲取姿態角
void MPU6050_Get_Angle(float *Pitch, float *Roll, float dt)
{
    short accel[3], gyro[3];
    float accel_pitch, accel_roll;
    float gyro_x_rate, gyro_y_rate;
    
    MPU6050_Read_Data(accel, gyro);
    
    // 改用 Y 軸控制前後 (Pitch)
    accel_pitch = atan2f((float)-accel[0], (float)accel[2]) * 180.0f / 3.1415926f;
    accel_roll  = atan2f((float)accel[1], (float)accel[2]) * 180.0f / 3.1415926f;
    
    gyro_x_rate = (float)gyro[0] / 16.4f;
    gyro_y_rate = (float)gyro[1] / 16.4f;
    
    // 互補濾波 (Complementary Filter) 融合角度
    *Pitch = 0.98f * (*Pitch + gyro_y_rate * dt) + 0.02f * accel_pitch;
    *Roll  = 0.98f * (*Roll + gyro_x_rate * dt) + 0.02f * accel_roll;
}
