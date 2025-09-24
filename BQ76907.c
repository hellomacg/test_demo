/*
 * BQ76907.c
 *
 *  Created on: 2025年8月7日
 *      Author: maode1
 */

#include "BQ76907.h"
#include "Lpi2c_Ip.h"
#include "Lpi2c_Ip_PBcfg.h"
#include "string.h"
#include "Siul2_Port_Ip.h"
#include "Siul2_Dio_Ip.h"
#include "IntCtrl_Ip.h"
#define I2C_MASTER          0

void Bq7808_delay(uint32_t count)
{
    volatile uint32_t i = 0;
    for (i = 0; i < count; ++i)
    {
        __asm("NOP"); /* 调用nop空指令 */
         __asm("NOP"); /* 调用nop空指令 */
          __asm("NOP"); /* 调用nop空指令 */
           __asm("NOP"); /* 调用nop空指令 */
            __asm("NOP"); /* 调用nop空指令 */
    }
}
// Calculate checksum for RAM writes
unsigned char Checksum(unsigned char *ptr, unsigned char len) {
  unsigned char i;
  unsigned char checksum = 0;

  for (i = 0; i < len; i++)
    checksum += ptr[i];

  checksum = 0xff & ~checksum;

  return (checksum);
}
/**
 * @brief 向 BQ76907 寄存器写入任意长度数据
 * @param regAddr  寄存器地址
 * @param pData    要写入的数据数组指针
 * @param len      数据长度（字节数）
 * @return true  写入成功
 * @return false 写入失败（参数错误/I2C超时/通信错误）
 */
bool BQ76907_WriteReg(uint8_t regAddr, const uint8_t *pData, uint8_t len)
{
    Lpi2c_Ip_StatusType err = LPI2C_IP_SUCCESS_STATUS;
    uint32_t timeout = 0xFFFF;
    uint8_t buf[len + 1];  // 缓冲区 = 寄存器地址 + 数据

    // 1. 参数校验
    if (pData == NULL || len == 0) {
        return false;
    }

    // 2. 准备缓冲区：寄存器地址 + 数据
      buf[0] = regAddr;                // 首字节为寄存器地址
      memcpy(&buf[1], pData, len);     // 剩余字节为数据
    // 3. 设置从机地址（BQ76907）
    Lpi2c_Ip_MasterSetSlaveAddr(Bq76907_I2cInst, Bq76907_ADD, false);

    // 4. 执行I2C写入（写入 len+1 字节：地址 + 数据）
    err = Lpi2c_Ip_MasterSendData(Bq76907_I2cInst, buf, len + 1, true);
    if (err != LPI2C_IP_SUCCESS_STATUS) {
        return false;
    }

    // 5. 等待传输完成（带超时）
    while (Lpi2c_Ip_MasterGetTransferStatus(Bq76907_I2cInst, NULL_PTR) == LPI2C_IP_BUSY_STATUS) {
        if (timeout-- == 0) {
            return false;
        }
    }

    return true;
}

/**
 * @brief 从 BQ76907 寄存器读取数据
 * @param regAddr  寄存器地址
 * @param pData    存储读取数据的缓冲区指针
 * @param len      要读取的字节数（1~N）
 * @return true    读取成功
 * @return false   读取失败（参数错误/I2C超时/通信错误）
 */
bool BQ76907_ReadReg(uint8_t regAddr, uint8_t *pData, uint8_t len)
{
    Lpi2c_Ip_StatusType err = LPI2C_IP_SUCCESS_STATUS;
    uint16_t timeout = 0xFFFF;

    // 1. 参数校验
    if (pData == NULL || len == 0) {
        return false;
    }

    // 2. 设置从机地址（BQ76907）
    Lpi2c_Ip_MasterSetSlaveAddr(Bq76907_I2cInst, Bq76907_ADD, false);

    // 3. 发送寄存器地址（不带STOP条件）
    err = Lpi2c_Ip_MasterSendData(Bq76907_I2cInst, &regAddr, 1, false);
    if (err != LPI2C_IP_SUCCESS_STATUS) {
        return false;
    }

    // 4. 等待地址发送完成
    while ((Lpi2c_Ip_MasterGetTransferStatus(Bq76907_I2cInst, NULL_PTR) == LPI2C_IP_BUSY_STATUS) && (timeout > 0)) {
       timeout--;
    }
    if (timeout == 0) {
        return false;
    }

    // 5. 读取数据（带STOP条件）
    timeout = 0xFFFF;
    err = Lpi2c_Ip_MasterReceiveData(Bq76907_I2cInst, pData, len, true);
    if (err != LPI2C_IP_SUCCESS_STATUS) {
        return false;
    }

    // 6. 等待读取完成
    while ((Lpi2c_Ip_MasterGetTransferStatus(Bq76907_I2cInst, NULL_PTR) == LPI2C_IP_BUSY_STATUS) && (timeout > 0)) {
        timeout--;
    }

    return (timeout > 0);
}

// See the TRM or the BQ7690x header file for a full list of Direct Commands
uint16_t DirectCommands(uint8_t command, uint16_t data, uint8_t type)
{

  // type: R = read, W = write
  uint8_t TX_data[2] = {0x00, 0x00};
  uint8_t RX_data[2] = {0x00, 0x00};
  uint16_t result = 0xffff;
  // little endian format
  TX_data[0] = data & 0xff;
  TX_data[1] = (data >> 8) & 0xff;

  if (type == R)
  {                 // Read
	  BQ76907_ReadReg(command, RX_data, 1); // RX_data is a global variable
	    result = RX_data[0];
	    return result;
  }
  if (type == R2)
  {                 // Read
	  BQ76907_ReadReg(command, RX_data, 2); // RX_data is a global variable
	    result = (RX_data[1] << 8) | RX_data[0];
	    return result;
  }
  if (type == W)
  { // write
	  BQ76907_WriteReg(command, TX_data, 1);
  }
  if (type == W2)
  { // write
	  BQ76907_WriteReg(command, TX_data, 2);
  }
}

// See the TRM or the BQ7690x header file for a full list of Subcommands
void Subcommands(uint16_t command, uint16_t data, uint8_t type)
{

  // type: R = read, W = write 1 byte, W2 = write 2 bytes
  // max readback size is 32 bytes
  uint8_t TX_Reg[4] = {0x00, 0x00, 0x00, 0x00};
  uint8_t TX_Buffer[2] = {0x00, 0x00};

  // TX_Reg in little endian format
  TX_Reg[0] = command & 0xff;
  TX_Reg[1] = (command >> 8) & 0xff;


  if (type == W)
  { // write
    // CB_Active_Cells, PROT_RECOVERY
    TX_Reg[2] = data & 0xff;
    BQ76907_WriteReg(0x3E, TX_Reg, 3);
    TX_Buffer[0] = Checksum(TX_Reg, 3);
    TX_Buffer[1] = 0x05; // combined length of registers address and data
    BQ76907_WriteReg(0x60, TX_Buffer, 2);
  }

  else if (type == W2)
  { // write data with 2 bytes
    TX_Reg[2] = data & 0xff;
    TX_Reg[3] = (data >> 8) & 0xff;
    BQ76907_WriteReg(0x3E, TX_Reg, 4);
    TX_Buffer[0] = Checksum(TX_Reg, 4);
    TX_Buffer[1] = 0x06; // combined length of registers address and data
    BQ76907_WriteReg(0x60, TX_Buffer, 2);
  }
}

// See the TRM or the BQ769x0 header file for a full list of Command-only subcommands
void CommandSubcommands(uint16_t command)
{
        //Note: For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively
	uint8_t TX_Reg[2] = {0x00, 0x00};

	//TX_Reg in little endian format
	TX_Reg[0] = command & 0xff;
	TX_Reg[1] = (command >> 8) & 0xff;

	BQ76907_WriteReg(0x3E,TX_Reg,2);
}

/**
 * @brief 读取指定电芯的电压值
 * @param cell 电芯编号（通常为0~N-1，取决于电池组串联数）
 * @return uint16_t 电压值（单位为mV，具体分辨率需参考器件手册）
 *                 返回0xFFFF表示读取失败
 */
int16_t ReadCellVoltage(uint8_t cell)
{
    uint8_t cmd_addr;
    int16_t result = 0xFFFF; // 默认错误值

    // 1. 参数校验（假设最大支持15节电芯，根据实际调整）
    if (cell > 6) {
        return result; // 电芯编号超出范围
    }

    // 2. 计算寄存器地址（每电芯占2字节，起始地址0x14）
    cmd_addr = 0x14 + (cell * 2);

    result = DirectCommands(cmd_addr,0x00,R2);

    return result;
}

/**
 * @brief 开启指定电芯的均衡功能
 * @param Cell 电芯选择位掩码：
 *             0x00 - 禁止均衡
 *             0x02 - 电芯1
 *             0x04 - 电芯2
 *             0x06 - 电芯3
 *             ...（支持多电芯组合，如0x07表示电芯1+2+3）
 * @return true  操作成功
 * @return false 操作失败
 */
bool Cell_Balancing(uint8_t Cell)
{
	Cell = Cell|0x01;
	Subcommands(0x0083, Cell, W);

    return true;
}
/**
 * @brief 检查已开启均衡功能的电芯
 * @return 电芯编号
 */
uint8_t Cell_Balancing_cbk(void)
{
	uint8_t Balancing_cbk=0;
	CommandSubcommands(0x0083);
	Balancing_cbk = DirectCommands(0X40,0x00,R);

    return Balancing_cbk;
}
/**
 * @brief 读取内部温度传感器值（单位：1°C）
 * @return int16_t 温度值（范围：-400~1250，对应-40.0°C~125.0°C）
 *                 返回0x8000表示读取失败
 */
int16_t Read_Int_Temperature(void)
{
    uint8_t data[2] = {0};
    int16_t temperature = 0x7FFF; // 默认错误值

    temperature = DirectCommands(0x28, 0x00, R2);

    // 3. 温度值有效性检查（可选）
    if ((temperature < -400) || (temperature > 1250)) {
        temperature = 0x7FFF; // 数据超出合理范围视为错误
    }

    return temperature;
}

/*ADC measurement of the TS pin.*/
int16_t Read_Ext_Temperature(void)
{
    uint8_t data[2] = {0};
    int16_t temperature = 0x7FFF; // 默认错误值

    temperature = DirectCommands(0x2A, 0x00, R2);

    // 3. 温度值有效性检查（可选）
    if ((temperature < -400) || (temperature > 1250)) {
        temperature = 0x7FFF; // 数据超出合理范围视为错误
    }

    return temperature;
}
/**
 * @brief 读取CC1电流值（单位：mA）
 * @return int16_t 电流值（正值表示充电，负值表示放电）
 *                返回0x8000表示读取失败
 * @note 假设：
 *       1. 寄存器0x3C返回16位有符号补码
 *       2. 电流分辨率：1mA/LSB
 *       3. 量程：±32767mA
 */
int16_t Read_CC1_Current(void)
{
    int16_t current = 0x7FFF; // 默认错误值

    current = DirectCommands(0x3C,0x00,R2);
    current = current/5;
    return current;
}

/**
 * @brief 读取CC2电流值（单位：mA）
 * @return int16_t 电流值（正值表示充电，负值表示放电）
 *                返回0x8000表示读取失败
 * @note 假设：
 *       1. 寄存器0x3C返回16位有符号补码
 *       2. 电流分辨率：1mA/LSB
 *       3. 量程：±32767mA
 */
int16_t Read_CC2_Current(void)
{
    int16_t current = 0x7FFF; // 默认错误值

    current = DirectCommands(0x3A,0x00,R2);
    current = current/5;
    return current;
}

/*16-bit voltage on top of stack*/
uint16_t Stack_Voltage(void)
{
	uint16_t result = 0x0000;
    result = DirectCommands(0x26,0x00,R2);
    return result;
}

/*Measurement of VSS using ADC, used for diagnostic of ADC input mux*/
uint16_t VSS_Voltage(void)
{
	uint16_t result = 0x0000;
    result = DirectCommands(0x24,0x00,R2);
    return result;
}

/*Provides individual fault signals when enabled safety faults have triggered.
Bit descriptions can be found in Safety Status A Register*/
uint8_t Safety_StatusA(void)
{
	uint8_t result = 0x00;
    result = DirectCommands(0x03,0x00,R);
    return result;
}

/*Provides individual fault signals when enabled safety faults have triggered.
Bit descriptions can be found in Safety Status B Register*/
uint8_t Safety_StatusB(void)
{
	uint8_t result = 0x00;
    result = DirectCommands(0x05,0x00,R);
    return result;
}

uint8_t Safety_AlertA(void)
{
	uint8_t result = 0x00;
    result = DirectCommands(0x02,0x00,R);
    return result;
}
uint8_t Safety_AlertB(void)
{
	uint8_t result = 0x00;
    result = DirectCommands(0x04,0x00,R);
    return result;
}
uint16_t Alarm_Status(void)
{
	uint16_t result = 0x0000;
    result = DirectCommands(0x62,0x00,R);
    return result;
}
uint16_t Alarm_Raw_Status(void)
{
	uint16_t result = 0x0000;
    result = DirectCommands(0x64,0x00,R);
    return result;
}
uint16_t Battery_Status(void)
{
	uint16_t result = 0x0000;
    result = DirectCommands(0x12,0x00,R);
    return result;
}

/**
 * @brief 允许或禁止休眠
 * @param enable 控制标志: true=允许休眠, false=禁止休眠
 * @return true 操作成功
 * @return false 操作失败
 */
bool Sleep_enable(bool enable)
{
	uint16 RxData = 0;
	if(enable)
	{
		/*This command is sent to allow the device to enter SLEEP mode*/
		CommandSubcommands(0x0099);
		RxData = DirectCommands(0x62,0x00,R);
		RxData = RxData | 0X0020;
		DirectCommands(0x62,RxData,W2);
	}
	else
	{
		/*This command is sent to allow the device to enter SLEEP mode*/
		CommandSubcommands(0x009a);
		RxData = DirectCommands(0x62,0x00,R);
		RxData = RxData & 0Xffdf;
		//RxData = RxData & 0Xffff;
		//RxData = 0Xffff;
		DirectCommands(0x62,RxData,W2);
	}
}

/**
 * @brief 开启/停止充电或放电
 * @param E_Charging 控制命令：
 *                   0x00 - 自动充放电
 *                   0x01 - 手动开启充电
 *                   0x02 - 手动开启放电
 *                   other- 手动禁止充放电
 * @return true  操作成功
 * @return false 操作失败
 */
bool OnOff_Charging(uint8_t E_Charging)
{
	if(E_Charging == 0x00)//自动充放
	{
		/*SET_CFGUPDATE*/
		CommandSubcommands(0x0090);
		Bq7808_delay(2000);
		/*This bit field includes settings related to the FET driver operation*/
		Subcommands(0x901E,FET_Options,W);
		Bq7808_delay(2000);
		/*EXIT_CFGUPDATE*/
		CommandSubcommands(0x0092);
		Bq7808_delay(2000);
	}
	else
	{
		/*SET_CFGUPDATE*/
		CommandSubcommands(0x0090);
		Bq7808_delay(2000);
		/*This bit field includes settings related to the FET driver operation*/
		Subcommands(0x901E,0X68,W);
		Bq7808_delay(2000);
		/*EXIT_CFGUPDATE*/
		CommandSubcommands(0x0092);
		Bq7808_delay(2000);
		if(E_Charging == 0x01)//手动充电
		{
			DirectCommands(0x68,0x06,W);
		}
		else if(E_Charging == 0x02)//手动放电
		{
			DirectCommands(0x68,0x09,W);
		}
		else if(E_Charging == 0x03)//手动关闭放充电
		{
			DirectCommands(0x68,0x0C,W);
		}
	}
    return 1;
}

/*This command is sent to reset the device*/
void Bq76907_reset(void)
{
	CommandSubcommands(0x0012);
}

#if 0
/**
 * @brief 带校准的电流读取（推荐）
 * @param gain 电流增益（uV/A，默认为5000uV/A）
 * @param r_sense 采样电阻（mΩ，默认为5mΩ）
 * @return float 实际电流（单位：A）
 */
float Read_CC1_Current_Calibrated(float gain, float r_sense)
{
    int16_t raw = Read_CC1_Current();
    if (raw == 0x7FFF) return 0; // 错误返回NaN

    // 电流计算公式（根据BQ76907手册调整）
    return (float)raw * (gain / 1000000.0f) / r_sense;
}
#endif
/**
 * @brief 充电安全管理策略
 * @return true  允许继续充电
 * @return false 需要停止充电（触发保护）
 */
bool Charging_Safety_Management(void)
{
    // 1. 监测所有电芯电压
    const uint8_t MAX_CELLS = 5; // 假设最大10节电芯
    uint16_t cell_voltages[MAX_CELLS];
    uint16_t max_voltage = 0;
    uint16_t min_voltage = 0xFFFF;

    // 读取所有电芯电压并找出极值
    for (uint8_t i = 0; i < MAX_CELLS; i++) {
        cell_voltages[i] = ReadCellVoltage(i);
        if (cell_voltages[i] == 0xFFFF) {
            return false; // 读取失败
        }

        if (cell_voltages[i] > max_voltage) {
            max_voltage = cell_voltages[i];
        }
        if (cell_voltages[i] < min_voltage) {
            min_voltage = cell_voltages[i];
        }
    }

    // 2. 读取温度
    int16_t temperature = Read_Int_Temperature();
    if (temperature == 0x8000) {
        return false; // 温度读取失败
    }
    float temp_c = temperature; // 转换为℃
    //float temp_c = temperature / 10.0f; // 转换为℃

    // 3. 读取电流
    int16_t current_ma = Read_CC2_Current();
    if (current_ma == 0x8000) {
        return false; // 电流读取失败
    }
    float current_a = current_ma / 1000.0f; // 转换为A

    // 4. 保护策略判断
    // 4.1 单体电压超过2.7V保护
    if (max_voltage > 2700) { // 假设电压单位为mV
        OnOff_Charging(0x00); // 停止充电
        return false;
    }

    // 4.2 过流保护（>3A）
    if (current_a > 3.0f) {
        OnOff_Charging(0x00);
        return false;
    }

    // 4.3 高温降额策略
    if (temp_c >= 70.0f && temp_c <= 85.0f) {
        if (max_voltage > 2500) { // 高温时限制2.5V
            OnOff_Charging(0x00);
            return false;
        }
    }
    // 4.4 超温保护（>85℃）
    else if (temp_c > 85.0f) {
        OnOff_Charging(0x00);
        return false;
    }

    // 5. 电压均衡检查（可选）
    if ((max_voltage - min_voltage) > 50) { // 50mV压差
        // 触发均衡逻辑
        uint8_t cells_to_balance = 0;
        for (uint8_t i = 0; i < MAX_CELLS; i++) {
            if (cell_voltages[i] > (min_voltage + 20)) { // 超过平均值20mV
                cells_to_balance |= (1 << i);
            }
        }
        Cell_Balancing(cells_to_balance);
    }

    return true; // 允许继续充电
}

/*Mask for Alarm Status(). Can be written to change during operation to change
which alarm sources are enabled. The default value of this parameter is set by
Settings:Configuration:Default Alarm Mask*/
void Alarm_Enable()
{
	DirectCommands(0x66,0xC000,W2);
}

void Bq76907_Init(void)
{
	/* 使能LPI2C中断 */
    IntCtrl_Ip_Init(&IntCtrlConfig_0);
	/* 初始化主机模块 */
	Lpi2c_Ip_MasterInit(I2C_MASTER, &I2c_Lpi2cMasterChannel0);
	//Siul2_Dio_Ip_WritePin(PTE_L_HALF, 10u, 1u);
	Bq7808_delay(2000);
	Bq76907_reset();
	Bq7808_delay(2000);

	/*SET_CFGUPDATE*/
	CommandSubcommands(0x0090);
	Bq7808_delay(2000);
	/*set Vcell Mode  5*/
	Subcommands(0x901b,Vcell_Mode,W);
	Bq7808_delay(2000);
	/*This bit field includes settings related to the FET driver operation*/
	Subcommands(0x901E,FET_Options,W);
	Bq7808_delay(2000);
	/* Enabled Protections A Register Field Descriptions*/
	Subcommands(0x9024,Protections_A,W);
	Bq7808_delay(2000);
	/* Enabled Protections B Register Field Descriptions*/
	Subcommands(0x9025,Protections_B,W);/*过温暂时不开*/
	Bq7808_delay(2000);
	/* DSG FET Protections A Register Field Descriptions*/
	Subcommands(0x9026,DSG_ProtectionsA,W);
	Bq7808_delay(2000);
	/* CHG FET Protections A Register Field Descriptions*/
	Subcommands(0x9027,CHG_ProtectionsA,W);
	Bq7808_delay(2000);
	/*Both FET Protections B Register Field Descriptions*/
	Subcommands(0x9028,Both_FET_Protections,W);
	Bq7808_delay(2000);
	/*Cell Overvoltage Protection Threshold  3200mv*/
	Subcommands(0x9032,Cell_Overvoltage,W2);
	Bq7808_delay(2000);
	/*Overcurrent in Charge Protection Threshold  16mv  3.2A*/
	Subcommands(0x9036,Charge_Threshold,W);
	Bq7808_delay(2000);
	/*Overcurrent in Charge Protection Threshold CC1 16mv*/
	//Subcommands(0x9038,Charge_Threshold,W);
	Subcommands(0x9038,1,W);
	Bq7808_delay(2000);
	/*Overcurrent in Charge Protection Threshold CC2 16mv*/
	Subcommands(0x9038,Charge_Threshold,W);
	Bq7808_delay(2000);
	/*Short Circuit in Discharge Protection Threshold Register Field Descriptions 20mv*/
	Subcommands(0x903C,Short_Circuit,W);
	Bq7808_delay(2000);
	/*Short Circuit in Discharge Protection Delay  7797 µs*/
	Subcommands(0x903D,Short_Circuit_Delay,W);
	Bq7808_delay(2000);
	/*EXIT_CFGUPDATE*/
	CommandSubcommands(0x0092);
	Bq7808_delay(2000);

	Alarm_Enable();
	Bq7808_delay(2000);
	Sleep_enable(0);
	Bq7808_delay(2000);
	OnOff_Charging(0);
}


