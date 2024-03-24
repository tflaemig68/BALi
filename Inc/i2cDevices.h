/*
 * I2C.h
 *
 *  Created on: 18.12.2023
 *      Author: tobia
 */

#ifndef I2CDEVICES_H_
#define I2CDEVICES_H_



extern uint8_t *convDecByteToHex(uint8_t byte);
extern void i2c_activate_pb89(I2C_TypeDef *i2c);

/* RFID SL018 Function
 *
 *
 *
 */
#define i2cAddr_RFID	0x50

extern void RFID_LED(I2C_TypeDef *i2c, bool LEDon);
extern int8_t RFID_readCard(I2C_TypeDef *i2c, char *CardID);
extern int8_t RFID_readFWVersion(I2C_TypeDef *i2c, char *strFirmware);

#define i2cAddr_LIS3DH 0x18
//extern int8_t i2cLIS3DH_init(I2C_TypeDef *i2c, int8_t restart);
extern int8_t i2cLIS3DH_Temp(I2C_TypeDef *i2c);
extern int16_t i2cLIS3DH_XYZ(I2C_TypeDef *i2c, int16_t *xyz);

#define _OK 1
#define i2cAddr_BMA020 0x38
//extern int8_t i2cBMA020INIT(I2C_TypeDef *i2c, int8_t restart);
extern int16_t i2cBMA020XYZ(I2C_TypeDef *i2c, int16_t *xyz);
extern int8_t bandwith;											// Bandbreite limitiert auf  25Hz;  750Hz;  1500Hz
extern int8_t precision;													// Auswahl zwischen 10(1) oder 8(0) Bit
extern int8_t range;														// Messbereich: einstellbar auf 2,4,8 (g)

extern void i2cBMA020Init(I2C_TypeDef *i2c);


extern void low_pass(int16_t raw_data[3], int16_t filt_data[3], int16_t _tp);




#define i2cAddr_LIDAR	0x29


extern bool enableLIS3DH;
extern bool enableBMA020;
extern bool enableRFID;
extern bool enableLIDAR;
#endif /* I2CDEVICES_H_ */



