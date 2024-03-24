/*
 * I2C.c
 *
 *  Created on: 18.12.2023
 *      Author: tobia
 */

#include <mcalGPIO.h>
#include <mcalI2C.h>
#include <mcalSysTick.h>
#include "i2cDevices.h"




    //RFID SL018

	uint8_t 	*strFirmware = (uint8_t *) " ....        \0";
	uint8_t  RFIDcmd_LEDon[3] = {0x02, 0x40, 0xFF};
	uint8_t  RFIDcmd_LEDoff[3] = {0x02, 0x40, 0x00};
	uint8_t  RFIDcmd_getFirmwareVersion[2] = {0x01, 0xF0};
	uint8_t  RFIDcmd_getMifareUID[2] = {0x01, 0x01};

	bool enableRFID = false;

	// LIS3DH
	bool enableLIS3DH = false;
	#define StatusReg 	0x27
	#define AcclReg 	0x28
	#define ADC1Reg 	0x08
	#define ADC3Temp 	0x0C
	#define	DummyReg 	0x0F			// check value at present dummyVal (0x33)
	#define ADCConfReg 	0x1F
	#define CtrlReg1 	0x20
	#define CtrlReg4	0x23
	#define CtrlReg5 	0x24
	#define StatusReg 	0x27
	#define OUTxyz 		0x28
	#define dummyVal	0x33
	#define I2Crepeat 	0x80
	#define XYZopt


	/*
	 * BMA020
	 */
	bool enableBMA020 = false;
	#define BMA020_CR1			0x15
	#define BMA020_CR2			0x14
	#define BMA020_CR3			0x0A
	#define BMA020_DR			0x02

	// Bitmasken

	//	Range
	#define BMA020_ACCEL_RANGE_2g 	0b00000
	#define BMA020_ACCEL_RANGE_4g 	0b01000
	#define BMA020_ACCEL_RANGE_8g 	0b10000

	//	Bandwidth
	#define BMA020_ACCEL_BANDW_25	0b000
	#define BMA020_ACCEL_BANDW_50	0b001
	#define BMA020_ACCEL_BANDW_100	0b010
	#define BMA020_ACCEL_BANDW_190	0b011
	#define BMA020_ACCEL_BANDW_375	0b100
	#define BMA020_ACCEL_BANDW_750	0b101
	#define BMA020_ACCEL_BANDW_1500	0b110

	// Shadowing
	#define BMA020_SHADW_DIS	0b1000

	// SW-Reset
	#define BMA020_SWRESET		0b10	//BMA020 is reset each time this bit (address 0Ah, bit 1) is written to 1

	#define BMA020precision 1												// Auswahl zwischen 10(1) oder 8(0) Bit
	uint16_t _bandwith = 750;													// Bandbreite limitiert auf  25Hz;  750Hz;  1500Hz
	uint8_t _range = 2;														// Messbereich: einstellbar auf 2,4,8 (g)
	uint8_t _shadow_dis = 0;


	// LIDAR
	bool enableLIDAR = false;

uint8_t *convDecByteToHex(uint8_t byte)
{
    static  uint8_t hex[2] = { 0 };

    uint8_t temp;

    temp = byte % 16;
    if (temp < 10)
    {
        temp += '0';
    }
    else
    {
        temp += '7';
    }
    hex[1] = temp;

    temp = byte / 16;
    if (temp < 10)
    {
        temp += '0';
    }
    else
    {
        temp += '7';
    }
    hex[0] = temp;

    return hex;
}


void setBMA020_Shadow(I2C_TypeDef *i2c, uint8_t ShadowDis)										// Ändern des Shadowbits (Genauigkeit)
{
	uint8_t daten, ret;
	i2cReadByteFromSlaveReg(i2c, i2cAddr_BMA020, BMA020_CR1, &ret);

	if (ShadowDis)																// niedrige Genauigkeit (8 Bit)
	{
		daten = (ret & 0b11110111);
		_shadow_dis = 0;
	}
	else																	// hohe Genauigkeit (10 Bit)
	{
		daten = (ret | BMA020_SHADW_DIS);
		_shadow_dis = 1;
	}
	i2cSendByteToSlaveReg(i2c, i2cAddr_BMA020, BMA020_CR1,   daten);
}


void setBMA020_Range(I2C_TypeDef *i2c, uint8_t Range)							// Ändern der Empfindlichkeit (Range) 2 4 8g
{
	uint8_t daten;
	i2cReadByteFromSlaveReg(i2c, i2cAddr_BMA020, BMA020_CR2, &daten);

	switch (Range)
	{
	case 2:															// Empfindlichkeit: +/- 2g
		{
		daten &= 0b11100111;
		_range = 2;
		break;
		}
	case 4:															// Empfindlichkeit: +/- 4g
		{
		daten &= 0b11101111;
		daten |= 0b00001000;
		_range = 4;
		break;
		}
	default:																	// Empfindlichkeit: +/- 8g
		{
		daten &= 0b11110111;
		daten |= 0b00010000;
		_range = 8;
		}
	}
	i2cSendByteToSlaveReg(i2c, i2cAddr_BMA020, BMA020_CR2, daten);
}

void setBMA020_Bandwidth(I2C_TypeDef *i2c, int16_t Bandwidth)		// Ändern der Bandbreite (Bandwidth) 25 50 750 1500
{
	uint8_t daten;
	i2cReadByteFromSlaveReg(i2c, i2cAddr_BMA020, BMA020_CR2, &daten);
	if (Bandwidth < 7)
	{
		daten = (daten & 0b11111000);
		daten = (daten |(uint8_t) Bandwidth);
		_bandwith=Bandwidth;
	}

	if (Bandwidth == 25)													// Bandbreite 25Hz
	{
		daten = (daten & 0b11111000);
		_bandwith =25;
	}
	if (Bandwidth == 750)													// Bandbreite 750Hz
	{
		daten = (daten & 0b11111101);
		daten = (daten | 0b00000101);
		_bandwith=750;
	}
	else																	// Bandbeite 1500Hz
	{
		daten = (daten & 0b11111110);
		daten = (daten | 0b00000110);
		_bandwith = 1500;
	}
	i2cSendByteToSlaveReg(i2c, i2cAddr_BMA020, BMA020_CR2, daten);
}


int8_t i2cBMA020INIT(I2C_TypeDef *i2c, int8_t restart)
{
	static int8_t step = -4;

	if ((restart != 0) && (step == -4))
	{
		step = -3;
	}

	switch (step)
	{
		case -4:		// CLK Speed von I2C auf Fast Mode
		{
			i2cSetClkSpd(i2c, I2C_CLOCK_1Mz); //set I2C Clock 1000kHz fast Mode
			step = -2;
		}
		break;
	/*	case -3:		// Check if Sensor present
		{

			if (1 == sensor_presCheck(i2c))
			{
				step = -2;


			}
			else
			{
				return 1;		//1 = failure
			}
			break;

		}
		*/
		case -3:
		{		// SW Reset
			i2cSendByteToSlaveReg(i2c, i2cAddr_BMA020, BMA020_CR3, BMA020_SWRESET); // reboot memory content
			step = -2;
		}
		break;
		case -2:
		{
			setBMA020_Range(i2c, 2);
			step = -1;
		}
		break;
		case -1:
		{		// Config Range & Bandwidth
			i2cSendByteToSlaveReg(i2c, i2cAddr_BMA020, BMA020_CR2, (BMA020_ACCEL_RANGE_2g|BMA020_ACCEL_BANDW_190)); // Set Range to ±2g, Set Filter bandwidth to 190Hz
			step = 0;
		}
		break;
		default:
		{
			step = -4;
		}

	}



	/*
	// Genauigkeit einstellen (10 oder 8 Bit)
	// Shadowbit setzen (Genauigkeit)

	setBMA020_Shadow(i2c, 0);
	// Empfindlichkeit einstellen (+/- 2,4,8 Bit)
	setBMA020_Range(i2c, 2);
	// Bandbreite einstellen
	setBMA020_Bandwidth(i2c, (int16_t) 750);
	*/
	return step;
}

int16_t i2cBMA020XYZ(I2C_TypeDef *i2c, int16_t *xyz)
{
	uint8_t _rdata[6];
	int16_t schiebeReg;
	i2cBurstRegRead(i2c, i2cAddr_BMA020, BMA020_DR, _rdata, 6);

	float skal_div = 1/4.088;															// Zwischenspeicher für Bearbeitung der dez. Achsdaten
	int i;

	switch (_range)													// Skalierungsfaktoren für den Messberich +/-(2,4,8g) ermitteln
	{
	case 2:
		{
		skal_div =1/16.352;
		break;
		}
	case 4:
		{
		skal_div = 1/8.176;
		}
	}

	for (i=0;i<=2;i++)
	{

		schiebeReg = _rdata[2*i+1];									// MSB in Schieberegister schreiben
		schiebeReg <<= 8;											// Schieben an die ersten 8 Bit

		if(BMA020precision)
		{
			schiebeReg += _rdata[2*i];										// Addieren des jeweiligen LSB zum MSB
			schiebeReg &= 0xFFC0; //b1111111111000000; 					// Abschneiden der geringeren Stellen, da diese keine Daten enthalten
		}
		xyz[i] = (int16_t)(((int16_t)schiebeReg) * skal_div);									// Konvertieren in Integer-Format und Messbereich einrechnen (2,4,8g) einberechnen
	}

	return 0;
}


int8_t i2cLIS3DH_presCheck(I2C_TypeDef *i2c)
{
	uint8_t ret;

	i2cReadByteFromSlaveReg(i2c, i2cAddr_LIS3DH, DummyReg, &ret);
	//spiReadRegBurst(spi, LIS3DH_CS_PORT, LIS3DH_CS, (DummyReg|spiRead), ret, 2);
	if (dummyVal == ret)
	{return 1;}
	else
	{return 0;}

}

/*
int8_t i2cLIS3DH_init(I2C_TypeDef *i2c, int8_t restart)
{
	#define stepStart -7
	static int8_t step = stepStart ;

	if ((restart != 0) && (step == stepStart))  {step = stepStart+1;}
	switch (step)
	{
		case -7:
		{
			//i2cResetDevice(i2c);
			//i2cInitI2C(i2c, I2C_DUTY_CYCLE_2, 17, I2C_CLOCK_400);

			i2cSetClkSpd(i2c,  I2C_CLOCK_1Mz); //set I2C Clock 1000kHz fast Mode
			//i2cEnableDevice(i2c);
			step = -6;
			break;
		}
		case -6:
		{
			if (1 == i2cLIS3DH_presCheck(i2c))
			{
				step = -5;
			}
			else
			{
				return 1;		//1 = failure
			}
			break;
		}
		case -5:
		{
			i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH, CtrlReg5,   0b10000000); // reboot memory content
			step = -4;
			break;
		}
		case -4:
		{
			i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH, CtrlReg4,   0b10001100); // BDU BLE FS1 FS0 HR ST1 ST0 SIM
			step = -3;
			break;
		}
		case -3:
		{
			i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH, CtrlReg1,  (uint8_t) 0b10010111); //
			step = -2;
			break;
		}
		case -2:
		{
			uint8_t LIS3DH_Reg4 = 0b10001000;			// BDU BLE FS1 FS0 HR ST1 ST0 SIM
#ifdef XYZopt
//			LIS3DH_Reg4 |=        0b01000000;			// enable to switch High and Low Byte order. special Hint: High Byte first for a fast read sequence to array of int16
#endif
			i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH, CtrlReg4,  LIS3DH_Reg4); // BDU BLE FS1 FS0 HR ST1 ST0 SIM
			step = -1;
			break;
		}
		case -1:
		{
			i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH, ADCConfReg,(uint8_t) 0b11000000); // ADC Temp 0 0 0 0 0 0

			step = 0;
			break;
		}
		default:
		{
			step = -7;
		}
	}
	return step;

}



*/

int16_t i2cLIS3DH_XYZ(I2C_TypeDef *i2c, int16_t *xyz)
{
#ifdef XYZopt
	i2cBurstRegRead(i2c, i2cAddr_LIS3DH, (OUTxyz|I2Crepeat),(uint8_t *) xyz, 6);
#else
	uint8_t readBuffer[6];
	i2cBurstRegRead(i2c, i2cAddr_LIS3DH, (OUTxyz|I2Crepeat),readBuffer, 6);
	*xyz = (readBuffer[1]<<8) + readBuffer[0];
	xyz++;
	*xyz = (readBuffer[3]<<8) + readBuffer[2];
	xyz++;
	*xyz = (readBuffer[5]<<8) + readBuffer[4];
#endif
	return 0;
}





/**
- enable the bits 6 and 7 in TEMP_CFG_REG(1Fh): enable aux ADC and enable temperature sensor.
- enable Block Data Update, BDU, feature. CTRL_REG4(0x23) , BDU (bit#7)=1.
- read both the ADC output 3 registers (because of BDU enabled): OUT_ADC_3_L(0x0C) and OUT_ADC_3_H(0x0D).
Useful bits: 8, left aligned, hence useful data in OUT_ADC_3_H.
Temperature sensor output change vs temperature: 1digit/degrCelsius
*/
int8_t i2cLIS3DH_Temp(I2C_TypeDef *i2c)
{
	uint8_t readBuffer[2];
	//i2cReadByteFromSlaveReg(i2c, i2cAddr_LIS3DH, (ADC3Temp+1), (uint8_t *) &readBuffer[1]);  	// only Hihg byte with data
	i2cBurstRegRead(i2c, i2cAddr_LIS3DH, (ADC3Temp|I2Crepeat), readBuffer, 2);					// but BDU =1 required for update data to read both, high and low Byte
	return ((int8_t) readBuffer[1]);					// send a delta Temp Value
}


void RFID_LED(I2C_TypeDef *i2c, bool LEDon)
{
	if (LEDon == 1)
	{
		i2cBurstWrite(i2c, i2cAddr_RFID, RFIDcmd_LEDon, 3);
	}
	else
	{
		i2cBurstWrite(i2c, i2cAddr_RFID, RFIDcmd_LEDoff, 3);
	}
}

int8_t RFID_readCard(I2C_TypeDef *i2c, char *CardID)
{
	static  uint8_t step = 1;
	uint8_t readBuffer[14];
	uint8_t len, i, j = 0;
	int8_t typeCard = -1;
	static int8_t RFID_Status;
	char *p_out;

	switch (step)
	{
		case 1:
		{
			i2cBurstWrite(i2c, i2cAddr_RFID, RFIDcmd_getMifareUID, 2);
			step = 2;
			break;
		}
		case 2:
		{
			i2cBurstRead(i2c, i2cAddr_RFID, readBuffer, 0xC);
			len = readBuffer[0]-2;
			step = 1;
			if (RFID_Status != readBuffer[2])
			{
				typeCard = readBuffer[len];
				j = 0;
				for (i = 0; i< 8; i++ )
				{
					if (i < len)
					{
						if (i == len-1) { CardID[j++]='-'; }					// add the Type number after this  -
						p_out =	(char *) convDecByteToHex(readBuffer[i+3]);
						CardID[j++]  = (char)*(p_out++);
						CardID[j++] =(char)*p_out;
					}
					else
					{CardID[j++]='.'; CardID[j++]='.';}

				}
				CardID[j]='.';
				RFID_Status = readBuffer[2];
			}
			break;
		}
		default:
		{
			step = 1;
		}
	}
	return typeCard;
}

int8_t RFID_readFWVersion(I2C_TypeDef *i2c, char *strFirmware)
{
	static  uint8_t step = 1;
		uint8_t readBuffer[16];
		uint8_t i, len;
		int8_t status = -1;

		switch (step)
		{
			case 1:
			{
				RFID_LED(i2c,0);
				step = 2;
				break;
			}
			case 2:
			{
				i2cBurstWrite(i2c, i2cAddr_RFID, RFIDcmd_getFirmwareVersion, 2);
				step = 3;
				break;
			}
			case 3:
			{
				i2cBurstRead(i2c, i2cAddr_RFID, readBuffer, 0xF);
				len = readBuffer[0];
				status = readBuffer[2];
				step = 4;
				for (i = 0; i<= len; i++ )
				{
				 strFirmware[i] = readBuffer[i+3];
				}

				break;
			}
			case 4:
			{
				RFID_LED(i2c,0);
				step = 1;
				break;
			}

			default:
			{
				step = 1;
			}
		}
		return status;
}




// Tiefpassfilterung der drei Richtungsvektoren xyz
void low_pass(int16_t raw_data[3], int16_t filt_data[3], int16_t _tp)
{
	static long _sto_xyz[3];
	uint8_t i;
	for (i=1;i<=3;i++)
	{
  	 	_sto_xyz[i] += (long) raw_data[i] - (filt_data[i] = _sto_xyz[i]/_tp);
	}
}

