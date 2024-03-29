/**
 * I2C Device Scan
 * ST7735 TFT Display
 * ===========================
 *
 * Ansteuerung eines TFT Display ueber SPI.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <stm32f4xx.h>

#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalSPI.h>
#include <mcalI2C.h>
#include <ST7735.h>
#include <RotaryPushButton.h>
#include <Balancer.h>

//#include "hw_config.h"

#include "i2cDevices.h"
#include "xyzScope.h"


bool timerTrigger = false;


// Declaration  Timer1 = Main Prog
// 				ST7725_Timer delay Counter
uint32_t	Timer1 = 0UL;
uint32_t    ST7735_Timer = 0UL;
uint32_t    I2C_Timer = 0UL;
//#define I2CTaskTime 20


/* Private function prototypes -----------------------------------------------*/
void test_ascii_screen(void);
void test_graphics(void);

uint8_t I2C_SCAN(I2C_TypeDef *i2c, uint8_t scanAddr);



int main(void)
{
/*  I2C Variables  */

	uint8_t        scanAddr = 0x7F;  //7Bit Adresse
	I2C_TypeDef   *i2c  = I2C1;
	I2C_TypeDef   *i2c2  = I2C2;
	int8_t ret;
	uint32_t   i2cTaskTime = 50UL;

/*  End I2C Variables  */

	char strCardID[]   = ".  .  .  .  .  .  .\0";
	char strFirmware[] = ". . .          \0";  // dummyString with NULL


	char strX[8],strY[8],strZ[8],strT[8];
	int8_t Temp;
	int16_t XYZraw[3];
	float XYZ[3], AlphaBeta[2];

	static uint8_t testmode = 1;
	uint16_t timeTMode5;

	//int testmode = 1;
   	//unsigned int r = 0;

       // Dies ist das Array, das die Adressen aller Timer-Variablen enthaelt.
       // Auch die Groesse des Arrays wird berechnet.
       uint32_t *timerList[] = { &I2C_Timer, &ST7735_Timer /*, weitere Timer */ };
       size_t    arraySize = sizeof(timerList)/sizeof(timerList[0]);


    BalaHWsetup();

    /* initialize the rotary push button module */
    initRotaryPushButton();


    // Initialisiert den Systick-Timer
    systickInit(SYSTICK_1MS);

    systickSetMillis(&I2C_Timer, i2cTaskTime);
    //lcd7735_initR(0);
    LED_red_on;

    //lcd7735_setup();
    //Inits needed for TFT Display

    	spiInit();
    	tftInitR(INITR_REDTAB);
    //tftSetup();
    //  tftInitR(INITR_REDTAB);

    tftSetRotation(LANDSCAPE);
    tftSetFont((uint8_t *)&SmallFont[0]);
    tftFillScreen(tft_BLACK);

    LED_red_off;

    tftPrint((char *)"I2C Scanner running \0",0,0,0);
    tftPrint((char *)"Select I2C Connector \0",0,14,0);






    while (1)
    {
	   if (true == timerTrigger)
	   {
			systickUpdateTimerList((uint32_t *) timerList, arraySize);
	   }

	   if (isSystickExpired(I2C_Timer))
	   {
		   systickSetTicktime(&I2C_Timer, i2cTaskTime);
		   LED_green_off;


		   switch (testmode)
		   {
		   	   case 0:  //I2C Scan
		   	   {
		   		   //lcd7735_setForeground(ST7735_YELLOW);
		   		   i2cSetClkSpd(i2c,  I2C_CLOCK_400);
		   		   i2cSetClkSpd(i2c2,  I2C_CLOCK_400);
		   		   //tftPrint((char *)".  .  .  .  . \0",66,14,0);
		   		   testmode  = 1;
		   	   }
		   	   case 1:  //I2C Scan
		   	   {
		   		   LED_red_on;
		   		   if ( I2C_SCAN(i2c, scanAddr) != 0)
				   {
					   LED_red_off;
					   switch (scanAddr)
					   {
						   case i2cAddr_RFID:
						   {
							   enableRFID = true;
							   tftPrint((char *)"RFID connected \0",0,28,0);
							   RFID_LED(i2c,true);
							   break;
						   }
						   case i2cAddr_LIDAR:
						   {
							   enableLIDAR = true;
							   //lcd7735_print((char *)"TOF/LIADR connected \0",0,28,0);
							   break;
						   }
						   case i2cAddr_LIS3DH:
						   {
							   enableLIS3DH = true;
							   tftPrint((char *)"LIS3DH connected \0",0,28,0);

							   tftPrint((char *)"Temp:\0",0,40,0);
							   tftPrint((char *)"X:\0",0,50,0);
							   tftPrint((char *)"Y:\0",0,60,0);
							   tftPrint((char *)"Z:\0",0,70,0);
							   LED_blue_on;
							   break;
						   }
						   case i2cAddr_BMA020:
						   {
							   enableBMA020 = true;
							   tftPrint((char *)"BMA020 connected \0",0,28,0);

							   tftPrint((char *)"X:\0",0,50,0);
							   tftPrint((char *)"Y:\0",0,60,0);
							   tftPrint((char *)"Z:\0",0,70,0);
							   LED_blue_on;
							   testmode = 4;
		 					   i2cTaskTime = 200;
							   break;
						   }


					   }
				   }

				   if ((scanAddr == 0) && (enableRFID))
				   {
					   scanAddr = 0x7F;
					   i2cTaskTime = 200UL;
					   		// SL018 only works with 100kHz
					   testmode = 2;
				   }
				   if ((scanAddr == 0) && (enableLIS3DH))
				   {
					   scanAddr = 0x7F;
					   testmode = 4;
					   i2cTaskTime = 200;

				   }
				   if ((scanAddr == 0))
				   {
					   scanAddr = 0x7F;
					   if (i2c == I2C1)
					   {
						   i2c = I2C2;
					   }
					   else
					   {
						   i2c = I2C1;
						   tftFillScreen(tft_BLACK);
					   }
				       testmode = 0;
				   }
				   else
				   {
					   scanAddr -=1;
				   }
				   break;
				}
		   	   	case 2:  // read RFID Firmware
				{
					if (RFID_readFWVersion(i2c, (char *)strFirmware) >= 0)
					{
						tftPrint((char *)"FW: \0",0,48,0);
						tftPrint((char *)strFirmware,24,48,0);
						testmode = 3;
						tftPrint((char *)"ID:\0",0,70,0);
					}
					else
					{
						;
					}
				}
				break;
		   	   	case 3:  // read RFID ID
		   		{
		   			if (RFID_readCard(i2c, strCardID)> 0)
		   			{
		   				tftPrint((char *)strCardID,24,70,0);
		   			}
		   		}
		   		break;

// LIS3DH function
		   	 	case 4:  // BMA020 Init		   			   		{
		   	 	{
		   			LED_red_off;
		   			//currentSensor = SENSOR_BMA020;
		   			ret = i2cBMA020INIT(i2c, 0);
		   	 		//i2cBMA020INIT(i2c, 0);
					if (ret > 0)										// no LIS3DH Sensor present
					{
						tftPrint("BMA020 not Present ",0,0,0);
						i2cTaskTime = 500;
						testmode = 1;
					}
					if (ret == 0)										// LIS3DH init-procedure finished
					{
						tftPrint("(C)24Fl I2C BMA020 ",0,0,0);
						i2cTaskTime = 70;									// Tasktime for display 70ms
						testmode = 5;
						timeTMode5 = 100;							// count of cycles in Mode5
					}
				}
				break;
		   	 	case 5:  // read BMA020 Data
				{
					LED_blue_on;

					i2cBMA020XYZ(i2c,(int16_t *) XYZraw);
					//low_pass(XYZraw, XYZfilt, 10);
					XYZ[0] = (float) XYZraw[0];  //skalierung 1mg/digit at +-2g
					XYZ[1] = (float) XYZraw[1];
					XYZ[2] = (float) XYZraw[2];
					sprintf(strX, "%+6.0f", XYZ[0]);
					tftPrint((char *)strX,20,50,0);
					sprintf(strY, "%+6.0f", XYZ[1]);
					tftPrint((char *)strY,20,60,0);
					sprintf(strZ, "%+6.0f", XYZ[2]);
					tftPrint((char *)strZ,20,70,0);
					if ((timeTMode5--) > 0)
					{
						testmode = 5;
						//tftPrint("T:    BMA020 (C)24Fl",0,0,0);
						i2cTaskTime = 200;
						LED_blue_off;

					}
		   	 	}
				break;
		 	 	case 6:  // LIS3DH Init		   			   		{
				{
					LED_red_off;
					int8_t ret = i2cLIS3DH_init(i2c, 0);
					if (ret > 0)										// no LIS3DH Sensor present
					{
						tftPrint("LIS3DH not Present ",0,0,0);
						i2cTaskTime = 500;
						testmode = 1;
					}
					if (ret == 0)										// LIS3DH init-procedure finished
					{
						tftPrint("(C)23Fl I2C LIS3DH ",0,0,0);
						i2cTaskTime = 70;									// Tasktime for display 70ms
						testmode = 7;
						timeTMode5 = 10;							// count of cycles in Mode5
					}
				}
				break;
		   		case 7:  // read LIS3DH Data
		   		{
		   			LED_blue_on;

		   			Temp = i2cLIS3DH_Temp(i2c);
		   			sprintf(strT, "%+3i", Temp);
		   			tftPrint((char *)strT,40,40,0);

		   			i2cLIS3DH_XYZ(i2c,(int16_t *) XYZraw);

  					XYZ[0] = (float) XYZraw[0]/0x3FFF;  //skalierung 1mg/digit at +-2g
		   			XYZ[1] = (float) XYZraw[1]/0x3FFF;
		   			XYZ[2] = (float) XYZraw[2]/0x3FFF;
		   			sprintf(strX, "%+6.3f", XYZ[0]);
		   			tftPrint((char *)strX,20,50,0);
		   			sprintf(strY, "%+6.3f", XYZ[1]);
		   			tftPrint((char *)strY,20,60,0);
		   			sprintf(strZ, "%+6.3f", XYZ[2]);
		   			tftPrint((char *)strZ,20,70,0);
					if ((timeTMode5--) > 0)
					{
						testmode = 8;
						tftFillScreen(tft_BLACK);
						tftPrint("T:    LIS3DH (C)23Fl",0,0,0);
						i2cTaskTime = 20;
						LED_blue_off;

					}
				    break;
				}
		   		case 8:  // Scope display the LIS3DH Data
				{
					i2cLIS3DH_XYZ(i2c, XYZraw);

					XYZ2AlphaBeta(XYZraw, AlphaBeta);
					if (AlBeScreen(AlphaBeta) == 0)
					{
						Temp = i2cLIS3DH_Temp(i2c);
						sprintf(strT, "%+3i", Temp);
						tftPrint((char *)strT,12,0,0);
					}
					//testmode = 2;
					break;
				}

//end LIS3DH function
		   	   default:
				{
					testmode = 0;
				}
		   }  //end switch (testmode)
	   } // end if systickexp
    } //end while
    return 0;
}

/* scanAdr. 7Bit Adresse value
 * return	0 if no device found on scanAdr
 *			if yes  return the scanAdr.
 *			and display on the ST7735 Display
 *
 *
 */



uint8_t I2C_SCAN(I2C_TypeDef *i2c, uint8_t scanAddr)
{
	uint8_t 	*outString2 = (uint8_t *) "Addr at: \0";
	uint8_t     port, *result;
#define yPosBase 28
	uint8_t foundAddr = 0;
	static int xPos[2] = {0,100};
	static int yPos[2] = {yPosBase, yPosBase};

	if (i2c == I2C1)
    {
	   port = 0;
    }
    else
    {
	   port = 1;
    }
    if (scanAddr == 0)
    {
    yPos[0] = yPosBase;
    yPos[1] = yPosBase;
    }

	foundAddr = i2cFindSlaveAddr(i2c, scanAddr);
	if (yPos[port] == 0)
	{
		tftPrint((char *)outString2,xPos[port],yPos[port],0);
		yPos[port] = 66;
	}
	result = convDecByteToHex(scanAddr);
	if (foundAddr != 0)
	{
		//outString = outString2;
		tftPrint((char *)result,xPos[port],yPos[port],0);
		yPos[port] = (int) 14 + yPos[port];
		if (yPos[port] > 100)
		{
			yPos[port] = yPosBase;
		}
	}
	else
	{
	//	tftPrint((char *)result,xPos,14,0);
	}
	return foundAddr;

}

/*
void balanceMotor(void)
{
	const int offset_phi = 0; 		// Absolutwert des Winkels für die Schwerpunktlage in degr*10
	int tp_fakttar = 50, tp_fakt3dg = 23;
	int kp = 180;//345; // 140;
	int kd = 999;	//1200 ;			// P Anteil kp/1000 , D-Anteil kd/1000
	int y_off = 100;
	//const float deg2rad = 0.0001745;				// Faktor PI/180°/100
	static int y_old, PotPos;
	static BYTE Ihold, OnMot;
	static long _sto_ltargmean;
	BYTE ret;
	long _ltargetpos;
	int _itargetpos, filt_target, x,y,z, PotPos_raw, phi, rw, Pos_OK, _txyz[4], gxyz[4];
	int rot_l, rot_r;
	static int pos_motL= 0, pos_motR= 0;
	int disp = 0;
	rot_l = -turnSteps;
	rot_r = turnSteps;


	//PotPos =  Conv_mV(ADCfilt[Chan_Uin])/(maxUpot_mV/250) - 125;
	y_off += PotPos;
	//kd += 3*PotPos;
	ret = read_axes(D3Sens_addr, _txyz);
	low_pass(_txyz, gxyz, tp_fakt3dg);
//----------------------------------------------------------------------------
// Regler auf Basis der Achs-Beschleunigungen
	y = gxyz[2];
	z = gxyz[3];
	sprintf(senden, "y%+04i,z%+04i", y,z);
	y += y_off;
	_ltargetpos = (((long)y* (long)kp + (long)(kd*(y - y_old))))/((long)z);
	y_old = y;

	_sto_ltargmean += (long)(_ltargetpos) - (filt_target = _sto_ltargmean/tp_fakttar);
	_itargetpos= -(int)_ltargetpos;  // Richtungsumkehr
//----------------------------------------------------------------------------
	sprintf(senden2, "tr%+05i,mt%+05i", _itargetpos, filt_target);
//	phi = (int)phi_yz(gxyz)-offset_phi+PotPos;
//	rw = (phi/10)*(phi_old/10);		// Werteüberlauf vermeiden
//	if (rw >= 0) // true kein Vorzeichenwechsel,d.h. kein Seitenwechsel


//		if ((phi < 150) && (phi > -150))
//		{
//			_itargetpos = (int) (kp/2* (float) tan(((float)phi)*deg2rad));		// Ruckelvermeidung durch geringere Verstärkung
//		}
//		else
//		{
//			_itargetpos = (int) (kp* (float) tan(((float)phi)*deg2rad));
//		}
//	_itargetpos += (int) (kd*(float) (phi - phi_old)*deg2rad);
	//_itargetpos += (int) (kd*(float) tan(((float)(phi - phi_old))*deg2rad));
//	phi_old = phi;



	Pos_OK = 0;
	if (z > 500)
	{
		Pos_OK = 1;
	} // <Pos nur wenn noch innerhalb von 40° gekippt ist
	else
	{
		softStop(motR_addr);
		softStop(motL_addr);
		sprintf(senden2, " 1___ %03i ---v ", y_off);
		resetPosition(motR_addr);
		pos_motR = 0;
		resetPosition(motL_addr);
		pos_motL = 0;
	}




//	if ((phi < 4500) && (phi > -4500)) {Pos_OK = 1;}		 // <Pos nur wenn noch innerhalb von 45° gekippt ist
//	if ((Pos_OK == 1) && (OnMot != 1))
//	{
//		setIhold(motL_addr,Ihold);
//		setIhold(motR_addr,Ihold);
//		OnMot = 1;
//	}
//	if (Pos_OK != 1)
//	{
//		Ihold = getIhold(motL_addr);
//		setIhold(motL_addr,0);
//		setIhold(motR_addr,0);
//		OnMot = 0;
//	}

	if (Pos_OK == 1)
	{
//			resetPosition(motL_addr);
//			setPosition(motL_addr, _itargetpos+rot_l);
//			resetPosition(motR_addr);
//			setPosition(motR_addr, _itargetpos+rot_r);


		if (_itargetpos <= rot_l )
		{

			// pos_motR = getActualPosition(motR_addr);		//
			setPosition(motR_addr, pos_motR+_itargetpos+rot_r);
			pos_motR += _itargetpos+rot_r;
		}
		else
		{
			//pos_motR = getActualPosition(motR_addr);		//
			setPosition(motR_addr, pos_motR+_itargetpos);
			pos_motR += _itargetpos;
		}
		if (rot_r <= _itargetpos)
		{
			//pos_motL = getActualPosition(motL_addr); //
			setPosition(motL_addr, pos_motL+_itargetpos+rot_l);
			pos_motL += _itargetpos+rot_l;
		}
		else
		{
			//pos_motL = getActualPosition(motL_addr); //
			setPosition(motL_addr, pos_motL+_itargetpos);
			pos_motL += _itargetpos;
		}
	}

}

*/



