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
//uint32_t    ST7735_Timer = 0UL;
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
       uint32_t *timerList[] = { &I2C_Timer /*, weitere Timer */ };
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

    tftSetup();
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
		   		   i2cSetClkSpd(i2c,  I2C_CLOCK_100);
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
		   	 	case 4:  // LIS3DH Init		   			   		{
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
						testmode = 5;
						timeTMode5 = 10;							// count of cycles in Mode5
					}
				}
				break;
		   		case 5:  // read LIS3DH Data
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
						testmode = 6;
						tftFillScreen(tft_BLACK);
						tftPrint("T:    LIS3DH (C)23Fl",0,0,0);
						i2cTaskTime = 20;
						LED_blue_off;

					}
				    break;
				}
		   		case 6:  // Scope display the LIS3DH Data
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





