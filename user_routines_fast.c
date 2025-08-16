/*******************************************************************************
* FILE NAME: user_routines_fast.c <FRC VERSION>
*
* DESCRIPTION:
*  This file is where the user can add their custom code within the framework
*  of the routines below. 
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
* OPTIONS:  Interrupts are disabled and not used by default.
*
*******************************************************************************/

#include <stdio.h>
#include <math.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "pwm.h"
#include "camera.h"
#include "tracking.h"

	int intMax(int x, int y);
	int intABS(int i);

	float servotoradians(int i)
				{
					float servoradians = i * 0.00618423750706652212295;
					return servoradians;
				}

	int pwmlimit(double p)
				{
					if (p > 254)
						{
							p = 254;
							return p;
						}
					else if ( p < 0)
						{
							p = 0;
							return p;
						}
					else
						{
							return p;
						}
				}

	int speed (double d)
				{
					int pwm = .00029518451429042 * pow(d, 3) + -.03343696764576 * pow(d, 2) + 1.8878283928015 * d + 103.81472257769;
					pwm = pwmlimit(pwm);
					return pwm;
				}

	int rotation (int p)
				{
					//int r = -.000034829005448167 * pow(p, 3) + .013269851075752 * pow(p, 2) - .8640312288006 * p + 94.045680723803;
					if (p >= 142)
						{
							p = 142;
						}

					else if (p <= 112)
						{
							p = 112;
						}

					if (p > 127)
						{
							p += 12;
						}

					return p;
				}		
	
	int sidestep (int p)
				{
					if (p >= 132)
						{
							p = 152;
						}

					else if (p <= 122)
						{
							p = 102;
						}

					else
						{
							p = 127;
						}
					return p;
				}

			int gripperrelease = 1;


	/*		rom const float rackdistance[180] = {0, 11917.21961, 5958.156008, 3971.599766, 2978.170341,
										   2381.991616, 1984.438215, 1700.384969, 1487.26929,
										   1321.445292, 1188.725439, 1080.081296, 989.493903,
										   912.7962849, 847.0120283, 789.9584275, 739.998451,
										   695.8802486, 656.6301574, 621.4794938, 589.8133091,
										   561.133786, 535.0336175, 511.1763311, 489.2815313,
										   469.1136843, 450.4734908, 433.191176, 417.121217,
										   402.1381618, 388.1332839, 375.0118848, 362.6911021,
										   351.0981159, 340.1686722, 329.8458594, 320.0790878, 
										   310.823236, 302.0379312, 285.7376516, 278.1606353, 
										   270.9292627, 264.0193811, 257.4090327, 251.0782104, 
										   245.008646, 239.1836249, 233.5878244, 228.207172, 
										   223.0287197, 218.0405342, 213.2315995, 208.5917296, 
										   204.1114925, 199.7821412, 195.5955523, 191.5441721, 
										   187.6209665, 183.8193776, 180.1332839, 176.5569644, 
										   173.0850661, 169.7125754, 166.434791, 163.2473, 
										   160.1459562, 157.1268601, 154.1863407, 151.3209389, 
										   148.5273927, 145.8026226, 143.1437197, 140.5479335, 
										   138.0126614, 135.5354387, 133.1139297, 130.745919, 
										   128.4293042, 126.1620884, 123.9423736, 121.7683548, 
										   119.6383143, 117.5506161, 115.5037015, 113.4960841, 
										   111.5263458, 109.593133, 107.6951526, 105.8311688, 
										   104, 102.2005153, 100.4316325, 98.6923149, 
										   96.98156892, 95.29844206, 93.64202057, 92.01142748, 
										   90.4058207, 88.82439125, 87.26636161, 85.73098409, 
										   84.21753942, 82.7253353, 81.25370512, 79.80200672, 
										   78.36962118, 76.95595177, 75.56042288, 74.18247903, 
										   72.82158394, 71.47721966, 70.14888572, 68.83609833, 
										   67.53838966, 66.25530709, 64.98641257, 63.73128193, 
										   62.48950435, 61.26068167, 60.04442796, 58.8403689, 
										   57.64814132, 56.46739273, 55.29778086, 54.13897322, 
										   52.99064671, 51.8524872, 50.72418918, 49.60545537, 
										   48.49599642, 47.39553054, 46.30378324, 45.22048696, 
										   44.14538085, 43.07821045, 42.01872745, 40.96668943, 
										   39.92185961, 38.88400663, 37.85290433, 36.82833151, 
										   35.81007175, 34.7979132, 33.79164838, 32.79107401, 
										   31.79599084, 30.80620344, 29.82152009, 28.84175255, 
										   27.86671598, 26.89622874, 25.93011226, 24.96819091, 
										   24.01029184, 23.05624488, 22.10588238, 21.15903911, 
										   20.21555212, 19.27526064, 18.33800596, 17.40363131, 
										   16.47198176, 15.54290411, 14.61624677, 13.69185971, 
										   12.7695943, 11.84930323, 10.93084043, 10.01406098, 
										   9.098820971, 8.184977474, 7.272388406, 6.36091246, 
										   5.450409009, 4.540738026, 3.631759991, 2.723335806, 
										   1.815326716, 0.907594213, 0};

			

/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/


/*******************************************************************************
* FUNCTION NAME: InterruptVectorLow
* PURPOSE:       Low priority interrupt vector
* CALLED FROM:   nowhere by default
* ARGUMENTS:     none
* RETURNS:       void
* DO NOT MODIFY OR DELETE THIS FUNCTION 
*******************************************************************************/
#pragma code InterruptVectorLow = LOW_INT_VECTOR
void InterruptVectorLow (void)
{
  _asm
    goto InterruptHandlerLow  /*jump to interrupt routine*/
  _endasm
}


/*******************************************************************************
* FUNCTION NAME: InterruptHandlerLow
* PURPOSE:       Low priority interrupt handler
* If you want to use these external low priority interrupts or any of the
* peripheral interrupts then you must enable them in your initialization
* routine.  Innovation First, Inc. will not provide support for using these
* interrupts, so be careful.  There is great potential for glitchy code if good
* interrupt programming practices are not followed.  Especially read p. 28 of
* the "MPLAB(R) C18 C Compiler User's Guide" for information on context saving.
* CALLED FROM:   this file, InterruptVectorLow routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
#pragma code
#pragma interruptlow InterruptHandlerLow save=PROD,section(".tmpdata")

void InterruptHandlerLow ()     
{
	if (PIR1bits.RC1IF && PIE1bits.RC1IE) // rx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_RX
		Rx_1_Int_Handler(); // call the rx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.RC2IF && PIE3bits.RC2IE) // rx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_RX
		Rx_2_Int_Handler(); // call the rx2 interrupt handler (in serial_ports.c)
		#endif
	} 
	else if (PIR1bits.TX1IF && PIE1bits.TX1IE) // tx1 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_ONE_TX
		Tx_1_Int_Handler(); // call the tx1 interrupt handler (in serial_ports.c)
		#endif
	}                              
	else if (PIR3bits.TX2IF && PIE3bits.TX2IE) // tx2 interrupt?
	{
		#ifdef ENABLE_SERIAL_PORT_TWO_TX
		Tx_2_Int_Handler(); // call the tx2 interrupt handler (in serial_ports.c)
		#endif
	}



//  ***  IFI Code Starts Here***
//                              
//  unsigned char int_byte;       
//  if (INTCON3bits.INT2IF && INTCON3bits.INT2IE)       /* The INT2 pin is RB2/DIG I/O 1. */
//  { 
//    INTCON3bits.INT2IF = 0;
//  }
//  else if (INTCON3bits.INT3IF && INTCON3bits.INT3IE)  /* The INT3 pin is RB3/DIG I/O 2. */
//  {
//    INTCON3bits.INT3IF = 0;
//  }
//  else if (INTCONbits.RBIF && INTCONbits.RBIE)  /* DIG I/O 3-6 (RB4, RB5, RB6, or RB7) changed. */
//  {
//    int_byte = PORTB;          /* You must read or write to PORTB */
//    INTCONbits.RBIF = 0;     /*     and clear the interrupt flag         */
//  }                                        /*     to clear the interrupt condition.  */
//  else
//  { 
//    CheckUartInts();    /* For Dynamic Debug Tool or buffered printf features. */
//  }
}


/*******************************************************************************
* FUNCTION NAME: User_Autonomous_Code
* PURPOSE:       Execute user's code during autonomous robot operation.
* You should modify this routine by adding code which you wish to run in
* autonomous mode.  It will be executed every program loop, and not
* wait for or use any data from the Operator Interface.
* CALLED FROM:   main.c file, main() routine when in Autonomous mode
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Autonomous_Code(void)
{

  /* Initialize all PWMs and Relays when entering Autonomous mode, or else it
     will be stuck with the last values mapped from the joysticks.  Remember, 
     even when Disabled it is reading inputs from the Operator Interface. 
  */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;
  relay1_fwd = relay1_rev = relay2_fwd = relay2_rev = 0;
  relay3_fwd = relay3_rev = relay4_fwd = relay4_rev = 0;
  relay5_fwd = relay5_rev = relay6_fwd = relay6_rev = 0;
  relay7_fwd = relay7_rev = relay8_fwd = relay8_rev = 0;

  while (autonomous_mode)   /* DO NOT CHANGE! */
  {
    if (statusflag.NEW_SPI_DATA)      /* 26.2ms loop area */
    {

	Getdata(&rxdata);
	Camera_Handler();
	Servo_Track();

		if(Get_Tracking_State() == CAMERA_ON_TARGET) //LED Light control for camera
		{
			Switch3_LED = 1;
		}
		else
		{
			Switch3_LED = 0;
		}

        /* Add your own autonomous code here. */
		if(Get_Tracking_State() == CAMERA_ON_TARGET || Get_Tracking_State() == TARGET_IN_VIEW)
		 {
			double distance; // slowpwm01, slowpwm02, slowpwm03, slowpwm04,
			int spin = rotation(PAN_SERVO);
			
			distance = 48/tan(servotoradians(TILT_SERVO)); // light height is at 116 minus camera height = 104

			if ((PAN_SERVO < 117) || (PAN_SERVO > 137)) //Left on the servo is negative
				{
					pwm01 = spin;
					pwm02 = spin;
					pwm03 = spin;
					pwm04 = spin;
				}

			if ((distance >= 24) && (PAN_SERVO > 117) && (PAN_SERVO < 137))
				{	
					int go, strafe;
					signed double speedMax, prepwm01, prepwm02, prepwm03, prepwm04;
					go = speed(distance);
					strafe = sidestep(PAN_SERVO);

					prepwm01 = go - spin - strafe; //looking at wheels counter-clockwise is postive
					prepwm02 = (254 - go) + spin + strafe; //to go right spin should be positive
					prepwm03 = go - spin + strafe;
					prepwm04 = (254 - go) + spin - strafe;

					speedMax = intMax(127, intABS(prepwm01)); //finds the maximum wheel value
  					speedMax = intMax(speedMax, intABS(prepwm02));
   					speedMax = intMax(speedMax, intABS(prepwm03));
   					speedMax = intMax(speedMax, intABS(prepwm04));
					
					prepwm01 = (prepwm01 * 127 / speedMax); //scales by the maximum wheel value
					prepwm02 = (prepwm02 * 127 / speedMax);
					prepwm03 = (prepwm03 * 127 / speedMax);
					prepwm04 = (prepwm04 * 127 / speedMax);

					if (prepwm01 > 127 && prepwm01 < 170)
						{
							prepwm01 += 12;
						}

					if (prepwm02 > 127 && prepwm02 < 170)
						{
							prepwm02 += 12;
						}

					if (prepwm03 > 127 && prepwm03 < 170)
						{
							prepwm03 += 12;
						}

					if (prepwm04 > 127 && prepwm04 < 170)
						{
							prepwm04 += 12;
						}

					pwm01 = prepwm01;
					pwm02 = prepwm02;
					pwm03 = prepwm03;
					pwm04 = prepwm04;
				}

/*			if((distance < 24) && (rc_dig_in02 != 1)) //run arm vertically; until it reaches second position (fourth limit switch)
				{
					relay3_fwd = 1;
					relay3_rev = 0;
				}
			else
				{
					relay3_fwd = 0; //otherwise stop belt engine
					relay3_rev = 0;
				} 

			if ((rc_dig_in04 == 1) && (rc_dig_in02 != 1)) //if arm is in second position run the rotation clockwise
				{
					relay2_fwd = 1;
					relay2_rev = 0;
				}
			else
				{
					relay2_fwd = 0; //otherwise stop rotation
					relay2_rev = 0;
				}

			if ((rc_dig_in04 == 1) && (rc_dig_in02 == 1)) //if arm is at the top and and rotated to the top
				{
					gripperrelease = 0;
				}

			relay1_fwd = gripperrelease;
			relay1_rev = 0;
*/
			if (distance <= 5)
				{
					pwm01 = pwm02 = pwm03 = pwm04 = 127;
				}

  			if(rc_dig_in05 == 1) //pressure switch
				{
					relay4_fwd = 0;
					relay4_rev = 0;
				}
			else
				{
					relay4_fwd = 1;
					relay4_rev = 0;
				}
		}

	else
		{
			pwm01 = pwm02 = pwm03 = pwm04 = 127;

  			if(rc_dig_in05 == 1)
				{
					relay4_fwd = 0;
					relay4_rev = 0;
				}
 			else
				{
					relay4_fwd = 1;
					relay4_rev = 0;
				}
		}

        PWM(pwm13,pwm14,pwm15,pwm16);

        Putdata(&txdata);   /* DO NOT DELETE, or you will get no PWM outputs! */
    }
  }
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Local_IO
* PURPOSE:       Execute user's realtime code.
* You should modify this routine by adding code which you wish to run fast.
* It will be executed every program loop, and not wait for fresh data 
* from the Operator Interface.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Local_IO(void)
{
  /* Add code here that you want to be executed every program loop. */

}

/*******************************************************************************
* FUNCTION NAME: Serial_Char_Callback
* PURPOSE:       Interrupt handler for the TTL_PORT.
* CALLED FROM:   user_SerialDrv.c
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     data        unsigned char    I    Data received from the TTL_PORT
* RETURNS:       void
*******************************************************************************/

void Serial_Char_Callback(unsigned char data)
{
  /* Add code to handle incomming data (remember, interrupts are still active) */
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
