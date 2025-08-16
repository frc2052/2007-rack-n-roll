/*******************************************************************************
* FILE NAME: user_routines.c <FRC VERSION>
*
* DESCRIPTION:
*  This file contains the default mappings of inputs  
*  (like switches, joysticks, and buttons) to outputs on the RC.  
*
* USAGE:
*  You can either modify this file to fit your needs, or remove it from your 
*  project and replace it with a modified copy. 
*
*******************************************************************************/

#include <stdio.h>

#include "ifi_aliases.h"
#include "ifi_default.h"
#include "ifi_utilities.h"
#include "user_routines.h"
#include "serial_ports.h"
#include "pwm.h"
#include "camera.h"
#include "camera_menu.h"
#include "tracking.h"
#include "tracking_menu.h"
#include "eeprom.h"
#include "terminal.h"


extern unsigned char aBreakerWasTripped;

int intMax(int x, int y)
{
	int max = x;
	if(y > max)
	{
   		max = y;
	}
   return max;
}

int intABS(int i)
{ 
	if (i < 0)
		return -i;
	else
		return i;
}

     rom const int ctbl[255] = {-127, -124.654754, -122.3407176, -120.0583477,
	-117.8073909, -115.5875939, -113.3987034, -111.2404659,
	-109.1126283, -107.0149371, -104.947139, -102.9089807, 
	-100.9002089, -98.92057018, -96.96981128, -95.04767885, 
	-93.15391954, -91.28828005, -89.45050702, -87.64034713, 
	-85.85754704, -84.10185344, -82.37301297, -80.67077232, 
	-78.99487816, -77.34507714, -75.72111594, -74.12274122, 
	-72.54969966, -71.00173793, -69.47860269, -67.9800406,
	-66.50579835, -65.05562259, -63.62926, -62.22645725, 
	-60.84696099, -59.49051791, -58.15687466, -56.84577793, 
	-55.55697437, -54.29021066, -53.04523346, -51.82178944, 
	-50.61962527, -49.43848762, -48.27812316, -47.13827855, 
	-46.01870047, -44.91913557, -43.83933055, -42.77903204, 
	-41.73798674, -40.71594131, -39.71264241, -38.72783671, 
	-37.76127088, -36.81269159, -35.88184552, -34.96847932, 
	-34.07233966, -33.19317322, -32.33072666, -31.48474665, 
	-30.65497986, -29.84117296, -29.04307261, -28.26042549, 
	-27.49297827, -26.7404776, -26.00267016, -25.27930263, 
	-24.57012165, -23.87487392, -23.19330609, -22.52516483,
	-21.87019681, -21.2281487, -20.59876716, -19.98179888, 
	-19.37699051, -18.78408872, -18.20284018, -17.63299156, 
	-17.07428953, -16.52648076, -15.98931191, -15.46252965, 
	-14.94588066, -14.4391116, -13.94196914, -13.45419994, 
	-12.97555068, -12.50576802, -12.04459864, -11.5917892, 
	-11.14708637, -10.71023681, -10.28098721, -9.859084213, 
	-9.444274504, -9.036304746, -8.634921609, -8.239871761, 
	-7.85090187, -7.467758605, -7.090188633, -6.717938625, 
	-6.350755247, -5.988385169, -5.630575058, -5.277071584, 
	-4.927621415, -4.581971219, -4.239867665, -3.901057421, 
	-3.565287155, -3.232303537, -2.901853234, -2.573682915, 
	-2.247539249, -1.923168903, -1.600318547, -1.278734848, 
	-0.958164476, -0.638354098, -0.319050383, 0, 
	0.319050383, 0.638354098, 0.958164476, 1.278734848, 
	1.600318547, 1.923168903, 2.247539249, 2.573682915, 
	2.901853234, 3.232303537, 3.565287155, 3.901057421, 
	4.239867665, 4.581971219, 4.927621415, 5.277071584, 
	5.630575058, 5.988385169, 6.350755247, 6.717938625, 
	7.090188633, 7.467758605, 7.85090187, 8.239871761, 
	8.634921609, 9.036304746, 9.444274504, 9.859084213, 
	10.28098721, 10.71023681, 11.14708637, 11.5917892, 
	12.04459864, 12.50576802, 12.97555068, 13.45419994, 
	13.94196914, 14.4391116, 14.94588066, 15.46252965, 
	15.98931191, 16.52648076, 17.07428953, 17.63299156, 
	18.20284018, 18.78408872, 19.37699051, 19.98179888, 
	20.59876716, 21.2281487, 21.87019681, 22.52516483, 
	23.19330609, 23.87487392, 24.57012165, 25.27930263, 
	26.00267016, 26.7404776, 27.49297827, 28.26042549, 
	29.04307261, 29.84117296, 30.65497986, 31.48474665, 
	32.33072666, 33.19317322, 34.07233966, 34.96847932, 
	35.88184552, 36.81269159, 37.76127088, 38.72783671, 
	39.71264241, 40.71594131, 41.73798674, 42.77903204, 
	43.83933055, 44.91913557, 46.01870047, 47.13827855, 
	48.27812316, 49.43848762, 50.61962527, 51.82178944, 
	53.04523346, 54.29021066, 55.55697437, 56.84577793, 
	58.15687466, 59.49051791, 60.84696099, 62.22645725, 
	63.62926, 65.05562259, 66.50579835, 67.9800406, 
	69.47860269, 71.00173793, 72.54969966, 74.12274122, 
	75.72111594, 77.34507714, 78.99487816, 80.67077232, 
	82.37301297, 84.10185344, 85.85754704, 87.64034713, 
	89.45050702, 91.28828005, 93.15391954, 95.04767885, 
	96.96981128, 98.92057018, 100.9002089, 102.9089807, 
	104.947139, 107.0149371, 109.1126283, 111.2404659,
	113.3987034, 115.5875939, 117.8073909, 120.0583477, 
	122.3407176, 124.654754, 127};


int joysticklimit(int inp)
{
	if(inp < 127)
		{
			inp += 1;
			return inp;
		}
	else if (inp > 127)
		{
			inp -= 1;
			return inp;
		}
	else
		{
			return inp;
		}
}

int count = 0;
int nestedcount = 0;
static int oldcount = 0;
int countvert = 0;
static int oldcountvert = 0;
int countvert2 = 0;
static int oldcountvert2 = 0;
int count3 = 0;
static int oldcount3 = 0;
int gripper = 0;
int rotationfw = 0;
int rotationrv = 0;
int groundlevel = 0;
int toplevel = 0;

/*** DEFINE USER VARIABLES AND INITIALIZE THEM HERE ***/
/* EXAMPLES: (see MPLAB C18 User's Guide, p.9 for all types)
unsigned char wheel_revolutions = 0; (can vary from 0 to 255)
unsigned int  delay_count = 7;       (can vary from 0 to 65,535)
int           angle_deviation = 142; (can vary from -32,768 to 32,767)
unsigned long very_big_counter = 0;  (can vary from 0 to 4,294,967,295)
*/


/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Max
* PURPOSE:       Sets a PWM value to neutral (127) if it exceeds 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Max(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value > 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Switch_Min
* PURPOSE:       Sets a PWM value to neutral (127) if it's less than 127 and the
*                limit switch is on.
* CALLED FROM:   this file
* ARGUMENTS:     
*     Argument       Type             IO   Description
*     --------       -------------    --   -----------
*     switch_state   unsigned char    I    limit switch state
*     *input_value   pointer           O   points to PWM byte value to be limited
* RETURNS:       void
*******************************************************************************/
void Limit_Switch_Min(unsigned char switch_state, unsigned char *input_value)
{
  if (switch_state == CLOSED)
  { 
    if(*input_value < 127)
      *input_value = 127;
  }
}


/*******************************************************************************
* FUNCTION NAME: Limit_Mix
* PURPOSE:       Limits the mixed value for one joystick drive.
* CALLED FROM:   Default_Routine, this file
* ARGUMENTS:     
*     Argument             Type    IO   Description
*     --------             ----    --   -----------
*     intermediate_value    int    I    
* RETURNS:       unsigned char
*******************************************************************************/
unsigned char Limit_Mix (int intermediate_value)
{
  static int limited_value;
  
  if (intermediate_value < 2000)
  {
    limited_value = 2000;
  }
  else if (intermediate_value > 2254)
  {
    limited_value = 2254;
  }
  else
  {
    limited_value = intermediate_value;
  }
  return (unsigned char) (limited_value - 2000);
}


/*******************************************************************************
* FUNCTION NAME: User_Initialization
* PURPOSE:       This routine is called first (and only once) in the Main function.  
*                You may modify and add to this function.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void User_Initialization (void)
{
  Set_Number_of_Analog_Channels(SIXTEEN_ANALOG);    /* DO NOT CHANGE! */

/* FIRST: Set up the I/O pins you want to use as digital INPUTS. */
  digital_io_01 = digital_io_02 = digital_io_03 = digital_io_04 = INPUT;
  digital_io_05 = digital_io_06 = digital_io_07 = digital_io_08 = INPUT;
  digital_io_09 = digital_io_10 = digital_io_11 = digital_io_12 = INPUT;
  digital_io_13 = digital_io_14 = digital_io_15 = digital_io_16 = INPUT;
  digital_io_18 = INPUT;  /* Used for pneumatic pressure switch. */

  relay1_fwd = relay1_rev = relay2_fwd = relay2_rev = 0;
  relay3_fwd = relay3_rev = relay4_fwd = relay4_rev = 0;
  relay5_fwd = relay5_rev = relay6_fwd = relay6_rev = 0;
  relay7_fwd = relay7_rev = relay8_fwd = relay8_rev = 0;

    /* 
     Note: digital_io_01 = digital_io_02 = ... digital_io_04 = INPUT; 
           is the same as the following:

           digital_io_01 = INPUT;
           digital_io_02 = INPUT;
           ...
           digital_io_04 = INPUT;
    */

/* SECOND: Set up the I/O pins you want to use as digital OUTPUTS. */
  digital_io_17 = OUTPUT;    /* Example - Not used in Default Code. */

/* THIRD: Initialize the values on the digital outputs. */
  rc_dig_out17 = 0;

/* FOURTH: Set your initial PWM values.  Neutral is 127. */
  pwm01 = pwm02 = pwm03 = pwm04 = pwm05 = pwm06 = pwm07 = pwm08 = 127;
  pwm09 = pwm10 = pwm11 = pwm12 = pwm13 = pwm14 = pwm15 = pwm16 = 127;

/* FIFTH: Set your PWM output types for PWM OUTPUTS 13-16.
  /*   Choose from these parameters for PWM 13-16 respectively:               */
  /*     IFI_PWM  - Standard IFI PWM output generated with Generate_Pwms(...) */
  /*     USER_CCP - User can use PWM pin as digital I/O or CCP pin.           */
//  Setup_PWM_Output_Type(IFI_PWM,IFI_PWM,IFI_PWM,IFI_PWM);

  // changed so PWM() can control PWM outputs 13 through 16
  Setup_PWM_Output_Type(USER_CCP,USER_CCP,USER_CCP,USER_CCP);

  /* 
     Example: The following would generate a 40KHz PWM with a 50% duty cycle on the CCP2 pin:

         CCP2CON = 0x3C;
         PR2 = 0xF9;
         CCPR2L = 0x7F;
         T2CON = 0;
         T2CONbits.TMR2ON = 1;

         Setup_PWM_Output_Type(USER_CCP,IFI_PWM,IFI_PWM,IFI_PWM);
  */

  /* Add any other initialization code here. */

  // initialize the CCP PWM hardware
  Initialize_PWM();
 
  // initialize the serial ports
  Init_Serial_Port_One();
  Init_Serial_Port_Two();

  // make sure printf() output goes to the proper port
  #ifdef TERMINAL_SERIAL_PORT_1    
  stdout_serial_port = SERIAL_PORT_ONE;
  #endif

  #ifdef TERMINAL_SERIAL_PORT_2    
  stdout_serial_port = SERIAL_PORT_TWO;
  #endif

  Putdata(&txdata);            /* DO NOT CHANGE! */

  User_Proc_Is_Ready();         /* DO NOT CHANGE! - last line of User_Initialization */
}

/*******************************************************************************
* FUNCTION NAME: Process_Data_From_Master_uP
* PURPOSE:       Executes every 26.2ms when it gets new data from the master 
*                microprocessor.
* CALLED FROM:   main.c
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Process_Data_From_Master_uP(void)
{
	static unsigned char count = 0;
	static unsigned char camera_menu_active = 0;
	static unsigned char tracking_menu_active = 0;
	unsigned char terminal_char;
	unsigned char returned_value;

	// don't move this unless you know what you're doing
	Getdata(&rxdata);

	// send diagnostic information to the terminal, but don't 
	// overwrite the camera or tracking menu if it's active
	if(camera_menu_active == 0 && tracking_menu_active == 0)
	{
		Tracking_Info_Terminal();
	}

	// This function is responsable for camera initialization 
	// and camera serial data interpretation. Once the camera
	// is initialized and starts sending tracking data, this 
	// function will continuously update the global T_Packet_Data 
	// structure with the received tracking information.
	Camera_Handler();

	// This function reads data placed in the T_Packet_Data
	// structure by the Camera_Handler() function and if new
	// tracking data is available, attempts to keep the center
	// of the tracked object in the center of the camera's
	// image using two servos that drive a pan/tilt platform.
	// If the camera doesn't have the object within it's field 
	// of view, this function will execute a search algorithm 
	// in an attempt to find the object.
	if(tracking_menu_active == 0)
	{
		Servo_Track();
	}

	// Turn on the "Switch 3" LED on the operator interface if
	// the camera is pointed at the green light target. The
	// Switch3_LED variable is also used by the Default_Routine()
	// function below, so disable it inside Default_Routine()
	// if you want to keep this functionality. 
	if(Get_Tracking_State() == CAMERA_ON_TARGET)
	{
		Switch3_LED = 1;
	}
	else
	{
		Switch3_LED = 0;
	}

	// this logic guarantees that only one of the menus can be
	// active at any giiven time
	if(camera_menu_active == 1)
	{
		// This function manages the camera menu functionality,
		// which is used to enter camera initialization and
		// color tracking parameters.
		camera_menu_active = Camera_Menu();
	}
	else if(tracking_menu_active == 1)
	{
		// This function manages the tracking menu functionality,
		// which is used to enter parameters that describe how
		// the pan and tilt servos will behave while in searching
		// and tracking modes.
		tracking_menu_active = Tracking_Menu();
	}
	else
	{
		// has the user sent any data via the terminal?
		terminal_char = Read_Terminal_Serial_Port();
		// check to see if any "hotkeys" have been pressed
		if(terminal_char == CM_SETUP_KEY)
		{
			camera_menu_active = 1;
		}
		else if(terminal_char == TM_SETUP_KEY)
		{
			tracking_menu_active = 1;
		}
	}

	// This funtion is used by the functions Camera_Menu() and
	// Tracking_Menu() to manage the writing of initialization
	// parameters to your robot controller's non-volatile
	// Electrically Erasable Programmable Read-Only Memory
	// (EEPROM)
	EEPROM_Write_Handler();


	// IFI's default routine is commented out for safety reasons
	// and because it also tries to use PWM outputs one and two,
	// which conflicts with the default assignment for the pan
	// and tilt servos.
    Default_Routine();

	// IFI's software based PWM pulse generator for PMW ouputs
	// 13 through 16. This has been replaced with a hardware-
	// based solution, PWM(), below.
//	Generate_Pwms(pwm13,pwm14,pwm15,pwm16);

	// see pwm_readme.txt for information about PWM();
	PWM(pwm13,pwm14,pwm15,pwm16);

	// don't move this unless you know what you're doing
	Putdata(&txdata);
}

/*******************************************************************************
* FUNCTION NAME: Default_Routine
* PURPOSE:       Performs the default mappings of inputs to outputs for the
*                Robot Controller.
* CALLED FROM:   this file, Process_Data_From_Master_uP routine
* ARGUMENTS:     none
* RETURNS:       void
*******************************************************************************/
void Default_Routine(void)
{

  	signed double wFL, wFR, wRL, wRR, velocity, rotation, strafe, wMax;
	signed int difference, difference2, difference3, difference4;
	signed int prepwm01, prepwm02, prepwm03, prepwm04;
	static char pwm01_1, pwm02_1, pwm03_1, pwm04_1;

	/*velocity = (p1_y - 127);
	rotation = (p1_x - 127);
	strafe = (p2_x - 127);*/

	/*velocity = deadband(p1_y,127,15);
	rotation = deadband(p1_x,127,15);
	strafe = deadband(p2_x,127,15);*/
	
	velocity = ctbl[joysticklimit(p1_y)];
	rotation = ctbl[joysticklimit(p1_x)];
	strafe = ctbl[joysticklimit(p2_x)];
	
	wFL = velocity - rotation - strafe;
	wFR = velocity + rotation + strafe;
	wRL = velocity - rotation + strafe;
	wRR = velocity + rotation - strafe;

   wMax = intMax(127, intABS(wFL));
   wMax = intMax(wMax, intABS(wFR));
   wMax = intMax(wMax, intABS(wRL));
   wMax = intMax(wMax, intABS(wRR));


	wFL = (wFL * 127 / wMax);
	if (-127 > wFL || wFL > 127)
		 printf("Received an unexpected number [wFL] /n");
else
	{
		wFR = (wFR * 127 / wMax);
		if (-127 > wFR || wFR > 127)
		 	printf("Received an unexpected number [wFR] /n");
	
		else
			{
				wRL = (wRL * 127 / wMax);
				if (-127 > wRL || wRL > 127)
				printf("Received an unexpected number [wRL] /n");
				else
					{
						wRR = (wRR * 127 / wMax);
						if (-127 > wRR || wRR > 127)	
	 					printf("Received an unexpected number [wRR] /n");
							else{

	prepwm01 = wFL + 127;
	prepwm02 = 127 - wFR;
	prepwm03 = wRL + 127;
	prepwm04 = 127 - wRR;	

		// looking at the wheels counter clockwise is postive
		// 7 is too little, 
	
	if ((prepwm01 > 127) && (prepwm01 < 180))	{
		prepwm01 = prepwm01 + 12;
	}

	if ((prepwm02 > 127) && (prepwm02 < 180))	{
		prepwm02 = prepwm02 + 12;
	}

	if ((prepwm03 > 127) && (prepwm03 < 180))	{
		prepwm03 = prepwm03 + 12;
	}

	if ((prepwm04 > 127) && (prepwm03 < 180))	{
		prepwm04 = prepwm04 + 12;
	}

	pwm01 = prepwm01;
	pwm02 = prepwm02;
	pwm03 = prepwm03;
	pwm04 = prepwm04;

	/*if (((intABS((char)pwm01 - pwm01_1)) > 0 ) || ((intABS((char)pwm02 - pwm02_1)) > 0 ) || ((intABS((char)pwm03 - pwm03_1)) > 0 ) || ((intABS((char)pwm04 - pwm04_1)) > 0 )) {
		printf(" pwm01: %d , pwm02: %d , pwm03: %d , pwm04: %d \r", pwm01, pwm02, pwm03, pwm04);
		printf(" Port 1 Y: %d , Port 1 X: %d , Port 2 X: %d \r", p1_y, p1_x, p2_x);
	}

	pwm01_1 = intABS(pwm01 - 127);
	pwm02_1 = intABS(pwm02 - 127);
	pwm03_1 = intABS(pwm03 - 127);
	pwm04_1 = intABS(pwm04 - 127);*/

  
 /*---------- Buttons to Relays----------------------------------------------
  *--------------------------------------------------------------------------
  *  This default code maps the joystick buttons to specific relay outputs.  
  *  Relays 1 and 2 use limit switches to stop the movement in one direction.
  *  The & used below is the C symbol for AND                                
  */

  count3 = p3_sw_trig;
  if ((oldcount3 == 0) && ((oldcount3 + count3) == 1)) 
	{
		gripper = intABS(gripper -= 1);
	}
 oldcount3 = count3;

		relay1_fwd = gripper; //relay1 controls the grippers pneumatics
		relay1_rev = 0;

  count = p3_sw_top;
  if((oldcount == 0) && ((oldcount + count) == 1))
	{
		nestedcount += 1;
		if((nestedcount % 2) == 1)
		{
			rotationfw = 1; //forward goes counter clockwise up
			rotationrv = 0; //reverse goes clockwise down
		}
		else
		{
			rotationfw = 0;
			rotationrv = 1;
		}
	}

  if (rc_dig_in01 == 0) //limit switch on the bottom end of the rotation
	{
		rotationrv = 0;
	}
  if (rc_dig_in03 == 0) //limit switch on the top of the rotation
	{
		rotationfw = 0;
	}

	oldcount = count;

	relay2_fwd = rotationfw; //relay2 controls the rotation of the arm
	relay2_rev = rotationrv;

  countvert = p3_sw_aux1;
  if((oldcountvert == 0) && ((oldcountvert + countvert) == 1) && (rc_dig_in05 != 0))
	{
		toplevel = 0;
		groundlevel = 1;
	}

  else if (rc_dig_in05 == 0); //limit switch on the bottom of the vert travel
		{
			groundlevel = 0;
		}
  oldcountvert = countvert;

  countvert2 = p3_sw_aux2;
  if((oldcountvert2 == 0) && ((oldcountvert2 + countvert2) == 1) && (rc_dig_in07 != 0))
	{
		toplevel = 1;
		groundlevel = 0;
	}
  
  else if (rc_dig_in07 == 0) // limit switch on top of the vert travel
	{
		toplevel = 0;
	}
  oldcountvert2 = countvert2;

	relay3_fwd = toplevel; //relay3 controls the up and down motion of the arm
	relay3_rev = groundlevel;
	

  if(rc_dig_in09 == 1) //pressure switch for compressor
		{
			relay4_fwd = 0;
			relay4_rev = 0;
		}
  else
		{
			relay4_fwd = 1;
			relay4_rev = 0;
		}

 /*---------- ROBOT FEEDBACK LEDs------------------------------------------------
  *------------------------------------------------------------------------------
  *   This section drives the "ROBOT FEEDBACK" lights on the Operator Interface.
  *   The lights are green for joystick forward and red for joystick reverse.
  *   Both red and green are on when the joystick is centered.  Use the
  *   trim tabs on the joystick to adjust the center.     
  *   These may be changed for any use that the user desires.                       
  */	
  
  if (user_display_mode == 0) /* User Mode is Off */
    
  { /* Check position of Port 1 Joystick */
    if (p1_y >= 0 && p1_y <= 56)
    {                     /* Joystick is in full reverse position */
      Pwm1_green  = 0;    /* Turn PWM1 green LED - OFF */
      Pwm1_red  = 1;      /* Turn PWM1 red LED   - ON  */
    }
    else if (p1_y >= 125 && p1_y <= 129)
    {                     /* Joystick is in neutral position */
      Pwm1_green  = 1;    /* Turn PWM1 green LED - ON */
      Pwm1_red  = 1;      /* Turn PWM1 red LED   - ON */
    }
    else if (p1_y >= 216 && p1_y <= 255)
    {                     /* Joystick is in full forward position*/
      Pwm1_green  = 1;    /* Turn PWM1 green LED - ON  */
      Pwm1_red  = 0;      /* Turn PWM1 red LED   - OFF */
    }
    else
    {                     /* In either forward or reverse position */
      Pwm1_green  = 0;    /* Turn PWM1 green LED - OFF */
      Pwm1_red  = 0;      /* Turn PWM1 red LED   - OFF */
    }  /*END Check position of Port 1 Joystick
    
    /* Check position of Port 2 Y Joystick 
           (or Port 1 X in Single Joystick Drive Mode) */
    if (p2_y >= 0 && p2_y <= 56)
    {                     /* Joystick is in full reverse position */
      Pwm2_green  = 0;    /* Turn pwm2 green LED - OFF */
      Pwm2_red  = 1;      /* Turn pwm2 red LED   - ON  */
    }
    else if (p2_y >= 125 && p2_y <= 129)
    {                     /* Joystick is in neutral position */
      Pwm2_green  = 1;    /* Turn PWM2 green LED - ON */
      Pwm2_red  = 1;      /* Turn PWM2 red LED   - ON */
    }
    else if (p2_y >= 216 && p2_y <= 255)
    {                     /* Joystick is in full forward position */
      Pwm2_green  = 1;    /* Turn PWM2 green LED - ON  */
      Pwm2_red  = 0;      /* Turn PWM2 red LED   - OFF */
    }
    else
    {                     /* In either forward or reverse position */
      Pwm2_green  = 0;    /* Turn PWM2 green LED - OFF */
      Pwm2_red  = 0;      /* Turn PWM2 red LED   - OFF */
    }  /* END Check position of Port 2 Joystick */
    
    /* This drives the Relay 1 and Relay 2 "Robot Feedback" lights on the OI. */
    Relay1_green = relay1_fwd;    /* LED is ON when Relay 1 is FWD */
    Relay1_red = relay1_rev;      /* LED is ON when Relay 1 is REV */
    Relay2_green = relay2_fwd;    /* LED is ON when Relay 2 is FWD */
    Relay2_red = relay2_rev;      /* LED is ON when Relay 2 is REV */

    Switch1_LED = !(int)rc_dig_in01;
    Switch2_LED = !(int)rc_dig_in02;
    Switch3_LED = !(int)rc_dig_in03;
    
  } /* (user_display_mode = 0) (User Mode is Off) */
  
  else  /* User Mode is On - displays data in OI 4-digit display*/
  {
    User_Mode_byte = backup_voltage*10; /* so that decimal doesn't get truncated. */
  }   
  
} /* END Default_Routine(); */
}
}
}
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
