/*Timer event driven (T=10ms) gesture control process plus the necessary math. 
MISSING: 
	TIMER, COMMUNICATION WITH BLE STACK (see pseudocode comments)
TODO: 
- Move declarations to pseudo_motion_sensor.h
- Global variables used here might be a problem
- Lot of cleanup
- ble_operation_in_progress flag needs to be implemented
- math libraries need to be compiled linked somewhere
- check if the enums and ints are compatible(e.g. current_motion_sensor_state)

*/

#include "math.h"
#include "pseudo_motion_sensor.h"
#include "sw_spi.h"
#include <stdint.h>
#include "nrf_gpio.h"

#define IMU_INT1				9 //WHY IS THIS HERE?

double calcDx();
double calcDy();
double calcDz();
double calcG();
double calcDg();
double calcTheta(); // Deviation from horizontal in degrees (-90,90). Up positive.
double calcPhi(); // Deviation from semipronation in degrees (-90,90). Pronation positive.
double calcYaw(); // Angular speed in approximately horizontal plane in degrees per second. Clockwise positive.
int armIsStationary();
int armIsNeutral();
int armIsUp();
int armIsDown();
int armRotation(); // 1 = CW activate, -1 = CCW activate, 0 = neither
int swipeRight();
int swipeLeft();

struct accel acc;
struct accel_int acc_temp;
struct accel gyro;
struct accel_int gyro_temp;

int n = SAVED_ACCELERATION_BUFFER_SIZE - 1;  //last value position
double dxv;
double dyv;
double dzv;
double gv;
double dgv;
double thetav;
double phiv;
double yawv;

double dx;
double dy;
double dz;
double g;
double dg;
double theta; // Deviation from horizontal in degrees (-90,90). Up positive.
double phi; // Deviation from semipronation in degrees (-90,90). Clockwise positive.
double yaw; // Angular speed in approximately horizontal plane in degrees per second. Clockwise positive.

// Static configuration values
double dg_limit = 0.1;
double g_limit = 0.2;
double yaw_limit = 50;
double theta_high = 45.0; // degree
double theta_neutral_high = 20.0; // theta_high/2 degree
double theta_limit_high = 35.0; // degree, above this blocks phi recognition
double theta_low = -60.0; // degree
double theta_neutral_low = -45.0; // degree
double theta_limit_low = -45.0; // degree, below this blocks phi recognition
double phi_cw = 40.0; // degree
double phi_ccw = -40.0; // degree
double phi_neutral_cw = 20; //clockwise activation angle, phi_activate/2 degree
double phi_neutral_ccw = -20; //counterclockwise activation angle, phi_activate/2 degree
double yaw_activate = 200.0; // degree per second
int stationary_limit = 5; // How many data points arm needs to be relatively stationary before recognizing gestures. ~10ms/data point
int verbose_vibrations = 0; // Boolean. Verbose vibrations for interface testing without receivers.
int three_command_mode = 0; // Boolean
int no_continuous = 0; // Boolean. Set to 1 if user cannot reliably return from pronation/supination.
int short_vibe_ticks = 10;
int long_vibe_ticks = 20;
int looong_vibe_ticks = 24;

// Values that change during runtime
//Motion_sensor_states
int current_motion_sensor_state = GETTING_START_STATE; 
//Motion_sensor_states 
int next_state = GETTING_START_STATE; //For buzzes and delays
//Vibration_modes 
int vibration_mode = VIBRATION_MODE_NORMAL;
int buzz_buzz_state = 0;
int delay_ticks = 0;
int delay_ticks_after_vibration = 0;
int orientation = -1; // 0 = neutral, 1 = up, 2 = CCW, 3 = CW, 4 = down, -1 = not defined
int stationary_counter = 0;
int gyro_enabled = 0; // Boolean
int device_commands = 5; // Initialise to 0 for actual operation. Valid values 1, 3 or 5.
int primary_continuous = 0; // Boolean value received from receivers. Volume etc: 1, channel etc: 0
int ble_operation_in_progress = 0; //If 0, BLE connected and idle. 

//////////////////
///IMU HANDLING///
//////////////////

void getXYZValues() 
{
	uint16_t tx_buffer[6];
	uint8_t rx_buffer[6];

	tx_buffer[0] = 0x2800 | 0x8000;
	tx_buffer[1] = 0x2900 | 0x8000;
	tx_buffer[2] = 0x2A00 | 0x8000;
	tx_buffer[3] = 0x2B00 | 0x8000;
	tx_buffer[4] = 0x2C00 | 0x8000;
	tx_buffer[5] = 0x2D00 | 0x8000;
	spi_sw_master_send_bytes(tx_buffer, rx_buffer, 6);

	acc.x[0] = acc.x[1];
	acc.y[0] = acc.y[1];
	acc.z[0] = acc.z[1];
	acc_temp.x[0] = (rx_buffer[0] + rx_buffer[1] * 0x0100);
	acc_temp.y[0] = (rx_buffer[2] + rx_buffer[3] * 0x0100);
	acc_temp.z[0] = (rx_buffer[4] + rx_buffer[5] * 0x0100);

	// Normitus s.e. g = 1 ja poistetaan datasta nollat
	acc.x[1] = -acc_temp.x[0] / (double) 0x2000 + 0.00001; // +x = kyynärpäätä kohti
	acc.y[1] = acc_temp.y[0] / (double) 0x2000 + 0.00001; //
	acc.z[1] = -acc_temp.z[0] / (double) 0x2000 + 0.00001; // +z = kämmentä kohti
	dx = calcDx();
	dy = calcDy();
	dz = calcDz();
	g = calcG();
	dg = calcDg();
	theta = calcTheta();
	phi = calcPhi();
	if (!gyro_enabled) 
	{
		yaw = 0;
	}
}

void getGyroValues() 
{
	// Yaw only
	uint16_t tx_buffer[2];
	uint8_t rx_buffer[2];

	tx_buffer[0] = 0x2400 | 0x8000;
	tx_buffer[1] = 0x2500 | 0x8000;
	spi_sw_master_send_bytes(tx_buffer, rx_buffer, 2);

	gyro.y[0] = gyro.y[1];
	gyro_temp.y[0] = (rx_buffer[0] + rx_buffer[1] * 0x0100);
	gyro.y[1] = -gyro_temp.y[0] / 114.28 + 0.001; // Yaw in degrees per second
	yaw = calcYaw(); // If !gyro_enabled: yaw = 0
}

void gyroEnable() 
{
	uint16_t tx_buffer2[2]; //Transmit buffer to send data from SPI master with sample data.
	uint16_t *tx2 = tx_buffer2;

	tx_buffer2[0] = 0x1100 | 0x40; // CTRL2_G: Set gyro 104 Hz, 245 dps full scale.
	tx_buffer2[1] = 0x0000;
	spi_sw_master_send_bytes(tx2, NULL, 2);
	gyro_enabled = 1;
}

void gyroDisable() 
{	
	uint16_t tx_buffer2[2]; //Transmit buffer to send data from SPI master with sample data.
	uint16_t *tx2 = tx_buffer2;

	tx_buffer2[0] = 0x1100 | 0x00; // CTRL2_G: Set gyro power down.
	tx_buffer2[1] = 0x0000;
	spi_sw_master_send_bytes(tx2, NULL, 2);
	gyro_enabled = 0;
}

////////////////////
///STATE HANDLERS///
////////////////////

void delay()
{
	if (delay_ticks <= 0)
		current_motion_sensor_state = next_state;
}

void vibrate()
{
	if (vibration_mode == VIBRATION_MODE_NORMAL)
	{
		if (delay_ticks <= 0)
		{
			nrf_gpio_pin_clear(10);
			if (delay_ticks_after_vibration > 0) //Horrible way of combining vibration & delay
			{
				DELAY_WITH_STATE_CHANGE(delay_ticks_after_vibration,next_state);
				delay_ticks_after_vibration = 0;
			}
			else
				current_motion_sensor_state = next_state;
		}
		else
			nrf_gpio_pin_set(10); //This will set the pin multiple times, but it shouldn't matter
	}
	else if (vibration_mode == VIBRATION_MODE_BUZZ_BUZZ)
	{
		if (buzz_buzz_state == 0)
		{
			if (delay_ticks <= 0)
			{
				nrf_gpio_pin_clear(10);
				buzz_buzz_state = 1;
				delay_ticks = 15;
			}
			else
				nrf_gpio_pin_set(10);
		}
		else //The pause part
		{
			if (delay_ticks <= 0)
			{
				buzz_buzz_state = 0; //Unnecessary
				delay_ticks = short_vibe_ticks;
				vibration_mode = VIBRATION_MODE_NORMAL; //Vibrate rest in normal mode
			}
		}
	}
}

/*Starting point */
void gestureControl() 
{
	buzz_buzz_state = 0;
	delay_ticks = 0;
	delay_ticks_after_vibration = 0;
	orientation = -1; 
	stationary_counter = 0;
	gyro_enabled = 0; 
	VIBRATION_WITH_STATE_CHANGE(short_vibe_ticks,GETTING_START_STATE,VIBRATION_MODE_BUZZ_BUZZ,20);
}

void endGestureControl()
{
	if (gyro_enabled)
		gyroDisable();
	buzz_buzz_state = 0;
	delay_ticks = 0;
	delay_ticks_after_vibration = 0;
	orientation = -1; 
	stationary_counter = 0;
	//Stop reading more values (stop timer?)
	//disconnect ble
}

void timer_10ms_tick_handler()
{
	if (delay_ticks > 0)
		delay_ticks--;
	
	switch(current_motion_sensor_state)
	{
		case GETTING_START_STATE:
			getStartingState();
			break;
		case RECEIVER_SELECTION:
			selectReceiver();
			break;
		case RECEIVER_CONTROL:
			controlReceiver();
			break;
		case SHUTDOWN_STATE:
			endGestureControl();
			break;
		case VIBRATION_STATE:
			vibrate();
			break;
		case DELAY_STATE:
			delay();
			break;
	}
}

void getStartingState() 
{
	if (!ble_operation_in_progress) //Don't do anything if we are in the middle of doing something with the ble connections
	{
		getXYZValues();
		if (armIsStationary()) 
		{
			if (armIsNeutral()) 
			{ //tarkistetaan onko ranneke neutraaliasennossa
				orientation = 0;
				current_motion_sensor_state = RECEIVER_SELECTION;
			} 
			else if (armIsUp()) 
			{   //käden nosto
				current_motion_sensor_state = RECEIVER_SELECTION;
				orientation = 1;
			} 
			else if (armRotation() == -1) 
			{   //pronaatio
				current_motion_sensor_state = RECEIVER_SELECTION;
				orientation = 2;
			} 
			else if (armRotation() == 1) 
			{   //supinaatio
				orientation = 3;
				current_motion_sensor_state = RECEIVER_SELECTION;
			} 
			else if (armIsDown()) 
			{   //käden lasku
				orientation = 4;
				current_motion_sensor_state = RECEIVER_SELECTION; //This is normally shutdown
			}
		}	
	}
}

void selectReceiver()
{
		getXYZValues();
		if (!ble_operation_in_progress) //Don't do anything if we are in the middle of doing something with the ble connections
		{
			if (0)//double_tap_received)  // Double tap. Only if not controlling a receiver. Interrupt lasts for 120 ms so should be readable without latching.
			{
				//double_tap_received = 0;
				//changeRoom(); //Pseudocode
				//New ble connection
				//No buzz?
			}
			else if (armIsStationary()) 
			{
				if (armIsNeutral())
				{ //check if controller is neutral position
					orientation = 0;
				} 
				else if (orientation != 1 && armIsUp()) 
				{   // forearm up 
					VIBRATION_WITH_STATE_CHANGE(long_vibe_ticks,RECEIVER_CONTROL,VIBRATION_MODE_NORMAL,10);
					//current_motion_sensor_state = RECEIVER_CONTROL;				
					orientation = 1;
				} 
				else if (orientation != 1 && orientation != 2 && (armRotation() == -1)) 
				{   // ccw 
					//previousReceiver(); //pseudocode
					if (verbose_vibrations) 
					{
						VIBRATION_WITH_STATE_CHANGE(short_vibe_ticks,RECEIVER_SELECTION,VIBRATION_MODE_NORMAL,30);
					}
				}
				else if (orientation != 1 && orientation != 3 && (armRotation() == 1)) 
				{   // cw
					//nextReceiver(); //pseudocode
					if (verbose_vibrations) 
					{
						VIBRATION_WITH_STATE_CHANGE(short_vibe_ticks,RECEIVER_SELECTION,VIBRATION_MODE_NORMAL,30);
					}	
					orientation = 3;
				} 
				else if (orientation != 4 && armIsDown())
				{   // forearm down
					orientation = 4;

					VIBRATION_WITH_STATE_CHANGE(looong_vibe_ticks,SHUTDOWN_STATE,VIBRATION_MODE_NORMAL,0);
					/*if (verbose_vibrations) 
					{
						VIBRATION_WITH_STATE_CHANGE(looong_vibe_ticks,SHUTDOWN_STATE,VIBRATION_MODE_NORMAL,0);
					}
					else
						current_motion_sensor_state = SHUTDOWN_STATE;
					*/
				}
			}
		}
}

void controlReceiver()
{
	if (device_commands == 5 && !gyro_enabled) //Enable gyro for swipes
	{
		gyroEnable();
		gyro_enabled = 1;
	}
	
	if (device_commands == 1) //Select also activates the only function and shuts the device down
	{
		//sendToggle(); //pseudocode
		VIBRATION_WITH_STATE_CHANGE(looong_vibe_ticks,SHUTDOWN_STATE,VIBRATION_MODE_NORMAL,0);
	}
	else
	{
		getXYZValues();
		if (!ble_operation_in_progress) //Don't accept new stuff while previous has not been sent & processed yet
		{
			if (armIsStationary()) 
			{
				if (armIsNeutral())
				{ //check if controller is neutral position
					orientation = 0;
				} 
				else if (orientation != 1 && armIsUp()) 
				{   // forearm up
					//sendToggle(); //pseudoCode
					orientation = 1;
					if (verbose_vibrations) 
					{
						VIBRATION_WITH_STATE_CHANGE(short_vibe_ticks,RECEIVER_CONTROL,VIBRATION_MODE_NORMAL,30);
					}	
				} 
				else if (orientation != 1 && (armRotation() == -1)) 
				{   // ccw
					if (orientation != 2) 
					{
						orientation = 2;
						if ((three_command_mode || (device_commands == 3)) && !primary_continuous) 
						{
							//sendPrevious(); //pseudoCode
						} 
						else 
						{
							//sendDecrease(); //pseudoCode
						}					
						if (verbose_vibrations) 
						{
							VIBRATION_WITH_STATE_CHANGE(short_vibe_ticks,RECEIVER_CONTROL,VIBRATION_MODE_NORMAL,30);
						}	
					}
				} 
				else if (orientation != 1 && (armRotation() == 1)) 
				{   // cw
					if (orientation != 3) 
					{
						orientation = 3;
						if ((three_command_mode || (device_commands == 3)) && !primary_continuous) 
						{
							//sendNext();
						} 
						else 
						{
							//sendIncrease();
						}

						if (verbose_vibrations) 
						{
							VIBRATION_WITH_STATE_CHANGE(short_vibe_ticks,RECEIVER_CONTROL,VIBRATION_MODE_NORMAL,30);
						}					
					}
				}
				else if (orientation != 4 && armIsDown()) 
				{   // forearm down, The End
					orientation = 4;
					VIBRATION_WITH_STATE_CHANGE(looong_vibe_ticks,SHUTDOWN_STATE,VIBRATION_MODE_NORMAL,0);
				}
			}
			if (gyro_enabled && orientation == 0) // neutral orientation prerequisite for accepting swipes
			{ 
				getGyroValues();
				if (swipeLeft()) 
				{
					//sendNext(); //pseudoCode
					if (verbose_vibrations) 
					{
						VIBRATION_WITH_STATE_CHANGE(short_vibe_ticks,RECEIVER_CONTROL,VIBRATION_MODE_NORMAL,70);
					}
					else
						DELAY_WITH_STATE_CHANGE(70,RECEIVER_CONTROL);
					//compute_delay(700); // Long delay to prevent activation on backstroke
				} 
				else if (swipeRight()) 
				{
					//sendPrevious(); //pseudoCode
					if (verbose_vibrations) 
					{
						VIBRATION_WITH_STATE_CHANGE(short_vibe_ticks,RECEIVER_CONTROL,VIBRATION_MODE_NORMAL,70);
					}
					else
						DELAY_WITH_STATE_CHANGE(70,RECEIVER_CONTROL);
						//compute_delay(700); // Long delay to prevent activation on backstroke
				}
			}
		}
	}
}

/////////////////////////
///MATH SECTION BEGINS///
/////////////////////////

int armIsStationary() {
	if (dg < dg_limit && fabs(g - 1) < g_limit) {
		stationary_counter++;
		if (stationary_counter > stationary_limit) {
			return 1;
		} else {
			return 0;
		}
	} else {
		stationary_counter = 0;
		return 0;
	}
}

int armIsNeutral() {
	return (phi < phi_neutral_cw && phi > phi_neutral_ccw
			&& theta < theta_neutral_high && theta > theta_neutral_low);
}

int armIsUp() {
	return (theta > theta_high);
}

int armIsDown() {
	return (theta < theta_low);
}

int armRotation() {
	if (fabs(yaw) > yaw_limit || theta > theta_limit_high
			|| theta < theta_limit_low || fabs(phi) > 80) {
		return 0;
	} else if (phi > phi_cw) {
		return 1;
	} else if (phi < phi_ccw) {
		return -1;
	} else {
		return 0;
	}
}

int swipeRight() {
	return (yaw > yaw_activate && fabs(fabs(acc.y[n]) - 1) < 0.2);
}

int swipeLeft() {
	return (yaw < -yaw_activate && fabs(fabs(acc.y[n]) - 1) < 0.2);
}

//delta x from last value
double calcDx() {
	dxv = acc.x[n] - acc.x[n - 1];
	return dxv;
}

//delta y from last value
double calcDy() {
	dyv = acc.y[n] - acc.y[n - 1];
	return dyv;
}

//delta z from last value
double calcDz() {
	dzv = acc.z[n] - acc.z[n - 1];
	return dzv;
}

double calcG() {
	gv = sqrt(pow(acc.x[n], 2.0) + pow(acc.y[n], 2.0) + pow(acc.z[n], 2.0));
	return gv;
}

double calcDg() {
	dgv = sqrt(pow(dx, 2.0) + pow(dy, 2.0) + pow(dz, 2.0));
	return dgv;
}

double calcTheta() {
	thetav = asin(acc.x[n] / g) * 180 / M_PI;
	return thetav;
}

double calcPhi() {
	phiv = atan(
			acc.z[n] / sqrt(0.001 * pow(acc.x[n], 2.0) + pow(acc.y[n], 2.0)))
			* 180 / M_PI;
	if (acc.y[n] > 0) {
		return -phiv;
	} else {
		return phiv;
	}
}

double calcYaw() 
{
	// This could be improved to take  orientation into account
	// Now assumes perfect semipronation
	if (acc.y[n] > 0) {
		yawv = gyro.y[n];
	} else {
		yawv = -gyro.y[n];
	}
	return yawv;
}
