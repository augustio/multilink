/*IR TX functions for Ruskis pilot. Should not be very timing sensitive, but needs to be tested.
*/

//#define IRLED_PIN 5 // For DK
#define IRLED_PIN 0 // For Elercv

#define GEWA_HALF_PERIOD 10 //Period probably 23.8us (42kHz). Had to reduce by 2 because there were mysterious additional delays
#define JVC_ON_PERIOD 8 //8on 18 off
#define JVC_OFF_PERIOD 18 //JVC carrier duty cycle should be ~0.25...0.33
#define JVC_REPEAT 1 //Repeat commands enable. Seems required
#define SAMSUNG_ON_PERIOD 6 //8 on 18 off (NEC protocol states 1/4 or 1/3 duty cycle). REAL REMOTE meas: 8.7 & 17.5 us
#define SAMSUNG_OFF_PERIOD 15 //8on & 18off produced actual 10.2 on & 21.0 off. 6 & 15 are 8.1 & 18.2
#define FINLUX_ON_PERIOD 7 //Measured 9.4 & 18.2
#define FINLUX_OFF_PERIOD 15 //8on & 18off produced actual 10.1 on & 21.0 off. 7 & 15 are 9.15 & 18
#define SONY_ON_PERIOD 6 //Measured 8.78 & 16.0. 8on & 16off produced actual 10.05 on & 19.10 off
#define SONY_OFF_PERIOD 13

/*GEWA patterns
 *Starts at MSB. 0b1 == pulsing on, 0b0 == pulsing off. Fill rest with zeros.
 */
#define GEWA_ENDINGS 0b1001101010101000 //This seems to be same for all the codes tested
#define GEWA_CMD_1_1 0b1001010101011000 //1_1 to 1_8 work. 1_9 maybe. Rest not tested
#define GEWA_CMD_1_2 0b1010100101010100
#define GEWA_CMD_1_3 0b1001010110011000
#define GEWA_CMD_1_4 0b1010101001010100
#define GEWA_CMD_1_5 0b1001101001010100
#define GEWA_CMD_1_6 0b1010010101010100
#define GEWA_CMD_1_8 0b1010011001010100
#define GEWA_CMD_1_9 0b1001100101101000
#define GEWA_CMD_X_E 0b1001011010101000
#define GEWA_CMD_4_1 0b1010010101101000
#define GEWA_CMD_4_2 0b1001100101101000 //Possibly duplicate with 1-9
#define GEWA_CMD_4_3 0b1010100101101000
#define GEWA_CMD_4_4 0b1001011001101000

//JVC patterns. All work
#define JVC_CMD_3_4  0b1111010101100000
#define JVC_CMD_3_6  0b1111010110100000
#define JVC_CMD_3_A  0b1100010111111000
#define JVC_CMD_3_B  0b1100010101111000
#define JVC_CMD_3_C  0b1100010111101000

//Samsung patterns. All work
#define SAMSUNG_CMD_5_A 0b11100000111000001101000000101111
#define SAMSUNG_CMD_5_B 0b11100000111000001110000000011111
#define SAMSUNG_CMD_5_C 0b11100000111000000100000010111111
#define SAMSUNG_CMD_P_UP	0b11100000111000000100100010110111 //aka "5_6"
#define SAMSUNG_CMD_P_DOWN	0b11100000111000000000100011110111 //aka "5_4"


//Finlux patterns with message length(see IRLED_cntrl.c for details). First 4 work
#define FINLUX_CMD_7_A  	0b00100000000111100001000000010101 //10101 == 20 bits + stop
#define FINLUX_CMD_7_B		0b00001000000111100000000000010101
#define FINLUX_CMD_7_C  	0b00100000000110010010000000010101
#define FINLUX_CMD_P_UP		0b00100000000100100000000000010111 //10101 == 22 bits + stop
#define FINLUX_CMD_P_DOWN	0b00100000010010000001000000010101 //Broken

//Sony patterns. All work
#define SONY_CMD_ON_OFF		0b1010100100000000
#define SONY_CMD_VOL_UP		0b0100100100000000
#define SONY_CMD_VOL_DOWN	0b1100100100000000
#define SONY_CMD_P_UP		0b0000100100000000
#define SONY_CMD_P_DOWN		0b1000100100000000

uint32_t WCn_valo();
uint32_t GEWA_IR_cntrl(uint16_t IR_pattern);
uint32_t GEWA_pulsate(uint8_t active);
uint32_t Samsung_control(uint32_t cmd);
uint32_t Samsung_pulsate(uint32_t n_cycles);
uint32_t JVC_control(uint16_t cmd);
uint32_t JVC_pulsate(uint32_t n_cycles);
uint32_t Finlux_control(uint32_t cmd);
uint32_t Finlux_pulsate(uint32_t n_cycles);
uint32_t IR_pulsate(uint32_t n_cycles, uint32_t on_time, uint32_t off_time);
uint32_t Sony_control(uint16_t cmd);
