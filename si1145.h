// Dependent on wiringPi
// gcc -Wall -lwiringPi -o si1145 si1145.c

// ALL COMMAND SI1145
// Default I2C RPI address in (0x39) = FLOAT ADDR (Slave) Other [(0x49) = VCC ADDR / (0x29) = GROUND ADDR]
#define SI1145_MODE_AUTO		(1)
#define SI1145_ADDR_FLOAT 		(0x60)    
#define SI1145_CONTROL_POWERON		(0x03)
#define SI1145_CONTROL_POWEROFF		(0x00)
#define SI1145_COMMAND_BIT 		(0x80)   //Must be 1
#define SI1145_REG_PARTID		(0x00)
#define SI1145_REG_HWKEY  		(0x07)
#define SI1145_REG_MEASRATE0 		(0x08)
#define SI1145_REG_MEASRATE1  		(0x09)
#define SI1145_REG_IRQEN  		(0x04)
#define SI1145_REG_IRQEN_ALSEVERYSAMPLE (0x01)
#define SI1145_REG_IRQEN_PS1EVERYSAMPLE (0x04)
#define SI1145_REG_IRQEN_PS2EVERYSAMPLE (0x08)
#define SI1145_REG_IRQEN_PS3EVERYSAMPLE (0x10)
#define SI1145_REG_IRQMODE1 		(0x05)
#define SI1145_REG_IRQMODE2 		(0x06)
#define SI1145_REG_INTCFG  		(0x03)
#define SI1145_REG_IRQSTAT  		(0x21)
#define SI1145_RESET    		(0x01)
#define SI1145_REG_COMMAND 		(0x18)
#define SI1145_REG_UCOEFF0  		(0x13)
#define SI1145_REG_UCOEFF1 		(0x14)
#define SI1145_REG_UCOEFF2 		(0x15)
#define SI1145_REG_UCOEFF3 		(0x16)
#define SI1145_REG_INTCFG_INTOE		(0x01)
#define SI1145_PARAM_CHLIST 		(0x01)
#define SI1145_PARAM_CHLIST_ENUV 	(0x80)
#define SI1145_PARAM_CHLIST_ENAUX 	(0x40)
#define SI1145_PARAM_CHLIST_ENALSIR 	(0x20)
#define SI1145_PARAM_CHLIST_ENALSVIS 	(0x10)
#define SI1145_PARAM_CHLIST_ENPS1 	(0x01)
#define SI1145_PARAM_CHLIST_ENPS2 	(0x02)
#define SI1145_PARAM_CHLIST_ENPS3 	(0x04)
#define SI1145_REG_PARAMRD 		(0x2E)	// PARAM_RD 0x2E
#define SI1145_REG_PARAMWR  		(0x17)  // PARAM_WR 0x17
#define SI1145_PARAM_SET 		(0xA0)
#define SI1145_PSALS_AUTO 		(0x0F)
#define SI1145_ALS_FORCE		(0x06)


// Outline functions used
uint8_t setUpDevice(uint16_t fd);
void powerOffDevice(uint16_t fd);
void resetDevice(uint16_t fd);
uint8_t writeParam(uint16_t fd, uint8_t p, uint8_t val);
uint16_t readUV(uint16_t fd); 
uint16_t readIR(uint16_t fd);
uint16_t readVisibleIR(uint16_t fd); 
