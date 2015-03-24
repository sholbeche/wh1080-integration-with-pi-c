// Dependent on wiringPi
// gcc -Wall -lwiringPi -o si1145 si1145.c

#include <stdio.h>
#include <inttypes.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <errno.h>
#include "si1145.h"

int main_si1145(){
	uint16_t fd = 0;
	fd = wiringPiI2CSetup(SI1145_ADDR_FLOAT);
	uint16_t ir, vis;
	float uv;
	while(1){
		if(!setUpDevice(fd)) {
			vis=readVisibleIR(fd);
			uv=(float)readUV(fd)/100.0;
			ir=readIR(fd);
			system ("tput clear");	// clear screen
			if ((vis>0) && (uv!=655.36) && (ir>0)) {
			printf("Light: %d\n", vis);
			printf("UV: %3.2f\n", uv);
			printf("IR: %d\n",ir);
			delay (500);
			}
			powerOffDevice(fd);
		}
	}
	return 0;
}


uint8_t setUpDevice(uint16_t fd){
	wiringPiI2CWriteReg8(fd, SI1145_COMMAND_BIT, SI1145_CONTROL_POWERON); //enable the device
// Wait for power on to complete
	delay(100);
// Reads visible + IR diode from the I2C device auto
	uint16_t id = wiringPiI2CReadReg16(fd,SI1145_REG_PARTID);
	if(id!=0x45) return 1;
	resetDevice(fd);

// enable UVindex measurement coefficients!
	wiringPiI2CWriteReg8(fd,SI1145_REG_UCOEFF0, 0x29);  // As per datasheet for UV readings
 	wiringPiI2CWriteReg8(fd,SI1145_REG_UCOEFF1, 0x89);
 	wiringPiI2CWriteReg8(fd,SI1145_REG_UCOEFF2, 0x02);
 	wiringPiI2CWriteReg8(fd,SI1145_REG_UCOEFF3, 0x00);

// enable UV, IR, Visible sensor readings
 	writeParam(fd, SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV | 
  					SI1145_PARAM_CHLIST_ENALSIR | 
					SI1145_PARAM_CHLIST_ENALSVIS );

	if(SI1145_MODE_AUTO) {
// enable interrupt on every sample
		wiringPiI2CWriteReg8(fd,SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);  
		wiringPiI2CWriteReg8(fd,SI1145_REG_IRQEN, SI1145_REG_IRQEN_ALSEVERYSAMPLE); 
// measurement rate for auto
  		wiringPiI2CWriteReg8(fd,SI1145_REG_MEASRATE0, 0xFF); // 255 * 31.25uS = 8ms
// auto run
  		wiringPiI2CWriteReg8(fd,SI1145_REG_COMMAND, SI1145_PSALS_AUTO);
	} else {
// measurement rate for FORCED operation
		wiringPiI2CWriteReg8(fd,SI1145_REG_MEASRATE0, 0x00);
		wiringPiI2CWriteReg8(fd,SI1145_REG_MEASRATE1, 0x00);
// enter FORCED mode and take reading
		wiringPiI2CWriteReg8(fd,SI1145_REG_COMMAND, SI1145_ALS_FORCE);
	}

	return 0;
}


void powerOffDevice(uint16_t fd) {
	wiringPiI2CWriteReg8(fd, SI1145_COMMAND_BIT, SI1145_CONTROL_POWEROFF); //disable the device
	return;
}


void resetDevice(uint16_t fd) {
	wiringPiI2CWriteReg8(fd,SI1145_REG_MEASRATE0, 0);
	wiringPiI2CWriteReg8(fd,SI1145_REG_MEASRATE1, 0);
	wiringPiI2CWriteReg8(fd,SI1145_REG_IRQEN, 0);
	wiringPiI2CWriteReg8(fd,SI1145_REG_IRQMODE1, 0);
	wiringPiI2CWriteReg8(fd,SI1145_REG_IRQMODE2, 0);
	wiringPiI2CWriteReg8(fd,SI1145_REG_INTCFG, 0);
	wiringPiI2CWriteReg8(fd,SI1145_REG_IRQSTAT, 0xFF);
	wiringPiI2CWriteReg8(fd,SI1145_REG_COMMAND, SI1145_RESET);
	delay(10);
	wiringPiI2CWriteReg8(fd,SI1145_REG_HWKEY, 0x17); // set for correct SI1145 operation
	delay(10);
}


uint8_t writeParam(uint16_t fd, uint8_t p, uint8_t v) {
	wiringPiI2CWriteReg8(fd,SI1145_REG_PARAMWR, v);
	wiringPiI2CWriteReg8(fd,SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
	return wiringPiI2CReadReg8(fd,SI1145_REG_PARAMRD);
}


// returns visible+IR light levels
uint16_t readVisibleIR(uint16_t fd) {
	return wiringPiI2CReadReg16(fd,0x22);  // ALS_VIS_DATA0, ALS_VIS_DATA1
}


// returns IR light levels
uint16_t readIR(uint16_t fd) {
	return wiringPiI2CReadReg16(fd,0x24);  // ALS_IR_DATA0, ALS_IR_DATA1 
}


// returns the UV index * 100 (divide by 100 to get the index)
uint16_t readUV(uint16_t fd) {
	return wiringPiI2CReadReg16(fd,0x2C);  // AUX_DATA0/UVINDEX0, AUX_DATA1/UVINDEX1
}
