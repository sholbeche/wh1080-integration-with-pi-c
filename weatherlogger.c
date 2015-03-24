#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <time.h>
#include <sched.h>
#include <sqlite3.h>

#include "bcm2835.h"
#include "rfm01.h"
#include "tsl2561.h"
#include "si1145.h"
#include "weatherlogger.h"

static int fd = -1;
static const char *device = SPI_SELECT;
static uint8_t mode=0;
static uint8_t bits = 8;
static uint32_t speed = 1000000;
static uint16_t delay=0;
static inited = 0;
int g_low_threshold = 1000;

char *direction_name[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};



uint16_t cmd_reset	= CMD_RESET;
uint16_t cmd_status 	= CMD_STATUS;

uint16_t cmd_drate 	= CMD_DRATE|0xaa;	// drate is c8xx rather than c6xx
uint16_t cmd_freq	= CMD_FREQ|0x620; // 433.92 MHz

#ifdef RFM01
        uint16_t cmd_afc        = CMD_AFC|AFC_ON|AFC_OUT_ON|AFC_MANUAL|AFC_FINE$
        uint16_t cmd_dcycle 	= CMD_LOWDUTY|0x00;
        uint16_t cmd_fifo       = CMD_FIFO|0x00;

        uint16_t cmd_config     = CMD_CONFIG|BAND_433|LOAD_CAP_12C0|BW_67;
        uint16_t cmd_rcon 	= (CMD_RCON|RX_EN|VDI_DRSSI|LNA_0|RSSI_91);
        uint16_t cmd_dfilter 	= (CMD_DFILTER|CR_LOCK_FAST|FILTER_OOK);
#endif

#ifdef RFM12B
        uint16_t cmd_config     = 0x8017;
        uint16_t cmd_power      = 0x8281; // RFM01 doesn't support this
        uint16_t cmd_sync       = 0xce55;
        uint16_t cmd_afc        = 0xc407; // or C400 for no AFC. C6xx on RFM01
        uint16_t cmd_dcycle 	= 0xc800;
        uint16_t cmd_pll        = 0xcc1f;
        uint16_t cmd_fifo       = 0xca8a; // CExx rather than CAxx on RFM01

        uint16_t cmd_dfilter = 0xc260;
        uint16_t cmd_rcon = (CMD_RCON|P16|VDI_MEDIUM|LNA_MEDIUM|RSSI_97|BW_340);
#endif


void uninit(void);
void scheduler_realtime();
void scheduler_standard();
void calculate_values(unsigned char *buf, float temp, float pressure, float pressureAlt);
uint8_t _crc8( uint8_t *addr, uint8_t len);

sqlite3* db = NULL;

static void pabort(const char *s)
{
    if(inited)
    {
        uninit();
    }
	perror(s);
	abort();
}

static uint16_t send_command16(int fd, uint16_t cmd)
{
	uint8_t tx[2];
	uint8_t *buf = (uint8_t *)&cmd;
	tx[0] = buf[1];
	tx[1] = buf[0];

	//printf("SPI %02x%02x\n", buf[1], buf[0]);

	uint8_t rx[2] = {0, 0};
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = 2,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	if(ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1)
		pabort("can't send spi message");
	
	return (((uint16_t)rx[0]) << 8) + rx[1];
}

void sendReset(void)
{
    send_command16(fd, cmd_fifo); // in case reset sensitivity is low
    send_command16(fd, cmd_reset);

}

void setupChip(void) {
	send_command16(fd, cmd_status);
	send_command16(fd, cmd_config);
	send_command16(fd, cmd_freq);
	send_command16(fd, cmd_drate);
	send_command16(fd, cmd_rcon);
	send_command16(fd, cmd_dfilter);
	send_command16(fd, cmd_fifo);
	send_command16(fd, cmd_afc);
	send_command16(fd, cmd_dcycle);
	#ifdef RFM12B
		send_command16(fd, cmd_power);
		send_command16(fd, cmd_sync);
		send_command16(fd, cmd_pll);
	#endif	
}

static int callback(void *NotUsed, int argc, char **argv, char **azColName){
   int i;
   for(i=0; i<argc; i++){
      printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
   }
   printf("\n");
   return 0;
}

void init(void) {
	char *sql;
	char *zErrMsg = 0;

// Create sqlite database and tables if they don't exist
	if(sqlite3_open(DATABASE, &db) != 0) {
		printf("Failed to open Database\n");
		exit(-1);
	}else{
		sql = "CREATE TABLE raw_data("  \
		"timestamp	NUMERIC	NOT NULL," \
		"in_temp	REAL," \
		"out_temp	REAL," \
		"in_humidity	INT," \
		"out_humidity	INT," \
		"pressureAlt	REAL," \
		"wind_avg_kmh	REAL," \
		"wind_gust_kmh	REAL," \
		"wind_direction	INT," \
		"rainfall	REAL," \
		"bytes		NUMERIC," \
		"light_si1145	INT," \
		"ir_si1145	INT," \
		"light_tsl2561	INT," \
		"ir_tsl2561	INT);";
		if(sqlite3_exec(db, sql, callback, 0, &zErrMsg) != SQLITE_OK ){
			fprintf(stderr, "SQL error: %s\n", zErrMsg);
			sqlite3_free(zErrMsg);
		}else{
			fprintf(stdout, "Table created\n");
		}
		sql = "CREATE TABLE derived_data("  \
		"timestamp	NUMERIC	NOT NULL," \
		"pressure0	REAL," \
		"dew_point	REAL," \
		"uv_si1145	REAL," \
		"lux_tsl2561	INT);";
		if(sqlite3_exec(db, sql, callback, 0, &zErrMsg) != SQLITE_OK ){
			fprintf(stderr, "SQL error: %s\n", zErrMsg);
			sqlite3_free(zErrMsg);
		}
	}

    if(map_peripheral(&gpio) == -1) 
    {
	  printf("Failed to map the GPIO registers into the virtual memory space.\n");
	  exit(-1);
    }
    
    if(map_peripheral(&timer_arm) == -1)
    {
        printf("Failed to map the registers into the virtual memory space.\n");
        unmap_peripheral(&gpio);
        exit(-1);
    }

    
    inited = 1;

	// 0xF90200; // run at 1MHz
	TIMER_ARM_CONTROL = TIMER_ARM_C_DISABLE|TIMER_ARM_C_FREE_EN
							|TIMER_ARM_C_16BIT|TIMER_ARM_C_PS1
							|TIMER_ARM_C_FPS(0xf9);
	
	// Init GPIO21 (on pin 13) as input (DATA), GPIO22 (pin 15) as output (nRES)
	*(gpio.addr + 2) = (*(gpio.addr + 2) & 0xfffffe07)|(0x001 << 6);

        #ifdef RFM01
                printf("Initialising RFM01\n");
        #endif
        #ifdef RFM12B
                printf("Initialising RFM12b\n");
        #endif

	
	fd = open(device, O_RDWR);
	if (fd < 0)
	{
	    pabort("Failed to open device");
	}

	// SPI mode
	if(ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1)
	{
	    pabort("Can't set SPI mode");
	}

	// Bits per word (driver only supports 8 -bits I think, but RFM12B handles this ok)
	if(ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1)
	{
		pabort("Can't set bits per word");
	}

	// SPI clock speed (Hz)
	if(ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1)
	{
		pabort("Can't set SPI clock speed");
	}

        printf("SPI: mode %d, %d-bit, %d KHz\n", mode, bits, speed/1000);
	
}

void uninit(void)
{
    if(inited)
    {
        unmap_peripheral(&gpio);
        unmap_peripheral(&timer_arm);
        if(fd != -1)
        {
            close(fd);
            fd = -1;
        }
        sqlite3_close(db);
        inited = 0;
    }
}

void strobe_afc(int fd) {

	send_command16(fd, cmd_afc|AFC_STROBE); // Strobe high
	send_command16(fd, cmd_afc & (~AFC_ON)); // Strobe low, disable AFC processing
	uint16_t status = send_command16(fd, cmd_status);
	// get offs bits and extend two's complement to a byte
	int8_t offset = (status & STATUS_OFFS) | (status & STATUS_OFFSIGN ? 0xe0 : 0);
	
	float freq_offs = (float)offset;
	#ifdef RFM12B
		freq_offs *= 2.5;
	#endif
	send_command16(fd, cmd_afc); // Strobe low, re-enable AFC

	printf("Frequency deviation %0.1fKHz (%d)\n", freq_offs, (int)offset);

	send_command16(fd, cmd_rcon);
}


void main_loop(void)
{
    uint16_t status;
    uint8_t device_id = 0xa1;

    uint8_t rssi = 0, oldrssi = 0;
    unsigned int shorts = 0;
    unsigned int rssitime, oldrssitime, now;
    int count = 0, timeout = 1;
    time_t last_valid = time(0);
    int crc_passed;

    unsigned int rssitime_buf[500];
    unsigned char bytes[11];

    oldrssitime = TIMER_ARM_COUNT;

    // Switch to realtime scheduler
    scheduler_realtime();
    do
    {
       // Read the GPIO pin for clocked DATA value
        status = ((*(gpio.addr + 13)) >> RFM_GPIO) & 1;
        rssi = status;
        rssitime = TIMER_ARM_COUNT;
        // Check if the pin transitioned
        if(rssi != oldrssi)
        {
            // If falling edge (1 -> 0), then store bit pulse duration
            if(rssi == 0)
            {
                rssitime_buf[count] = rssitime - oldrssitime;
                if(++count == 500)
                    count = 499;
            }
            oldrssi = rssi;
            oldrssitime = rssitime;
            timeout = 0;
        }
        // Check time since last transition. If timeout, then dump packet.
        int packet_offset = 0;
        now = TIMER_ARM_COUNT;
        if(!timeout && (now - oldrssitime) > 5000)
        { // && count > 0

            if(count > 60)
            { // then maybe something at least interesting
                // Look for device_id
                int idx, matchcount = 0;
                uint8_t bit;
                for(idx=0; idx < count; idx++)
                {
                    bit = rssitime_buf[idx] < g_low_threshold ? 1 : 0;
                    //if( bit == (device_id & (0x80 >> matchcount)) >> (7-matchcount) )
                    //{
                        matchcount++;
                    //}
                    //else
                    //{
                     //   matchcount = 0;
                   // }
                    if(matchcount == 8)
                    {
                        packet_offset = idx - 7;
                        break;
                    }
                }
                printf("\rData bits = %d   (offset %d) (%d short)\n", count, packet_offset, shorts);
                if(count == 88 && matchcount == 8)
                { // then probably a data packet

                    strobe_afc(fd); // lock frequency to good signal

                    int b;
                    uint8_t byte;
                    for(idx=0; idx < 11; idx++)
                    {
                        byte = 0;
                        for(b=0; b < 8; b++)
                        {
                            // Short pulses 1, long pulses 0
                            uint8_t bit = rssitime_buf[packet_offset + (idx * 8 + b)] < g_low_threshold ? 1 : 0;
                            byte = (byte << 1) + bit;
                        }
                        bytes[idx] = byte;
                        printf("%02x ", byte);
                    }
                    crc_passed = bytes[10] == _crc8(bytes+1, 9);
                    printf("crc %s (gap %ds)\n", crc_passed ? "ok" : "fail", (int)(time(0) - last_valid));
                    last_valid = time(0);
                    fflush(stdout);
                }
            }
            else
            {
                if(shorts++ % 10 == 0)
                {
                    printf(".");
                    fflush(stdout);
                }
            }
            timeout = 1;

            // If we get enough bits, then dump stats to indicate pulse lengths coming from the device.
            if(count > 40)
            {
                // These are slightly confusing - lo used to mean low side of threshold, but printf below reports them as binary
                // 0 and 1. So the meanings are opposite - to be fixed.
                unsigned int idx, min_lo=999999, min_hi=999999, max_lo = 0, max_hi = 0;
                unsigned int val;
                for(idx = 0; idx < count; idx++)
                {
                    // printf("RSSI 1 -> 0  %3d: %4dus ( %s )\n", idx, rssitime_buf[idx],
                    // rssitime_buf[idx] >= LOW_THRESHOLD ? "Hi" : "Lo");
                    val = rssitime_buf[idx];

                    // Short pulses are binary '1', long pulses are binary '0'
                    if(val < g_low_threshold)
                    {
                        if(val < min_lo)
                            min_lo = val;
                        if(val > max_lo)
                            max_lo = val;
                    }
                    else
                    {
                        if(val < min_hi)
                            min_hi = val;
                        if(val > max_hi)
                            max_hi = val;
                    }
                }

                printf("Pulse stats: Hi: %u - %u   Lo: %u - %u  (%d point)\n", min_lo, max_lo, min_hi, max_hi, count);

// Recalculate the pulse threshold if we got a perfect read.
		if(count == 88 && crc_passed) {
//g_low_threshold = ( ((max_lo + min_lo) / 2) + ((max_hi + min_hi) / 2)) / 2;
			g_low_threshold =(max_lo + min_hi) / 2;
			printf("Threshold now %d\n", g_low_threshold);

// Note the time of the last reading...
			unsigned int wait_start = TIMER_ARM_COUNT, elapsed;

// at this point, we can do other stuff that requires the RT schedule
			float temp = 21.0f;
			float pressure = 1000.0f;
			float pressureAlt = 1000.0f;
			float altitude = ALTITUDE_M;
			#ifdef USE_BMP085
                    		read_bmp085(&altitude, &temp, &pressure, &pressureAlt); // read pressure, calculate for the given altitude
			#endif

			calculate_values(bytes+1, temp, pressure, pressureAlt);

// Wait for remainder of 47 seconds in standard scheduler until we can expect the next read
			scheduler_standard();

			do {
				elapsed = (TIMER_ARM_COUNT - wait_start) / 1000000;
				printf("Wait %us \r", (48*LOGGING_INTERVAL) - elapsed);
				fflush(stdout);
				usleep(250000);
			}while(elapsed < (48*LOGGING_INTERVAL)- 5);

			printf("Listening for transmission\n");
			scheduler_realtime();
		}
            }
            count = 0;

        }
        usleep(5); // No point running with nanosecond loops when pulses are in the hundreds of microseconds...

    }
    while(1); // Ctrl+C to exit for now...
}

float sample_rssi(int fd, int duration, int interval) {

	unsigned int start_time, now;
	unsigned int loop_count = 0, rssi_total = 0;

	start_time = TIMER_ARM_COUNT;

	do {
		uint16_t status = send_command16(fd, cmd_status);
		int rssi = (status & STATUS_RSSI) ? 1 : 0;
		loop_count++;
		rssi_total+=rssi;
		now = TIMER_ARM_COUNT;
		usleep(interval);	// microseconds
	} while(now - start_time < (duration * 1000));	// duration as microseconds

	float duty = ((float)rssi_total/loop_count) * 100;

	return duty;
}


int main(int argc, char** argv) {
	init();
//	sendReset();
	usleep(100000);
	setupChip();
	usleep(5000);	// Allow crystal oscillator to start

	uint16_t bw_scale[6] = {BW_67, BW_134, BW_200, BW_270, BW_340, BW_400};

	struct RSSI rssi_scale[24] = {
		L0R73,L0R79,L0R85,L0R91,L0R97,L0R103,
		L6R73,L6R79,L6R85,L6R91,L6R97,L6R103,
		L14R73,L14R79,L14R85,L14R91,L14R97,L14R103,
		L20R73,L20R79,L20R85,L20R91,L20R97,L20R103,
	};

	int idx1, idx2;
	float sumidx = 0.0;
	for(idx1=0; idx1 < 24; idx1++) 
	{

		uint16_t cmd_rcon_mod = (cmd_rcon & ~(RSSI_X2|LNA_XX)) | (rssi_scale[idx1].rssi_setth |rssi_scale[idx1].g_lna);

		printf("%15s idx %-2d  ", rssi_scale[idx1].name, idx1);

		for(idx2=0; idx2 < 6; idx2++) {

			uint16_t cmd_config_mod = (cmd_config & ~BW_X2) | bw_scale[idx2];

			send_command16(fd, cmd_config_mod);
			send_command16(fd, cmd_rcon_mod);
			usleep(1000);

			rssi_scale[idx1].duty[idx2] = sample_rssi(fd, 50, 100);
			sumidx = sumidx + rssi_scale[idx1].duty[idx2];
			if(cmd_rcon_mod == cmd_rcon && cmd_config_mod == cmd_config)
				printf("%6.2f< ", rssi_scale[idx1].duty[idx2]);
			else
				printf("%6.2f  ", rssi_scale[idx1].duty[idx2]);
			fflush(stdout);
		}
		printf("\n");
	}
	send_command16(fd, cmd_config);
	send_command16(fd, cmd_rcon);
	usleep(1000);

	do {
		float duty = sample_rssi(fd, 250, 100);
		printf("RSSI Duty %0.2f\r", duty);
		fflush(stdout);
		usleep(250000);
	} while(argc > 1);

if(sumidx<0.01) exit(-1);
	printf("starting main loop\n");
	main_loop();
    
	uninit();    
	return 0; 
}




FILE* create_or_open(void)
{
    time_t now;
    struct tm * timeinfo;
    now = time (NULL);
    time ( &now);
    timeinfo = localtime ( &now);
    
    char buffer[100];
    int written = sprintf(buffer, "mkdir -p %s", DATADIR);
    
    strftime(buffer+written, 100-written, "%Y/%Y-%m", timeinfo);
    system(buffer);
    
    written = sprintf(buffer, "%s", DATADIR);
    
    strftime(buffer+written, 100-written, "%Y/%Y-%m/%Y-%m-%d.txt", timeinfo);
    
    FILE* file = fopen(buffer, "a+");
    if(file != NULL)
    {
    
        strftime(buffer, 100, "%Y-%m-%d %H:%M:%S,", timeinfo);
        fprintf(file, "%s", buffer);
    }
    return file;
}
void calculate_dew_point(float temp, float humidity, float* dp)
{
  float a =  17.27;
  float b = 237.7;
  float gamma = ((a * temp)/(b+temp)) + log(humidity/100.0);
  if(a == gamma)
    *dp = 0;
  else
    *dp= (b*gamma)/(a-gamma);
}

void calculate_values(unsigned char *buf, float temp, float pressure, float pressureAlt) {

    unsigned short device_id = ((unsigned short)buf[0] << 4) | (buf[1] >> 4);
    unsigned short temperature_raw = (((unsigned short)buf[1] & 0x0f) << 8) | buf[2];
    float temperature = ((float)temperature_raw - 400) / 10;
    int humidity = buf[3];

    unsigned short wind_avg_raw = (unsigned short)buf[4];
    float wind_avg_ms = roundf((float)wind_avg_raw * 34.0f) / 100;
    float wind_avg_mph = wind_avg_ms * 2.23693629f;
    float wind_avg_kmh = wind_avg_ms * 3.6f;
    
    unsigned short wind_gust_raw = (unsigned short)buf[5];
    float wind_gust_ms = roundf((float)wind_gust_raw * 34.0f) / 100;
    float wind_gust_mph = wind_gust_ms * 2.23693629f;
    float wind_gust_kmh = wind_gust_ms * 3.6f;

    unsigned short rain_raw = (((unsigned short)buf[6] & 0x0f) << 8) | buf[7];

    float rain = (float)rain_raw * 0.3f;

    int direction = buf[8] & 0x0f;

    char *direction_str = direction_name[direction];

    float dp = 0;
    calculate_dew_point(temperature, (float) humidity, &dp);	

	#ifdef USE_TSL2561  	// If Lux detector is available gather measurements
		int rc;
		uint16_t broadband, ir;
		uint32_t lux=0;
// prepare the sensor
// (the first parameter is the raspberry pi i2c master controller attached to the TSL2561, the second is the i2c selection jumper)
// The i2c selection address can be one of: TSL2561_ADDR_LOW, TSL2561_ADDR_FLOAT or TSL2561_ADDR_HIGH
		TSL2561 light1 = TSL2561_INIT(1, TSL2561_ADDR_FLOAT);
	
// initialize the sensor
		rc = TSL2561_OPEN(&light1);
		if(rc != 0) {
			fprintf(stderr, "Error initializing TSL2561 sensor (%s). Check your i2c bus (es. i2cdetect)\n", strerror(light1.lasterr));
// you don't need to TSL2561_CLOSE() if TSL2561_OPEN() failed, but it's safe doing it.
			TSL2561_CLOSE(&light1);
		}else{

// set the gain to 1X (it can be TSL2561_GAIN_1X or TSL2561_GAIN_16X)
// use 16X gain to get more precision in dark ambients, or enable auto gain below
			rc = TSL2561_SETGAIN(&light1, TSL2561_GAIN_1X);

// set the integration time 
// (TSL2561_INTEGRATIONTIME_402MS or TSL2561_INTEGRATIONTIME_101MS or TSL2561_INTEGRATIONTIME_13MS)
// TSL2561_INTEGRATIONTIME_402MS is slower but more precise, TSL2561_INTEGRATIONTIME_13MS is very fast but not so precise
			rc = TSL2561_SETINTEGRATIONTIME(&light1, TSL2561_INTEGRATIONTIME_101MS);

// sense the luminosity from the sensor (lux is the luminosity taken in "lux" measure units)
// the last parameter can be 1 to enable library auto gain, or 0 to disable it
			rc = TSL2561_SENSELIGHT(&light1, &broadband, &ir, &lux, 1);
			printf("RC: %i(%s), broadband: %i, ir: %i, lux: %i\n", rc, strerror(light1.lasterr), broadband, ir, lux);
			TSL2561_CLOSE(&light1);
		}
	#endif

	#ifdef USE_SI1145	// If UV detector is available gather measurements
		uint16_t fd = 0;
		fd = wiringPiI2CSetup(SI1145_ADDR_FLOAT);
		uint16_t irSI1145, visSI1145;
		float uvSI1145;
		if(!setUpDevice(fd)) {
			visSI1145 = readVisibleIR(fd);
			uvSI1145 = (float) readUV(fd) / 100.0;
			irSI1145 = readIR(fd);
			if((visSI1145 > 0) && (uvSI1145 != 655.36) && (irSI1145 > 0)) {
				printf("Light: %d, ir: %d, uv: %3.2f\n", visSI1145, irSI1145, uvSI1145);
			}
			powerOffDevice(fd);
		}
	#endif

    int x = 0;
    char buffer_string[128] = "";
    char* bfrptr = buffer_string;
    for(x = 0; x < 9; x++)
    {
        snprintf(bfrptr, 10, "0x%02x,", buf[x]);
        bfrptr += 5;
    }
    
    time_t rawtime;
    time ( &rawtime );
    
    struct tm* timeInfo;
    char timebuffer[128] = "";
    timeInfo = localtime(&rawtime);

    strftime(timebuffer, 128, "%Y-%m-%d %H:%M:%S", timeInfo);	

    printf("\nTime is: %s\n", timebuffer );
    
    printf("Buffer string is %s\n", buffer_string);

    printf("Station Id: %04X\n", device_id);
    printf("Temperature: %0.1fC, Humidity: %d%%\n", temperature, humidity);
    printf("Wind speed: %0.1f mph, Gust Speed %0.1f mph, %s\n", wind_avg_mph, wind_gust_mph, direction_str);
    printf("Total rain: %0.1f mm\n", rain);
	printf("SI1145_Light: %d\n",visSI1145);
	printf("SI1145_IR: %d\n",irSI1145);
	printf("SI1145_UV: %3.2f\n",uvSI1145);
	printf("TSL2561_Light: %i\n", broadband);
	printf("TSL2561_IR: %i\n", ir);
	printf("TSL2561_Lux: %i\n", lux);


    FILE* file = create_or_open();
    if(file != NULL)
    {
        // delay in mins, hum in, temp_in, hum_out, temp_out, abs_pressure, wind_ave, wind_gust, wind_dir, rain, status
        fprintf(file,"1,50,%0.1f,%d,%0.1f,%.1f,%0.1f,%0.1f,%d,%0.1f\n",
              temp, humidity, temperature, pressure, wind_avg_kmh, wind_gust_kmh, direction, rain);
        fclose(file);
    }

    char* sql = sqlite3_mprintf("INSERT INTO raw_data(timestamp,in_temp,out_temp,in_humidity,out_humidity,pressureAlt,wind_avg_kmh,wind_gust_kmh,wind_direction,rainfall,bytes,light_si1145,ir_si1145,light_tsl2561,ir_tsl2561) values (%Q, %f,%f,%d,%d,%f,%f,%f,%d,%0.1f,%Q,%d,%d,%d,%d)",
                                 timebuffer, temp, temperature,50,humidity,pressureAlt,wind_avg_kmh,wind_gust_kmh,direction,rain,buffer_string,visSI1145,irSI1145,broadband,ir);

   char* zerrMsg = NULL;
   int  db_write = sqlite3_exec(db, sql, NULL, 0, &zerrMsg);
   sqlite3_free(sql);
   if(db_write != SQLITE_OK)
   {
       printf("SQL error: %s\n", zerrMsg);
       sqlite3_free(zerrMsg);
   }

   sql = sqlite3_mprintf("INSERT INTO derived_data(timestamp,pressure0,dew_point,uv_si1145,lux_tsl2561) values(%Q, %f, %f, %f, %d)", timebuffer, pressure, dp, uvSI1145, lux);
   zerrMsg = NULL;
   db_write = sqlite3_exec(db, sql, NULL, 0, &zerrMsg);
   sqlite3_free(sql);
   if(db_write != SQLITE_OK)
   {
       printf("SQL error: %s\n", zerrMsg);
       sqlite3_free(zerrMsg);
   }

    
    
}

/*
* Function taken from Luc Small (http://lucsmall.com), itself
* derived from the OneWire Arduino library. Modifications to
* the polynomial according to Fine Offset's CRC8 calulations.
*/
uint8_t _crc8( uint8_t *addr, uint8_t len)
{
    uint8_t crc = 0;

    // Indicated changes are from reference CRC-8 function in OneWire library
    while (len--)
    {
        uint8_t inbyte = *addr++;
        uint8_t i;
        for (i = 8; i; i--)
        {
            uint8_t mix = (crc ^ inbyte) & 0x80; // changed from & 0x01
            crc <<= 1; // changed from right shift
            if (mix)
                crc ^= 0x31;// changed from 0x8C;
            inbyte <<= 1; // changed from right shift
        }
    }
    return crc;
}

void scheduler_realtime() {

    struct sched_param p;

    p.__sched_priority = sched_get_priority_max(SCHED_RR);

    if( sched_setscheduler( 0, SCHED_RR, &p ) == -1 )
    {
        perror("Failed to switch to realtime scheduler.");
    }
}

void scheduler_standard() {

    struct sched_param p;

    p.__sched_priority = 0;

    if( sched_setscheduler( 0, SCHED_OTHER, &p ) == -1 )
    {
        perror("Failed to switch to normal scheduler.");
    }
}
