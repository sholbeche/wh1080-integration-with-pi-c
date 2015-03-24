#define LOGGING_INTERVAL	(1)	// Record data every x periods of 48 seconds
#define DATADIR 			"/home/pi/weather/data/raw/"
#define DATABASE 			"/home/pi/weather/data/datalogger.db"

#define USE_BMP085
#define ALTITUDE_M      	170.0f

#define SPI_SELECT		"/dev/spidev0.1"		// 0.0 for CE0 and 0.1 for CE1
#define RFM_GPIO		27				// GPIO port used for WH1080 data input

#define USE_TSL2561			// Measure broadband and IR to derive Lux
#define USE_SI1145			// Measure visible and IR to derive UV
