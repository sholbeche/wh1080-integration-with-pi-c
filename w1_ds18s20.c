// gcc -Wall -o w1m w1_multi.c
// sudo modprobe w1-gpio
// sudo modprobe w1-therm

#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#define CALIBLOW	0
#define CALIBHIGH	75000

typedef struct {
	char *tag;
	int calLow;
	int calHigh;
} Devices;

int * readTemp(Devices *sensors);

int main (void) {

	int * r;
	int i;

	Devices sensors[] = {{"28-0000062a818f",223,223},{"28-0000062b7873",35,35},
				{"28-0000062bf83e",152,152},{"28-0000062c60ab",-90,-90},
				{"28-0000062cad64",-27,-27}};
//	Devices sensors[] = {{"28-0000062a818f",87,-1803},{"28-0000062b7873",238,-1415},
//				{"28-0000062bf83e",-38,-1340},{"28-0000062c60ab",-300,2547},
//				{"28-0000062cad64",12,2011}};

	r = readTemp(sensors);

	for ( i = 0; i < 5; i++ ) {
		printf("r[%d] = %d\n", i, *(r + i));
	}
	return 0;
}


int * readTemp(Devices *sensors) {
	DIR *dir;
	struct dirent *dirent;
	char buf[256];     // Data from device
	char tmpData[5];   // Temp C * 1000 reported by device 
	const char path[] = "/sys/bus/w1/devices"; 
	ssize_t numRead;
	static int ret[1];
	int i = 0;
	int j = 0;
	int devCnt = 0;
	float tempCTot = 0;
	static int temp[5];

// 1st pass counts devices
	dir = opendir (path);
	if (dir != NULL) {
		while ((dirent = readdir (dir))) {
// 1-wire devices are links beginning with 28-
			if (dirent->d_type == DT_LNK &&
				strstr(dirent->d_name, "28-") != NULL) {
				i++;
			}
		}
		(void) closedir (dir);
	} else {
		perror ("Couldn't open the w1 devices directory");
		return ret;
	}
	devCnt = i;
	i = 0;

// 2nd pass allocates space for data based on device count
	char dev[devCnt][16];
	char devPath[devCnt][128];
	dir = opendir (path);
	if (dir != NULL) {
		while ((dirent = readdir (dir))) {
// 1-wire devices are links beginning with 28-
			if (dirent->d_type == DT_LNK && 
				strstr(dirent->d_name, "28-") != NULL) { 
				strcpy(dev[i], dirent->d_name);
// Assemble path to OneWire device
				sprintf(devPath[i], "%s/%s/w1_slave", path, dev[i]);
				i++;
			}
		}
		(void) closedir (dir);
 	} else {
		perror ("Couldn't open the w1 devices directory");
		return ret;
	}
	i = 0;

// Read temp continuously
// Opening the device's file triggers new reading
	while(1) {
		int fd = open(devPath[i], O_RDONLY);
		if(fd == -1) {
			perror ("Couldn't open the w1 device.");
			return ret;
		}
		while((numRead = read(fd, buf, 256)) > 0) {
			strncpy(tmpData, strstr(buf, "t=") + 2, 5);
			float tempC = strtof(tmpData, NULL);
			tempCTot = tempC /1000 + tempCTot;

			for ( j = 0 ;j < 5; j++ ) {
				if (strcmp(dev[i],sensors[j].tag) == 0)
//					temp[j] = tempC + sensors[j].calLow + (sensors[j].calHigh - sensors[j].calLow) * 
//										(tempC - CALIBLOW) / (CALIBHIGH - CALIBLOW);
					temp[j] = tempC;
			}

			if (strcmp(dev[i], "28-0000062a818f") == 0) {
				tempC = tempC + 0;//55.0;
			}else if(strcmp(dev[i], "28-0000062b7873") == 0) {
				tempC = tempC + 0;//55.0;
			}else if(strcmp(dev[i], "28-0000062c60ab") == 0) {
				tempC = tempC - 0;//70.0;
			}else if(strcmp(dev[i], "28-0000062bf83e") == 0) {
				tempC = tempC - 0;//70.0;
			}else if(strcmp(dev[i], "28-0000062cad64") == 0) {
				tempC = tempC + 0;//55.0;
			}
			printf("Device: %s - ", dev[i]);
			printf("Temp: %.3f C  ", tempC / 1000);
			printf("%.3f F\n", (tempC / 1000) * 9 / 5 + 32);
		}
		close(fd);
		i++;
		if(i == devCnt) {
			i = 0;
			printf("Av Temp: %.2f C\n",tempCTot/5.0);
            		printf("%s\n", ""); // Blank line after each cycle
			tempCTot = 0;
			return temp;
		}
	}
	return ret;
}
