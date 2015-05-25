#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#include <stdint.h>
#include "LSM9DS0.h"
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <gps.h>
#include <libgpsmm.h>

using namespace std;

int file;
void writeMagReg(uint8_t reg, uint8_t value);
void readBlock(uint8_t command, uint8_t size, uint8_t *data);
void readMAG(int  *m);
void writeAccelReg(uint8_t reg, uint8_t value);
void readAccel(int *a);
void writeGyroReg(uint8_t reg, uint8_t value);
void readGyro(int *g);


#define DT 0.02
#define AA 0.97

#define A_GAIN 0.0573
#define G_GAIN 0.070
#define RAD_TO_DEG 57.29578


int main(int argc, char *argv[])
{
    /**GPS Stuff**/
    /*for(;;)
    {
        gpsmm gps("localhost", DEFAULT_GPSD_PORT);

        if(gps.stream(WATCH_ENABLE|WATCH_NMEA) == NULL)
        {
            cout<<"No GPS running. Retry in 5 seconds" << endl;
            continue;
        }

        const char* buffer = NULL;

        for(;;)
        {
            struct gps_data_t* newdata;

            if(!gps.waiting(5000000))
            continue;

            if((newdata = gps.read()) == NULL)
            {
                cerr<< "Read error.\n";
                break;
            }
            else
            {
                buffer = gps.data();

                cout<< "Space"<< endl;
                cout<< buffer << endl;
            }

        }

    }*/

    /**Other Module Stuff**/

    char filename[20];
    int magRaw[3];
    int accelRaw[3];
    int gyroRaw[3];

    float rate_gyro_y = 0.0;
    float rate_gyro_x = 0.0;
    float rate_gyro_z = 0.0;

    float gyroXAngle = 0.0;
    float gyroYAngle = 0.0;
    float gyroZAngle = 0.0;

    float AccXAngle = 0.0;
    float AccYAngle = 0.0;

    float CFAngleX = 0.0;
    float CFAngleY = 0.0;

    sprintf(filename, "/dev/i2c-1", 1);
    file = open(filename, O_RDWR);
    if(file<0)
    {
        printf("Unable to open I2C bus");
        exit(1);
    }

    if(ioctl(file, I2C_SLAVE, GYRO_ADDRESS)< 0)
    {
        printf("Error: Could not select gyro\n");
    }

    if(ioctl(file, I2C_SLAVE, ACC_ADDRESS)< 0)
    {
        printf("Error: Could not select accelerometer\n");
    }

    if(ioctl(file, I2C_SLAVE, MAG_ADDRESS)< 0)
    {
        printf("Error: Could not select magnetometer\n");
    }

        	//Enable the gyroscope
	writeGyroReg(CTRL_REG1_G, 0x0F); //Normal power mode, all axes enabled
    writeGyroReg(CTRL_REG4_G, 0b00110000); //Continuous updates, 2000dps full scale

            	//Enable the accelerometer
	writeAccelReg(CTRL_REG1_XM, 0x67);   //x,y,z enabled continuous updates 100Hz data rate
	writeAccelReg(CTRL_REG2_XM, 0b00100000);   //+/- 16G scale
	//writeAccelReg(CTRL_REG5_XM, 0b11110000);

	    	//Enable the magnetometer
	writeMagReg(CTRL_REG5_XM, 0b11110000);   // Temp enable, M data rate = 50Hz
	writeMagReg(CTRL_REG6_XM, 0b01100000);   // +/-12gauss
	writeMagReg(CTRL_REG7_XM, 0b00000000);   // Continuous-conversion mode

    while(1)
	{
		readGyro(gyroRaw);
        readAccel(accelRaw);
        readMAG(magRaw);
		//printf("gyroRaw X %i    \tgyroRaw Y %i \tgyroRaw Z %i \n", gyroRaw[0],gyroRaw[1],gyroRaw[2]);

        //convert gyro raw readings to degrees per second
		rate_gyro_x = (float) gyroRaw[0] * G_GAIN;
		rate_gyro_y = (float) gyroRaw[1] * G_GAIN;
		rate_gyro_z = (float) gyroRaw[2] * G_GAIN;

        //calculate angle from the gyro
		gyroXAngle += rate_gyro_x*DT;
		gyroYAngle += rate_gyro_y*DT;
		gyroZAngle += rate_gyro_z*DT;

		AccXAngle = (float) (atan2(accelRaw[1], accelRaw[2])+M_PI)*RAD_TO_DEG;
		AccYAngle = (float) (atan2(accelRaw[2], accelRaw[0])+M_PI)*RAD_TO_DEG;

		AccXAngle -= (float)180.0;
		if(AccYAngle > 90)
            AccYAngle -= (float)270;
        else
            AccYAngle += (float)90;

            	//Complementary filter used to combine the accelerometer and gyro values.
        CFAngleX=AA*(CFAngleX+rate_gyro_x*DT) +(1 - AA) * AccXAngle;
        CFAngleY=AA*(CFAngleY+rate_gyro_y*DT) +(1 - AA) * AccYAngle;

		//Only needed if the heading value does not increase when the magnetometer is rotated clockwise
		//magRaw[1] = -magRaw[1];

		//Compute heading
        float heading = 180 * atan2(magRaw[1],magRaw[0])/M_PI;

		//Convert heading to 0 - 360
        if(heading < 0)
        heading += 360;

		//printf("heading %7.3f \t ", heading);

		//printf("gyro x: %f   \tgyro y: %f   \tgyro z: %f \n", gyroXAngle, gyroYAngle, gyroZAngle);;
		//printf("accel x: %f   \taccel y: %f \n", AccXAngle, AccYAngle);
		//printf("magRaw X %i    \tmagRaw Y %i \tMagRaw Z %i \n", magRaw[0],magRaw[1],magRaw[2]);
        printf ("   GyroX  %7.3f \t AccXangle \e[m %7.3f \t \033[22;31mCFangleX %7.3f\033[0m\t GyroY  %7.3f \t AccYangle %7.3f \t \033[22;36mCFangleY %7.3f\t\033[0m\n",gyroXAngle,AccXAngle,CFAngleX,gyroYAngle,AccYAngle,CFAngleY);

		//Sleep for 0.25ms
		//usleep(25000);
		delay(500);

	}

}

void writeMagReg(uint8_t reg, uint8_t value)
{
  int result = i2c_smbus_write_byte_data(file, reg, value);
    if (result == -1)
    {
        printf ("Failed to write byte to I2C Mag.");
        exit(1);
    }
}

void  readBlock(uint8_t command, uint8_t size, uint8_t *data)
{
    int result = i2c_smbus_read_i2c_block_data(file, command, size, data);
    if (result != size)
    {
       printf("Failed to read block from I2C.");
        exit(1);
    }
}

void readMAG(int  *m)
{
        uint8_t block[6];

        readBlock(0x80 | OUT_X_L_M, sizeof(block), block);

        *m = (int16_t)(block[0] | block[1] << 8);
        *(m+1) = (int16_t)(block[2] | block[3] << 8);
        *(m+2) = (int16_t)(block[4] | block[5] << 8);

}

void writeAccelReg(uint8_t reg, uint8_t value)
{
  int result = i2c_smbus_write_byte_data(file, reg, value);
    if (result == -1)
    {
        printf ("Failed to write byte to I2C Accel.");
        exit(1);
    }
}

void readAccel(int  *a)
{
        uint8_t block[6];

        readBlock(0x80 | OUT_X_L_A, sizeof(block), block);

        *a = (int16_t)(block[0] | block[1] << 8);
        *(a+1) = (int16_t)(block[2] | block[3] << 8);
        *(a+2) = (int16_t)(block[4] | block[5] << 8);

}

void writeGyroReg(uint8_t reg, uint8_t value)
{
  int result = i2c_smbus_write_byte_data(file, reg, value);
    if (result == -1)
    {
        printf ("Failed to write byte to I2C Gyro.");
        exit(1);
    }
}

void readGyro(int  *g)
{
        uint8_t block[6];

        readBlock(0x80 | OUT_X_L_G, sizeof(block), block);

        *g = (int16_t)(block[0] | block[1] << 8);
        *(g+1) = (int16_t)(block[2] | block[3] << 8);
        *(g+2) = (int16_t)(block[4] | block[5] << 8);

}

