#include "serial.h"
#include "wit_c_sdk.h"
#include "REG.h"
#include <stdint.h>

//Headers for the Fusion Algorithm
#include "Fusion.h"
#include <stdbool.h>
#include <stdio.h>


#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80


static int fd, s_iCurBaud = 9600;
static volatile char s_cDataUpdate = 0;

#define SAMPLE_PERIOD (0.5f) //Sample Period is for the IMU
#define SAMPLE_RATE (2)

const int c_uiBaud[] = {2400 , 4800 , 9600 };//, 19200 , 38400 , 57600 , 115200 , 230400 , 460800 , 921600};


static void AutoScanSensor(char* dev);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static void calibrateSensors();

//Additional Structures & Methods for Calculating Position
typedef struct {
	float x;
	float y;
	float z;
} Vector3f;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngle;

//integrate(prev_var, current_var, SAMPLE_PERIOD)
Vector3f integrate(Vector3f previous, Vector3f current, float dt) {
    Vector3f area = {0.0, 0.0, 0.0};
    area.x = (previous.x + current.x) * dt * 0.5;
    area.y = (previous.y + current.y) * dt * 0.5;
    area.z = (previous.z + current.z) * dt * 0.5;
    return area;
}

Vector3f displacement = {0.0, 0.0, 0.0}; //Declare Fixed Origin
EulerAngle orientation = {0.0, 0.0, 0.0}; 

bool status = false;

void update_displacement(Vector3f new_displacement) {
    printf("New Displacement: (%f, %f, %f)\n", new_displacement.x, new_displacement.y, new_displacement.z);
}

void update_orientation(EulerAngle new_orientation) {
    printf("New Orientation: (%f, %f, %f)\n", new_orientation.roll, new_orientation.pitch, new_orientation.yaw);
}

// Getter functions for angle and position in Python
EulerAngle get_orientation() {
    return orientation;
}

Vector3f get_displacement() {
    return displacement;
}

bool get_status() {
	return status;
}

float iteration = 0;//iterations

//MAIN METHOD
int main(int argc,char* argv[]){
	
	//Initialise Reference velocity & position variables
    Vector3f velocity = {0.0, 0.0, 0.0};
    Vector3f prev_acceleration = {0.0, 0.0, 0.0};
    Vector3f prev_velocity = {0.0, 0.0, 0.0};
    Vector3f prev_displacement = {0.0, 0.0, 0.0};
    
	//if(argc < 2)
	//{
	//	printf("please input dev name\n");
	//	return 0;
	//}


    //if((fd = serial_open(argv[1] , 9600)<0))
	// {
	//     printf("open %s fail\n", argv[1]);
	//     return 0;
	// }
	//else printf("open %s success\n", argv[1]);


	float fAcc[3], fGyro[3], fAngle[3];
	int i , ret;
	char cBuff[1];
	
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitRegisterCallBack(SensorDataUpdata);
	
	printf("\r\n********************** wit-motion Normal example  ************************\r\n");
	AutoScanSensor(argv[1]);
	
    //Initialise Fusion Algorithm
    // Define calibration (replace with actual calibration data if available)
    const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

    // Initialise Fusion algorithms
    FusionOffset offset;
    FusionAhrs ahrs;

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,
            .gain = 0.5f,
            .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
            .accelerationRejection = 10.0f,
            .magneticRejection = 10.0f,
            .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);
	
	while(1)
	{
	     
	    while(serial_read_data(fd, cBuff, 1))
		  {
		      WitSerialDataIn(cBuff[0]);
		  }
		  printf("\n");
          Delayms(500);
          
          iteration++;
		  //printf("iteration: %.3f\n", iteration);
        
          if(s_cDataUpdate & ACC_UPDATE)
		   {
			   FusionVector gyroscope;
			   FusionVector accelerometer;
			   FusionVector magnetometer;
			   for(i = 0; i < 3; i++)
			    {
				    fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				    fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				    fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;		    
			    }
			

  			  if(s_cDataUpdate & ACC_UPDATE)
			    {
				   //printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
				   accelerometer.axis.x = fAcc[0];
				   accelerometer.axis.y = fAcc[1];
				   accelerometer.axis.z = fAcc[2];
				   s_cDataUpdate &= ~ACC_UPDATE;
		   	    }
			  if(s_cDataUpdate & GYRO_UPDATE)
			    {
				   //printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
				   gyroscope.axis.x = fGyro[0];
        		   gyroscope.axis.y = fGyro[1];
        		   gyroscope.axis.z = fGyro[2];
				   s_cDataUpdate &= ~GYRO_UPDATE;
			    }
			  if(s_cDataUpdate & ANGLE_UPDATE)
			    {
				   //printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
				   s_cDataUpdate &= ~ANGLE_UPDATE;
			    }
			  if(s_cDataUpdate & MAG_UPDATE)
			    {
				   //printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
				   s_cDataUpdate &= ~MAG_UPDATE;
				   magnetometer.axis.x = fGyro[0];
        		   magnetometer.axis.y = fGyro[1];
        		   magnetometer.axis.z = fGyro[2];
			    }
			    
			    //Apply Calibration
			    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
                accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
                magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);
                 
                // Update gyroscope offset correction algorithm
        		gyroscope = FusionOffsetUpdate(&offset, gyroscope);
        		
                //Update AHRS algorithm
			    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
		    
		    	const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
		    	const FusionVector linear = FusionAhrsGetEarthAcceleration(&ahrs);
		    	//printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n",euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
		    	
		    	//printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.3f, Y %0.3f, Z %0.3f\n", 
		    	//euler.angle.roll, euler.angle.pitch, euler.angle.yaw, linear.axis.x, linear.axis.y, linear.axis.z);
		    	//Update Orientation
		    	
		    	if(iteration>6)
		    	{
		    		orientation.roll = euler.angle.roll;
					orientation.pitch = euler.angle.pitch;
					orientation.yaw = euler.angle.yaw;
					update_orientation(orientation);
		    	}			
            	
            	//Double Integration to Calculate Position
				Vector3f acceleration = {linear.axis.x, linear.axis.y, linear.axis.z};
				
				//Convert Acceleration to m/s^2
				acceleration.x *= 9.81;
				acceleration.y *= 9.81;
				acceleration.z *= 9.81;
        		
        		
        		
        		// Update Displacement
        		if(iteration>6)
		    	{
        			// Update velocity
        			velocity = integrate(prev_acceleration, acceleration, SAMPLE_PERIOD);
        		
        			displacement = integrate(prev_velocity, velocity, SAMPLE_PERIOD);
        			displacement.x /= 5;
					displacement.y /= 5;
					displacement.z /= 5;
					
					prev_velocity = velocity;
            		prev_displacement = displacement;
            		prev_acceleration = acceleration;
            		update_displacement(displacement);
        		}	
		    }    
     }
    
    serial_close(fd);
	return 0;
}


static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}


static void Delayms(uint16_t ucMs)
{ 
     usleep(ucMs*1000);
}
 
	
static void AutoScanSensor(char* dev)
{
	int i, iRetry;
	char cBuff[1];
	
	for(i = 1; i < 3; i++)
	{
		serial_close(fd);
		s_iCurBaud = c_uiBaud[i];
		fd = serial_open("/dev/ttyS0" , c_uiBaud[i]);
		
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			Delayms(200);
			while(serial_read_data(fd, cBuff, 1))
			{
				WitSerialDataIn(cBuff[0]);
			}
			if(s_cDataUpdate != 0)
			{
				printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}

