/*===========================================================================
*
* MAGNETIC SENSOR DRIVER
* Copyright (c) 2016, "Memsic Inc."
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
* 3. Neither the name of "Memsic Inc." nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

============================================================================*/
/*===========================================================================

REVISON HISTORY FOR FILE
This section contains comments describing changes made to the module.
Notice that changes are listed in reverse chronological order.

when              who       what, where, why
----------      ----      ----------------------------------------------------
12.25            zhangyx     V1.0 Create
=============================================================================*/

#include <memsic_wrapper.h>
#include <MemsicConfig.h>
#include <MemsicOri.h>
#include <MemsicGyr.h>
#include <MemsicRov.h>
#include <MemsicCustomSpec.h>
#include <MemsicNineAxisFusion.h>
#include <platform.h>

static int has_Gyro=0;

float raw_acc[3]={0.0, 0.0, 9.8};
float raw_mag[3]={0.0, 0.0, 0.0};


int SetMagpara(float *calmagpara)
{
	float iniCalPara[4] = {0,0,0,50};
	iniCalPara[0]=calmagpara[0];
	iniCalPara[1]=calmagpara[1];
	iniCalPara[2]=calmagpara[2];
	iniCalPara[3]=calmagpara[3];

	if (has_Gyro) {//9d lib init lib
		IniPara iniPara;
		//float iniCalPara[4] = {0,0,0,0};
	

		if((myabs(iniCalPara[0]) < 1e-6)&&(myabs(iniCalPara[1]) < 1e-6)&&(myabs(iniCalPara[2])< 1e-6)) {
		    iniPara.iniAccuracy = 0;	

		 } else {

		    iniPara.iniAccuracy = 3;
		 }

      
		iniPara.iniCalPara = iniCalPara;
		iniPara.t = ts0;
		iniPara.si = (float*)si;
		iniPara.magVar = magVar;
		iniPara.enableGyroCal = enableGyroCal;
		iniPara.outlierGyroRestart = outlierGyroRestart;
		iniPara.iniGyroBias = iniGyroBias;
		iniPara.gyroGrade = gyroGrade;
		iniPara.extraSampleCount = extraSampleCount;
		InitializeAlgo(iniPara);  
	} else {//6d lib init lib
	 AlgoInitial(iniCalPara); 	  
    }
	return 0;

}


int MEMSIC_InitLib(int hwGyroSupport)
{
	
	
	has_Gyro=hwGyroSupport;


#if 0
	float iniCalPara[4] = {0,0,0,50};
	has_Gyro=hwGyroSupport;
	if (has_Gyro) {//9d lib init lib
		IniPara iniPara;
		//float iniCalPara[4] = {0,0,0,0};

		iniPara.iniCalPara = iniCalPara;
		iniPara.iniAccuracy = 0;
		iniPara.t = ts0;
		iniPara.si = (float*)si;
		iniPara.magVar = magVar;
		iniPara.enableGyroCal = enableGyroCal;
		iniPara.outlierGyroRestart = outlierGyroRestart;
		iniPara.iniGyroBias = iniGyroBias;
		iniPara.gyroGrade = gyroGrade;
		iniPara.extraSampleCount = extraSampleCount;
		InitializeAlgo(iniPara);  
	} else {//6d lib init lib
	 AlgoInitial(iniCalPara); 	  
    }
#endif
	return 0;
}

int MEMSIC_FusionSetGyroData(float data_x, float data_y, float data_z, int64_t time_stamp)
{

	return 1;
}

int MEMSIC_FusionSetAccData(float data_x, float data_y, float data_z, int64_t time_stamp)
{
    raw_acc[0]=data_x;
    raw_acc[1]=data_y;
    raw_acc[2]=data_z;
    return 1;
}

int MEMSIC_FusionSetMagData(float data_x, float data_y, float data_z, int64_t time_stamp)
{
	raw_mag[0]=data_x;
	raw_mag[1]=data_y;
	raw_mag[2]=data_z;

    //osLog(LOG_ERROR, "%s: x:%f, y:%f, z:%f\n", __func__, (double)raw_mag[0], (double)raw_mag[1], (double)raw_mag[2]);
	return 1;
}

int MEMSIC_GetGravity(float *outData, int16_t *accuracy)
{
    float gravity_vec[3]={0.0,0.0,1.0};
    if (has_Gyro) {
		GetGravity(gravity_vec);
        outData[0]=gravity_vec[0]; 
        outData[1]=gravity_vec[1]; 
        outData[2]=gravity_vec[2]; 
        *accuracy=GetAccuracy();
    } else {
        GetMemsicGravityAcc(gravity_vec);
        outData[0]=gravity_vec[0]; 
        outData[1]=gravity_vec[1]; 
        outData[2]=gravity_vec[2]; 
        *accuracy=GetMagAccuracy();
    }
    return 1;
}

int MEMSIC_GetRotaionVector(float *outData, int16_t *accuracy)
{	
    float cali_rov_vec[4]={0};
    if (has_Gyro) {
        GetRotVec(cali_rov_vec);
        outData[0]=cali_rov_vec[0];
        outData[1]=cali_rov_vec[1];
        outData[2]=cali_rov_vec[2];
        outData[3]=cali_rov_vec[3];
        *accuracy=GetAccuracy();
    } else {
		GetCalRov(cali_rov_vec);
		outData[0]=cali_rov_vec[0];
		outData[1]=cali_rov_vec[1];
		outData[2]=cali_rov_vec[2];
		outData[3]=cali_rov_vec[3];
		*accuracy=GetMagAccuracy();
    }
    return 1;
}
int MEMSIC_GetOrientaion(float *outData, int16_t *accuracy)
{
    float ypr[3]={0};
    if (has_Gyro) { 
        GetOri(ypr);
        outData[0]=ypr[0];
        outData[1]=ypr[1];
        outData[2]=ypr[2];
       // osLog(LOG_ERROR, "%s: x:%f, y:%f, z:%f\n", __func__, (double)outData[0], (double)outData[1], (double)outData[2]);
        *accuracy=GetAccuracy();
    } else {
        GetCalOri(ypr);
        outData[0]=ypr[0];
        outData[1]=ypr[1];
        outData[2]=ypr[2];
        *accuracy=GetMagAccuracy();
    }
    return 1;
}

int MEMSIC_GetLinearaccel(float *outData, int16_t *accuracy) 
{
    float linear_acc_vec[3]={0.0,0.0,0.0};
    if (has_Gyro) {
        GetLinearAcc(linear_acc_vec);
        outData[0]=linear_acc_vec[0]; 
        outData[1]=linear_acc_vec[1]; 
        outData[2]=linear_acc_vec[2]; 
        *accuracy=GetAccuracy();
    } else {
        GetMemsicLinearAcc(linear_acc_vec);
        outData[0]=(float)linear_acc_vec[0]; 
        outData[1]=(float)linear_acc_vec[1]; 
        outData[2]=(float)linear_acc_vec[2]; 
        *accuracy=GetMagAccuracy();
    }
    return 1;
}

int MEMSIC_GetGameRotaionVector(float *outData, int16_t *accuracy)
{
    float Game_rov_vec[4]={0};
	if (has_Gyro) {
			GetGameRotVec(Game_rov_vec);
			outData[0]=Game_rov_vec[0];
			outData[1]=Game_rov_vec[1];
			outData[2]=Game_rov_vec[2];
			outData[3]=Game_rov_vec[3];
			*accuracy=GetAccuracy();

		} else {
			GameRotVec(Game_rov_vec);
			outData[0]=Game_rov_vec[0];
			outData[1]=Game_rov_vec[1];
			outData[2]=Game_rov_vec[2];
			outData[3]=Game_rov_vec[3];
			*accuracy=GetMagAccuracy();

		}

    return 1;
}

int MEMSIC_GetGeoMagnetic(float *outData, int16_t *accuracy)
{
    static float cali_mag_vec[3]={0};
    if (has_Gyro) { //get 9d cal mag
        GetCalibratedMag(cali_mag_vec);
        outData[0]=cali_mag_vec[0]; //the unit is ut
        outData[1]=cali_mag_vec[1];
        outData[2]=cali_mag_vec[2];
        *accuracy=GetAccuracy();
    } else {
        GetCalMag(cali_mag_vec);
        outData[0]=cali_mag_vec[0];
        outData[1]=cali_mag_vec[1];
        outData[2]=cali_mag_vec[2];
        *accuracy=GetMagAccuracy();
    }
    return 1;
}

int MEMSIC_GetVirtualGyro(float *outData, int16_t *accuracy)
{
    float cali_gyr_vec[3]={0};
    GetCalGyr(cali_gyr_vec);
    outData[0]=cali_gyr_vec[0];
    outData[1]=cali_gyr_vec[1];
    outData[2]=cali_gyr_vec[2];
    *accuracy=GetMagAccuracy();
    return 1;
}





