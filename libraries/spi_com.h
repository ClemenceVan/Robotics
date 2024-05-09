#ifndef SPI_COM_H
#define SPI_COM_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{

	signed Set_Speed_M1 :16;
	signed Set_Speed_M2 :16;
	signed Act_Speed_M1 :16;
	signed Act_Speed_M2 :16;
	signed Encoder_M1 :32;
	signed Encoder_M2 :32;                                                                            
} MotorDataType;


void Send_Read_Motor_Data(MotorDataType *);
#ifdef _WIN32
	void Send_Read_Motor_Data(MotorDataType *MotorData) {
		MotorData->Set_Speed_M1 = 0;
		MotorData->Set_Speed_M2 = 0;
		MotorData->Act_Speed_M1 = 0;
		MotorData->Act_Speed_M2 = 0;
		MotorData->Encoder_M1 = 0;
		MotorData->Encoder_M2 = 0;
	}
#endif

#endif

#ifdef __cplusplus
}
#endif