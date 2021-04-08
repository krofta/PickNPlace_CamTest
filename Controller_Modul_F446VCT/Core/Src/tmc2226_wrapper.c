/*
 * tmc2226_drive.c
 *
 *  Created on: Apr 7, 2021
 *      Author: nk
 */




//#include "boards/Board.h"
#include "../tmc/ic/TMC2226/TMC2226.h"
#include "StepDir.h"
#include "tmc2226_wrapper.h"
#include "main.h"



extern UART_HandleTypeDef huart4;
extern TIM_HandleTypeDef htim4;





static int32_t thigh;
//static uint16_t vref; // mV

//extern IOPinTypeDef DummyPin;

// Helper macro - index is always 1 here (channel 1 <-> index 0, channel 2 <-> index 1)
#define TMC2226_CRC(data, length) tmc_CRC8(data, length, 1)





TMC2226TypeDef *motorToIC(uint8_t motor)
{
	//UNUSED(motor);
	return &motorBoard.ch[motor].TMC2226;
}

UART_HandleTypeDef *channelToUART(uint8_t channel)
{
	return motorBoard.ch[channel].TMC2226_UART;
}

// => UART wrapper
// Write [writeLength] bytes from the [data] array.
// If [readLength] is greater than zero, read [readLength] bytes from the
// [data] array.
void tmc2226_readWriteArray(uint8_t channel, uint8_t *data, size_t writeLength, size_t readLength)
{
	HAL_StatusTypeDef status = HAL_OK;
	if(writeLength > 0)
		status = HAL_UART_Transmit(&huart4, data, writeLength, 10000);
	if(readLength > 0)
		status = HAL_UART_Receive(&huart4, data, readLength, 10000);
	if( status == HAL_OK)
		status = HAL_OK;
	//UART_readWrite(channelToUART(channel), data, writeLength, readLength);
}
// <= UART wrapper

// => CRC wrapper
// Return the CRC8 of [length] bytes of data stored in the [data] array.
uint8_t tmc2226_CRC8(uint8_t *data, size_t length)
{
	return TMC2226_CRC(data, length);
}
// <= CRC wrapper

void tmc2226_writeRegister(uint8_t motor, uint8_t address, int32_t value)
{
	tmc2226_writeInt(motorToIC(motor), address, value);

}

void tmc2226_readRegister(uint8_t motor, uint8_t address, int32_t *value)
{
	*value = tmc2226_readInt(motorToIC(motor), address);
}

uint32_t rotate(uint8_t motor, int32_t velocity)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	StepDir_rotate(motor, velocity);

	return TMC_ERROR_NONE;
}

uint32_t right(uint8_t motor, int32_t velocity)
{
	return rotate(motor, velocity);
}

uint32_t left(uint8_t motor, int32_t velocity)
{
	return rotate(motor, -velocity);
}

uint32_t stop(uint8_t motor)
{
	return rotate(motor, 0);
}

uint32_t moveTo(uint8_t motor, int32_t position)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	StepDir_moveTo(motor, position);

	return TMC_ERROR_NONE;
}

uint32_t moveBy(uint8_t motor, int32_t *ticks)
{
	if(motor >= MOTORS)
		return TMC_ERROR_MOTOR;

	// determine actual position and add numbers of ticks to move
	*ticks += StepDir_getActualPosition(motor);

	return moveTo(motor, *ticks);
}

/*
static uint32_t SAP(uint8_t type, uint8_t motor, int32_t value)
{
	return handleParameter(WRITE, motor, type, &value);
}

static uint32_t GAP(uint8_t type, uint8_t motor, int32_t *value)
{
	return handleParameter(READ, motor, type, value);
}
*/

void checkErrors(uint32_t tick)
{
	UNUSED(tick);
	//motorBoard.chRot.errors = 0;
}

void deInit(uint8_t motor)
{
	enableDriver(motor, DRIVER_DISABLE);

}

uint8_t reset(uint8_t motor)
{
	StepDir_init(STEPDIR_PRECISION);
	StepDir_setPins(motor, motorBoard.ch[motor].Pins.STEP, motorBoard.ch[motor].Pins.DIR, motorBoard.ch[motor].Pins.DIAG);

	return tmc2226_reset(&motorBoard.ch[motor].TMC2226);
}

uint8_t restore(uint8_t motor)
{
	return tmc2226_restore(&motorBoard.ch[motor].TMC2226);
}

void enableDriver(uint8_t motor, DriverState state)
{
	if(state == DRIVER_USE_GLOBAL_ENABLE)
		state = motorBoard.driverEnable;

	if(state == DRIVER_DISABLE){
		for(int i = 0; i < STEP_DIR_CHANNELS; i++)
			HAL_GPIO_WritePin(motorBoard.ch[i].Pins.ENN->GPIOx, motorBoard.ch[i].Pins.ENN->GPIO_Pin, GPIO_PIN_RESET);
	}
	else if((state == DRIVER_ENABLE) && (motorBoard.driverEnable == DRIVER_ENABLE)){
		for(int i = 0; i < STEP_DIR_CHANNELS; i++)
			HAL_GPIO_WritePin(motorBoard.ch[i].Pins.ENN->GPIOx, motorBoard.ch[i].Pins.ENN->GPIO_Pin, GPIO_PIN_SET);
	}
}

uint16_t getVREF(uint8_t motor)
{
	return motorBoard.ch[motor].vref;
}

// Set VREF (in mV)
void setVREF(uint8_t motor, uint16_t value)
{
	if (value >= VREF_FULLSCALE)
		return;

	if( motor < 0 || motor > STEP_DIR_CHANNELS)
		return;

	// Store the VREF value for accurate reading
	// Calculating VREF from the timer duty cycle introduces rounding errors
	motorBoard.ch[motor].vref = value;
	//vref = value;
	int val = 0;
	val = motorBoard.ch[motor].vref * TIMER_MAX / VREF_FULLSCALE;
	__HAL_TIM_SET_COMPARE(motorBoard.ch[motor].timer,motorBoard.ch[motor].timer_channel, val);
	HAL_TIM_PWM_Start(motorBoard.ch[motor].timer, motorBoard.ch[motor].timer_channel);

}

void periodicJob(uint8_t motor, uint32_t tick)
{
	tmc2226_periodicJob(&motorBoard.ch[motor].TMC2226, tick);
	StepDir_periodicJob(motor);
}

void TMC2226_init(void)
{
	tmc_fillCRC8Table(0x07, true, 1);
	thigh = 0;

	//   ----ROTATIONSMOTOR
	motorBoard.ch[0].Pins.ENN->GPIOx   		= EN_ROT_GPIO_Port;
	motorBoard.ch[0].Pins.ENN->GPIO_Pin		= EN_ROT_Pin;
	motorBoard.ch[0].Pins.STEP->GPIOx    	= STEP_ROT_GPIO_Port;
	motorBoard.ch[0].Pins.STEP->GPIO_Pin	= STEP_ROT_Pin;
	motorBoard.ch[0].Pins.DIR->GPIOx		= DIR_ROT_GPIO_Port;
	motorBoard.ch[0].Pins.DIR->GPIO_Pin		= DIR_ROT_Pin;
	motorBoard.ch[0].Pins.DIAG->GPIOx    	= DIAG_ROT_GPIO_Port;
	motorBoard.ch[0].Pins.DIAG->GPIO_Pin	= DIAG_ROT_Pin;
	motorBoard.ch[0].Pins.INDEX->GPIOx   	= INDEX_ROT_GPIO_Port;
	motorBoard.ch[0].Pins.INDEX->GPIO_Pin	= INDEX_ROT_Pin;

	motorBoard.ch[0].timer 				  = &htim4;
	motorBoard.ch[0].timer_channel 		  = TIM_CHANNEL_1;

	motorBoard.ch[0].TMC2226_UART  		  = &huart4;
	motorBoard.ch[0].config->reset        = reset;
	motorBoard.ch[0].config->restore      = restore;

	motorBoard.ch[0].rotate               = rotate;
	motorBoard.ch[0].right                = right;
	motorBoard.ch[0].left                 = left;
	motorBoard.ch[0].stop                 = stop;
	/*
	motorBoard.chRot.GAP                  = GAP;
	motorBoard.chRot.SAP                  = SAP;
	*/
	motorBoard.ch[0].moveTo               = moveTo;
	motorBoard.ch[0].moveBy               = moveBy;
	motorBoard.ch[0].writeRegister        = tmc2226_writeRegister;
	motorBoard.ch[0].readRegister         = tmc2226_readRegister;
	//motorBoard.ch[0].userFunction         = userFunction;
	motorBoard.ch[0].enableDriver         = enableDriver;
	motorBoard.ch[0].checkErrors          = checkErrors;
	motorBoard.ch[0].numberOfMotors       = MOTORS;
	motorBoard.ch[0].VMMin                = VM_MIN;
	motorBoard.ch[0].VMMax                = VM_MAX;
	motorBoard.ch[0].deInit               = deInit;
	motorBoard.ch[0].periodicJob          = periodicJob;

	//   ----Z ACHSE
	motorBoard.ch[1].Pins.ENN->GPIOx   	= EN_Z_GPIO_Port;
	motorBoard.ch[1].Pins.ENN->GPIO_Pin	= EN_Z_Pin;
	motorBoard.ch[1].Pins.STEP->GPIOx    = STEP_Z_GPIO_Port;
	motorBoard.ch[1].Pins.STEP->GPIO_Pin	= STEP_Z_Pin;
	motorBoard.ch[1].Pins.DIR->GPIOx		= DIR_Z_GPIO_Port;
	motorBoard.ch[1].Pins.DIR->GPIO_Pin	= DIR_Z_Pin;
	motorBoard.ch[1].Pins.DIAG->GPIOx    = DIAG_Z_GPIO_Port;
	motorBoard.ch[1].Pins.DIAG->GPIO_Pin	= DIAG_Z_Pin;
	motorBoard.ch[1].Pins.INDEX->GPIOx   = INDEX_Z_GPIO_Port;
	motorBoard.ch[1].Pins.INDEX->GPIO_Pin= INDEX_Z_Pin;

	motorBoard.ch[1].timer 				  = &htim4;
	motorBoard.ch[1].timer_channel 		  = TIM_CHANNEL_1;

	motorBoard.ch[1].TMC2226_UART  		  = &huart4;
	motorBoard.ch[1].config->reset        = reset;
	motorBoard.ch[1].config->restore      = restore;

	motorBoard.ch[1].rotate               = rotate;
	motorBoard.ch[1].right                = right;
	motorBoard.ch[1].left                 = left;
	motorBoard.ch[1].stop                 = stop;
	/*
	motorBoard.chRot.GAP                  = GAP;
	motorBoard.chRot.SAP                  = SAP;
	*/
	motorBoard.ch[1].moveTo               = moveTo;
	motorBoard.ch[1].moveBy               = moveBy;
	motorBoard.ch[1].writeRegister        = tmc2226_writeRegister;
	motorBoard.ch[1].readRegister         = tmc2226_readRegister;
	//motorBoard.ch[0].userFunction         = userFunction;
	motorBoard.ch[1].enableDriver         = enableDriver;
	motorBoard.ch[1].checkErrors          = checkErrors;
	motorBoard.ch[1].numberOfMotors       = MOTORS;
	motorBoard.ch[1].VMMin                = VM_MIN;
	motorBoard.ch[1].VMMax                = VM_MAX;
	motorBoard.ch[1].deInit               = deInit;
	motorBoard.ch[1].periodicJob          = periodicJob;

	//   ----X ACHSE
	motorBoard.ch[2].Pins.ENN->GPIOx   	= EN_X_GPIO_Port;
	motorBoard.ch[2].Pins.ENN->GPIO_Pin	= EN_X_Pin;
	motorBoard.ch[2].Pins.STEP->GPIOx    = STEP_X_GPIO_Port;
	motorBoard.ch[2].Pins.STEP->GPIO_Pin	= STEP_X_Pin;
	motorBoard.ch[2].Pins.DIR->GPIOx		= DIR_X_GPIO_Port;
	motorBoard.ch[2].Pins.DIR->GPIO_Pin	= DIR_X_Pin;
	motorBoard.ch[2].Pins.DIAG->GPIOx    = DIAG_X_GPIO_Port;
	motorBoard.ch[2].Pins.DIAG->GPIO_Pin	= DIAG_X_Pin;
	motorBoard.ch[2].Pins.INDEX->GPIOx   = INDEX_X_GPIO_Port;
	motorBoard.ch[2].Pins.INDEX->GPIO_Pin= INDEX_X_Pin;

	motorBoard.ch[2].timer 				  = &htim4;
	motorBoard.ch[2].timer_channel 		  = TIM_CHANNEL_1;

	motorBoard.ch[2].TMC2226_UART  		  = &huart4;
	motorBoard.ch[2].config->reset        = reset;
	motorBoard.ch[2].config->restore      = restore;

	motorBoard.ch[2].rotate               = rotate;
	motorBoard.ch[2].right                = right;
	motorBoard.ch[2].left                 = left;
	motorBoard.ch[2].stop                 = stop;
	/*
	motorBoard.chRot.GAP                  = GAP;
	motorBoard.chRot.SAP                  = SAP;
	*/
	motorBoard.ch[2].moveTo               = moveTo;
	motorBoard.ch[2].moveBy               = moveBy;
	motorBoard.ch[2].writeRegister        = tmc2226_writeRegister;
	motorBoard.ch[2].readRegister         = tmc2226_readRegister;
	//motorBoard.ch[0].userFunction         = userFunction;
	motorBoard.ch[2].enableDriver         = enableDriver;
	motorBoard.ch[2].checkErrors          = checkErrors;
	motorBoard.ch[2].numberOfMotors       = MOTORS;
	motorBoard.ch[2].VMMin                = VM_MIN;
	motorBoard.ch[2].VMMax                = VM_MAX;
	motorBoard.ch[2].deInit               = deInit;
	motorBoard.ch[2].periodicJob          = periodicJob;


	//   ----X ACHSE
	motorBoard.ch[3].Pins.ENN->GPIOx   	= EN_Y_GPIO_Port;
	motorBoard.ch[3].Pins.ENN->GPIO_Pin	= EN_Y_Pin;
	motorBoard.ch[3].Pins.STEP->GPIOx    = STEP_Y_GPIO_Port;
	motorBoard.ch[3].Pins.STEP->GPIO_Pin	= STEP_Y_Pin;
	motorBoard.ch[3].Pins.DIR->GPIOx		= DIR_Y_GPIO_Port;
	motorBoard.ch[3].Pins.DIR->GPIO_Pin	= DIR_Y_Pin;
	motorBoard.ch[3].Pins.DIAG->GPIOx    = DIAG_Y_GPIO_Port;
	motorBoard.ch[3].Pins.DIAG->GPIO_Pin	= DIAG_Y_Pin;
	motorBoard.ch[3].Pins.INDEX->GPIOx   = INDEX_Y_GPIO_Port;
	motorBoard.ch[3].Pins.INDEX->GPIO_Pin= INDEX_Y_Pin;

	motorBoard.ch[3].timer 				  = &htim4;
	motorBoard.ch[3].timer_channel 		  = TIM_CHANNEL_1;

	motorBoard.ch[3].TMC2226_UART  		  = &huart4;
	motorBoard.ch[3].config->reset        = reset;
	motorBoard.ch[3].config->restore      = restore;

	motorBoard.ch[3].rotate               = rotate;
	motorBoard.ch[3].right                = right;
	motorBoard.ch[3].left                 = left;
	motorBoard.ch[3].stop                 = stop;
	/*
	motorBoard.chRot.GAP                  = GAP;
	motorBoard.chRot.SAP                  = SAP;
	*/
	motorBoard.ch[3].moveTo               = moveTo;
	motorBoard.ch[3].moveBy               = moveBy;
	motorBoard.ch[3].writeRegister        = tmc2226_writeRegister;
	motorBoard.ch[3].readRegister         = tmc2226_readRegister;
	//motorBoard.ch[0].userFunction         = userFunction;
	motorBoard.ch[3].enableDriver         = enableDriver;
	motorBoard.ch[3].checkErrors          = checkErrors;
	motorBoard.ch[3].numberOfMotors       = MOTORS;
	motorBoard.ch[3].VMMin                = VM_MIN;
	motorBoard.ch[3].VMMax                = VM_MAX;
	motorBoard.ch[3].deInit               = deInit;
	motorBoard.ch[3].periodicJob          = periodicJob;



	//tmc2226_init(&motorBoard.ch[0].TMC2226, 0, 0, motorBoard.ch[0].config, &tmc2226_defaultRegisterResetState[0]);

/*
	for(int i = 0; i < STEP_DIR_CHANNELS; i++){
		tmc2226_init(&motorBoard.ch[i].TMC2226, i, i, motorBoard.ch[i].config, &tmc2226_defaultRegisterResetState[0]);
		StepDir_setPins(i, motorBoard.ch[i].Pins.STEP, motorBoard.ch[i].Pins.DIR, motorBoard.ch[i].Pins.DIAG);
		StepDir_setVelocityMax(i, 51200);
		StepDir_setAcceleration(i, 51200);
		setVREF(i,500); // mV
		enableDriver(0, DRIVER_ENABLE);
	}

	StepDir_init(STEPDIR_PRECISION);




	//Timer.init();


	enableDriver(0, DRIVER_ENABLE);
	*/
};
