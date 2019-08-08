#include "ui_log.h"

void update_OLED_logging(void);
void update_OLED_error(void);

ui_log_typedef ui_handle;
ui_data_typedef ui_buffer[UI_BUFFER_SIZE] = {};

extern TIM_HandleTypeDef htim2, htim6, htim7;
extern uint8_t led_green;
//extern volatile uint8_t OLED_busy;
uint16_t voltage_tmp=0, current_tmp=0;

void ui_initHandle(ui_log_typedef *handle) {
	handle->status = UI_IDLE;
	handle->u_mode = MANUAL;
	handle->i_mode = MANUAL;
	handle->u_range = U_HIGH;
	handle->i_range = I_HIGH;
	handle->log_rate = LOG_RATE_DEFAULT;
	handle->buf_first = 0;
	handle->buf_last = 0;
	handle->buf = ui_buffer;
	handle->counter = 0;
	//handle->log = 0;
}

//Start data logging
void ui_startLog(ui_log_typedef *handle) {
	if(handle->status == UI_RUN) {
		return;
	}
	else if(handle->status == UI_ERROR) {
		//Make sure current loop closed
		HAL_GPIO_WritePin(I_MEAS_SD_GPIO_Port, I_MEAS_SD_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
	}

	handle->buf_first = 0;
	handle->buf_last = 0;
	handle->counter = 0;
	handle->status = UI_RUN;

	//update_OLED_logging();//Display logging status on OLED

	//HAL_TIM_Base_Start(&htim2);
	//HAL_TIM_Base_Stop_IT(&htim6);//Stop OLED frame
	HAL_TIM_Base_Start_IT(&htim7);
	led_green = 1;
}

//Stop data logging
void ui_stopLog(ui_log_typedef *handle) {
	//HAL_TIM_Base_Stop(&htim2);
	HAL_TIM_Base_Stop_IT(&htim7);
	led_green = 0;
	if(handle->status==UI_ERROR) {
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
	}
	//Some data might not be transmitted to the host, data loss
	handle->status = UI_IDLE;
	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
	//Disable 10ohm shunt, make sure current pass has low resistance
	//HAL_GPIO_WritePin(I_RANGE_SEL_GPIO_Port, I_RANGE_SEL_Pin, GPIO_PIN_SET); //Bypass 10ohm shunt
	ui_setCurrentRange(handle, 0x0);//Auto range, high range first
	ui_setLogRate(handle,10);//Restore log rate, different from log rate stored in handle

	//HAL_TIM_Base_Start_IT(&htim6);//Start OLED frame
}

//Update ADC trigger timer period
uint8_t ui_setLogRate(ui_log_typedef *handle, uint16_t freq) {
	if(handle->status!=UI_IDLE) {
		return 1;
	}
	HAL_TIM_Base_Stop(&htim2);//Stop timer first

	//calculate time base period from freq, timer clock is 1MHz
	uint32_t period = 1000000 / freq;
	htim2.Init.Period = period;
	HAL_TIM_Base_Init(&htim2);

	HAL_TIM_Base_Start(&htim2);//Restart timer
	return 0;
}

//Configure current measurement mode and range
void ui_setCurrentRange(ui_log_typedef *handle, uint8_t range) {
	switch(range) {
	case 0x0:
		handle->i_mode = AUTO;
		handle->i_range = I_HIGH;
		HAL_GPIO_WritePin(I_RANGE_SEL_GPIO_Port, I_RANGE_SEL_Pin, GPIO_PIN_SET); //Bypass 10ohm shunt
		break;
	case 0x1:
		handle->i_mode = MANUAL;
		handle->i_range = I_LOW;
		HAL_GPIO_WritePin(I_RANGE_SEL_GPIO_Port, I_RANGE_SEL_Pin, GPIO_PIN_RESET); //10ohm shunt
		break;
	case 0x2:
		handle->i_mode = MANUAL;
		handle->i_range = I_HIGH;
		HAL_GPIO_WritePin(I_RANGE_SEL_GPIO_Port, I_RANGE_SEL_Pin, GPIO_PIN_SET); //Bypass 10ohm shunt
		break;
	default:
		handle->i_mode = AUTO;
		handle->i_range = I_HIGH;
		HAL_GPIO_WritePin(I_RANGE_SEL_GPIO_Port, I_RANGE_SEL_Pin, GPIO_PIN_SET); //Bypass 10ohm shunt
		break;
	}
}

//Callback function when a sequence of ADC is finished
void callback_ADC_EndOfSequence(ui_log_typedef *handle, uint16_t *data) {
	uint32_t dataU1 = 0, dataI1=0, dataI2=0;
	unsigned char i=0;

	if(handle->status==UI_IDLE) {
		//Not in log mode
		for(i=0; i<ADC_AVERAGE_COUNT; i++) {
			dataU1+=data[ADC_U1_RANK+3*i];
		}
		for(i=0; i<ADC_AVERAGE_COUNT; i++) {
			dataI1+=data[ADC_I1_RANK+3*i];
		}
		for(i=0; i<ADC_AVERAGE_COUNT; i++) {
			dataI2+=data[ADC_I2_RANK+3*i];
		}
		dataU1 >>= 2;
		dataI1 >>= 2;
		dataI2 >>= 2;
		//dataU1 = dataU1*33*31/4096;
		dataU1 >>= 2; //Approximation for faster calculation, error about 0.1 percent
		//dataI1 = ((float)dataI1*3300/4096/64.9*10);
		dataI1 >>= 3; //Approximation for faster calculation, error about 0.7 percent
		//dataI2 = ((float)dataI2*3300/4096/59/10*100);
		dataI2 = (dataI2>>3) + (dataI2>>7) + (dataI2>>8);//Approximation for faster calculation, error about 0.2 percent

		switch(handle->i_range) {
		case I_LOW:
			if(dataI2>=500) {
				handle->i_range = I_HIGH;
				HAL_GPIO_WritePin(I_RANGE_SEL_GPIO_Port, I_RANGE_SEL_Pin, GPIO_PIN_SET); //Bypass 10ohm shunt
			}
			break;
		case I_HIGH:
			if(dataI1<=5) {
				handle->i_range = I_LOW;
				HAL_GPIO_WritePin(I_RANGE_SEL_GPIO_Port, I_RANGE_SEL_Pin, GPIO_PIN_RESET); //10ohm shunt
			}
			break;
		default:
			break;
		}

		voltage_tmp = (dataU1 & 0x03FF) + (PRE_U_HIGH << 10);
		if(handle->i_range == I_LOW) {
			current_tmp = (dataI2 & 0x03FF) + (PRE_I_LOW << 10);
		}
		else {
			current_tmp = (dataI1 & 0x03FF) + (PRE_I_HIGH << 10);
		}

	}
	else if(handle->status==UI_RUN) {

		if(handle->buf_last < handle->buf_first) {
			if(handle->buf_first == handle->buf_last+1) {
				handle->status = UI_ERROR;
				callback_ui_logError(handle);
				return;
			}
		}
		else if(handle->buf_first == 0 && handle->buf_last==UI_BUFFER_SIZE-1) {
			handle->status = UI_ERROR;
			callback_ui_logError(handle);
			return;
		}
		else {
			handle->counter++;
			if(++handle->buf_last == UI_BUFFER_SIZE) {
				handle->buf_last = 0;
			}


			for(i=0; i<ADC_AVERAGE_COUNT; i++) {
				dataU1+=data[ADC_U1_RANK+3*i];
			}
			for(i=0; i<ADC_AVERAGE_COUNT; i++) {
				dataI1+=data[ADC_I1_RANK+3*i];
			}
			for(i=0; i<ADC_AVERAGE_COUNT; i++) {
				dataI2+=data[ADC_I2_RANK+3*i];
			}
			dataU1 >>= 2;
			dataI1 >>= 2;
			dataI2 >>= 2;

			//dataU1 = dataU1*33*31/4096;
			dataU1 >>= 2; //Approximation for faster calculation, error about 0.1 percent
			//dataI1 = ((float)dataI1*3300/4096/64.9*10);
			dataI1 >>= 3; //Approximation for faster calculation, error about 0.7 percent
			//dataI2 = ((float)dataI2*3300/4096/59/10*100);
			dataI2 = (dataI2>>3) + (dataI2>>7) + (dataI2>>8);//Approximation for faster calculation, error about 0.2 percent

			if(handle->i_mode==AUTO) {
				switch(handle->i_range) {
				case I_LOW:
					if(dataI2>=500) {
						handle->i_range = I_HIGH;
						HAL_GPIO_WritePin(I_RANGE_SEL_GPIO_Port, I_RANGE_SEL_Pin, GPIO_PIN_SET); //Bypass 10ohm shunt
					}
					break;
				case I_HIGH:
					if(dataI1<=5) {
						handle->i_range = I_LOW;
						HAL_GPIO_WritePin(I_RANGE_SEL_GPIO_Port, I_RANGE_SEL_Pin, GPIO_PIN_RESET); //10ohm shunt
					}
					break;
				default:
					break;
				}
			}

			dataU1 = (dataU1 & 0x03FF) + (PRE_U_HIGH << 10);
			handle->buf[handle->buf_last].meas.voltage_L8 = (uint8_t)dataU1;
			handle->buf[handle->buf_last].meas.voltage_H4 = (uint8_t)(dataU1>>8);

			if(handle->i_range == I_LOW) {
				dataI2 = (dataI2 & 0x03FF) + (PRE_I_LOW << 10);
				handle->buf[handle->buf_last].meas.current_L4 = (uint8_t)(dataI2 & 0xF);
				handle->buf[handle->buf_last].meas.current_H8 = (uint8_t)(dataI2>>4);
			}
			else {
				dataI1 = (dataI1 & 0x03FF) + (PRE_I_HIGH << 10);
				handle->buf[handle->buf_last].meas.current_L4 = (uint8_t)(dataI1 & 0xF);
				handle->buf[handle->buf_last].meas.current_H8 = (uint8_t)(dataI1>>4);
			}
			handle->buf[handle->buf_last].meas.sep = '\n';
		}
	}
}

//Transmit data to host when in logging status
void ui_backgroundTask(ui_log_typedef *handle) {
	uint8_t buf_first_pos = handle->buf_first;
	uint8_t buf_last_pos = handle->buf_last;
	uint8_t *ptr;
	if(handle->status==UI_RUN) {
		//Send data to host
		if(handle->buf_first!=handle->buf_last) {
			ptr = handle->buf[buf_last_pos].data;
			uint8_t status = CDC_Transmit_FS(ptr,4);
			if(status==USBD_OK) {
				if(++buf_first_pos==UI_BUFFER_SIZE) {
					buf_first_pos = 0;
				}
				handle->buf_first = buf_first_pos;//Update counter
			}
			else if(status==USBD_FAIL) {
				handle->status = UI_ERROR;
				callback_ui_logError(handle);
			}
		}
	}
}

//Callback function when an error occurs
void callback_ui_logError(ui_log_typedef *handle) {
	if(handle->status==UI_ERROR) {
		led_green = 0;
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
		//update_OLED_error();
	}
}

//Callback function when an over-current event occurs
void callback_OverCurrent(ui_log_typedef *handle) {
	//Over-current event, the transistor should be switched off
	//HAL_GPIO_WritePin(I_MEAS_SD_GPIO_Port, I_MEAS_SD_Pin, GPIO_PIN_RESET);
	//Stop sampling
	HAL_TIM_Base_Stop(&htim2);
	//Set error flag
	handle->status = UI_ERROR;
	callback_ui_logError(handle);
}

//Callback function to calibrate the device
//Not implemented yet
void ui_calibration(ui_log_typedef *handle) {
	handle->status = UI_CAL;
}

//End of ui_log.c
