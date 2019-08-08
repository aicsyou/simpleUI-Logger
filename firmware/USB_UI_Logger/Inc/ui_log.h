#ifndef UI_LOG_H_
#define UI_LOG_H_

#include "main.h"
#include "usbd_cdc_if.h"

#define UI_BUFFER_SIZE	64
#define LOG_RATE_DEFAULT 10
#define LOG_RATE_MAX 1000

#define ADC_I1_RANK 0
#define ADC_I2_RANK 1
#define ADC_U1_RANK 2

#define ADC_BUFFER_SIZE 1
#define ADC_AVERAGE_COUNT 4 //Averaging to reduce noise
#define ADC_DMA_BUFFER_SIZE ADC_BUFFER_SIZE * ADC_AVERAGE_COUNT * 3	//3 channels

#define PRE_U_HIGH	0x1
#define PRE_I_HIGH  0x1
#define PRE_I_LOW	0x2

typedef enum {
	MANUAL,
	AUTO
}uiRangeMode;

typedef enum {
	I_LOW,
	I_HIGH
} i_range_typedef;

typedef enum {
	U_HIGH
} u_range_typedef;

typedef enum {
	UI_IDLE,
	UI_RUN,
	UI_ERROR,
	UI_CAL
}ui_status_typedef;

typedef union {
	struct {
		uint8_t current_H8;

		uint8_t voltage_H4 :4;
		uint8_t current_L4 :4;

		uint8_t voltage_L8;
		uint8_t sep;
	} meas;
	uint8_t data[4];
} ui_data_typedef;

typedef struct {
	ui_status_typedef status;
	uiRangeMode u_mode;
	uiRangeMode i_mode;
	u_range_typedef u_range;
	i_range_typedef i_range;
	uint16_t log_rate;
	uint8_t buf_first;
	uint8_t buf_last;
	ui_data_typedef *buf;
	uint8_t counter;
	//uint8_t log;
}ui_log_typedef;

typedef struct {
	__IO	int16_t 	u_offset;	//Scale factor 1000-> +59<=>+0.059
	__IO	uint16_t	u_gain_corr;
	__IO	int16_t 	i1_offset;	//Scale factor 1000-> +59<=>+0.059
	__IO	uint16_t	i1_gain_corr;
	__IO	int16_t 	i2_offset;	//Scale factor 1000-> +59<=>+0.059
	__IO	uint16_t	i2_gain_corr;
} cal_map_typedef;

void ui_initHandle(ui_log_typedef *handle);
void ui_startLog(ui_log_typedef *handle);
void ui_stopLog(ui_log_typedef *handle);
void callback_ADC_EndOfSequence(ui_log_typedef *handle, uint16_t *data);
void ui_backgroundTask(ui_log_typedef *handle);
uint8_t ui_setLogRate(ui_log_typedef *handle, uint16_t freq);
void ui_setCurrentRange(ui_log_typedef *handle, uint8_t range);
void callback_ui_logError(ui_log_typedef *handle);
void callback_OverCurrent(ui_log_typedef *handle);
void ui_calibration(ui_log_typedef *handle);

#endif
