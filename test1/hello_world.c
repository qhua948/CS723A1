#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"
#include "system.h"
#include "sys/alt_irq.h"
#include "io.h"
#include "altera_avalon_pio_regs.h"
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/timers.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/semphr.h"

#define Flash_Task_P (tskIDLE_PRIORITY+1)
#define SAMPLE_RATE 16000
#define FREQUENCY_QUEUE_SIZE 100

static TimerHandle_t timer;
static TimerHandle_t timerReset;
static TaskHandle_t GUITask_H;
static TaskHandle_t normalTask_H;
static TaskHandle_t managedTask_H;
static TaskHandle_t maintenanceTask_H;
static QueueHandle_t frequencyQueue;
static QueueHandle_t deltaFrequencyQueue;
static QueueHandle_t eventQueue;
static QueueHandle_t frequencyQueue_out;
static QueueHandle_t deltaFrequencyQueue_out;
static int hasFirstFrequency = 0;
static int resettingTimer = 0;
static int wasUnstable = 0;
static double lastFreq = .0;
static int hasLastFrequency = 0;
FILE* fp;
static unsigned char esc = 0x1b;

#define EFFECTIVE_SWITCHES_MASK 0x1F // 5 loads
static unsigned int loadState = 0x0;
static unsigned int shedState = 0x0;
static SemaphoreHandle_t loadStateSem;

typedef enum {
	NORMAL, MANAGED, MAINENANCE
} OperatingState;

typedef enum {
	UNSTABLE, STABLE
} Stability;

typedef enum {
	SHED, UNSHED
} EventT;

static const EventT SHED_EVENT = SHED;
static const EventT UNSHED_EVENT = UNSHED;


static OperatingState currentState = NORMAL;
static Stability lastStability = UNSTABLE;


static double deltaFreq;
static unsigned int freq = 60;
static double unstableThreshold = 0.5f;
#define unstableInstantThreshold 40

//For frequency plot
#define FREQPLT_ORI_X 101		//x axis pixel position at the plot origin
#define FREQPLT_GRID_SIZE_X 5	//pixel separation in the x axis between two data points
#define FREQPLT_ORI_Y 199.0		//y axis pixel position at the plot origin
#define FREQPLT_FREQ_RES 20.0	//number of pixels per Hz (y axis scale)

#define ROCPLT_ORI_X 101
#define ROCPLT_GRID_SIZE_X 5
#define ROCPLT_ORI_Y 259.0
#define ROCPLT_ROC_RES 0.5		//number of pixels per Hz/s (y axis scale)

#define MIN_FREQ 45.0 //minimum frequency to draw

#define GUITask_P      (tskIDLE_PRIORITY+1)


typedef struct{
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
}Line;


/****** VGA display ******/

void GUITask(void *pvParameters){
	//initialize VGA controllers
	alt_up_pixel_buffer_dma_dev *pixel_buf;
	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);
	if(pixel_buf == NULL){
		printf("can't find pixel buffer device\n");
	}
	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);

	alt_up_char_buffer_dev *char_buf;
	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
	if(char_buf == NULL){
		printf("can't find char buffer device\n");
	}
	alt_up_char_buffer_clear(char_buf);


	//Set up plot axes
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 50, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 220, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);

	alt_up_char_buffer_string(char_buf, "Frequency(Hz)", 4, 4);
	alt_up_char_buffer_string(char_buf, "52", 10, 7);
	alt_up_char_buffer_string(char_buf, "50", 10, 12);
	alt_up_char_buffer_string(char_buf, "48", 10, 17);
	alt_up_char_buffer_string(char_buf, "46", 10, 22);

	alt_up_char_buffer_string(char_buf, "df/dt(Hz/s)", 4, 26);
	alt_up_char_buffer_string(char_buf, "60", 10, 28);
	alt_up_char_buffer_string(char_buf, "30", 10, 30);
	alt_up_char_buffer_string(char_buf, "0", 10, 32);
	alt_up_char_buffer_string(char_buf, "-30", 9, 34);
	alt_up_char_buffer_string(char_buf, "-60", 9, 36);


	double freq[100], dfreq[100];
	int i = 0, j = 0;
	Line line_freq, line_roc;

	while(1){

		//receive frequency data from queue
		int savedI = i;
		while(uxQueueMessagesWaiting(frequencyQueue) != 0){
			xQueueReceive(frequencyQueue, freq+i, 0);
			i = ++i % 100;
		}
		i = savedI;
		while(uxQueueMessagesWaiting(deltaFrequencyQueue) != 0){
			xQueueReceive(deltaFrequencyQueue, dfreq+j, 0);
			i = ++i % 100;
		}


		//clear old graph to draw new graph
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 0, 639, 199, 0, 0);
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 201, 639, 299, 0, 0);

		for(j=0;j<99;++j){ //i here points to the oldest data, j loops through all the data to be drawn on VGA
			if (((int)(freq[(i+j)%100]) > MIN_FREQ) && ((int)(freq[(i+j+1)%100]) > MIN_FREQ)){
				//Calculate coordinates of the two data points to draw a line in between
				//Frequency plot
				line_freq.x1 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * j;
				line_freq.y1 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j)%100] - MIN_FREQ));

				line_freq.x2 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * (j + 1);
				line_freq.y2 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j+1)%100] - MIN_FREQ));

				//Frequency RoC plot
				line_roc.x1 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * j;
				line_roc.y1 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(i+j)%100]);

				line_roc.x2 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * (j + 1);
				line_roc.y2 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(i+j+1)%100]);

				//Draw
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_freq.x1, line_freq.y1, line_freq.x2, line_freq.y2, 0x3ff << 0, 0);
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_roc.x1, line_roc.y1, line_roc.x2, line_roc.y2, 0x3ff << 0, 0);
			}
		}
		vTaskDelay(10);

	}
}

double inline absoluteVal(double val) {
	if(val < 0) return -val; else return val;
}

void freqIsr(){
	unsigned int samples;
	double freq;
	samples = IORD(FREQUENCY_ANALYSER_BASE, 0);
	freq = SAMPLE_RATE / (double)samples;
	xQueueSendToBackFromISR(frequencyQueue, &freq, pdFALSE);
	if(hasLastFrequency) {
		// We can calculate the delta frequency
		deltaFreq = (freq - lastFreq) * 2.0 * freq * lastFreq / (freq + lastFreq);
		xQueueSendToBackFromISR(deltaFrequencyQueue, &freq, pdFALSE);
	} else {
		lastFreq = freq;
		hasLastFrequency = 1;
	}

	if(currentState == MANAGED) {
		/* TODO: These clause could be refractored */
		// We need to check the timer
		if(freq < unstableInstantThreshold || deltaFreq > unstableThreshold) {
			// Its unstable
			if(lastStability == STABLE) {
				// Reset timer
				xTimerResetFromISR(timer, NULL);
			}
			lastStability = UNSTABLE;
		} else {
			// Its stable
			if(lastStability != STABLE) {
				// Reset timer
				xTimerResetFromISR(timer, NULL);
			}
			lastStability = STABLE;
		}
	} else if(currentState == NORMAL) {
		if(freq < unstableInstantThreshold || deltaFreq > unstableThreshold) {
			// Normal state and unstable
			currentState = MANAGED;
			xTimerStartFromISR(timer, 0);
			// Send a SHED Event to the consumer
			xQueueSendToBackFromISR(eventQueue, &SHED_EVENT, NULL);
		}
	}

	switch(currentState) {
	case NORMAL:
		fprintf(fp, "%c%sNORMAL\n", esc, "[2J");
		break;
	case MANAGED:
		fprintf(fp, "%c%sMANAGED\n", esc, "[2J");
		break;
	case MAINENANCE:
		fprintf(fp, "%c%sMAINTEANCE\n", esc, "[2J");
		break;
	}
	return;
}

// Try to turn on a load, thread safe
// Will also managed the green LEDs
// Return a 0 when a load has been turned on, a non-zero value if all load were on before this function is called
unsigned int tryTurnOnLoad() {
	xSemaphoreTake(loadStateSem, 1000);
	unsigned int usrState = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & EFFECTIVE_SWITCHES_MASK;
	unsigned int maskedState = EFFECTIVE_SWITCHES_MASK & loadState;
	if(usrState == maskedState) {
		// All loads are all on, do nothing and return an non-zero value
		xSemaphoreGive(loadStateSem);
		return 1;
	} else {
		int i;
		for(i = 4; i >= 0x0; i--) {
			if(((maskedState & (0x1 << i)) == 0) && (usrState & (0x1 << i)) != 0) {
				// Turn the one with highest priority on
				loadState = maskedState | (0x1 << i);
				shedState = shedState & (0xFF ^ (0x1) << i) & EFFECTIVE_SWITCHES_MASK;
				xSemaphoreGive(loadStateSem);
				return 0;
			}
		}
	}
}

// Analogous to tryTurnOnLoad(), thread safe
void tryTurnOffLoad() {
	xSemaphoreTake(loadStateSem, 1000);
	unsigned int maskedState = loadState & EFFECTIVE_SWITCHES_MASK;
	if(maskedState != 0x0) {
		int i;
		for(i = 0; i < 0xFF; i++) {
			if((maskedState & (0x1 << i)) != 0x0) {
				loadState = maskedState ^ (0x1 << i);
				shedState = shedState | (0x1 << i);
				xSemaphoreGive(loadStateSem);
				return;
			}
		}
	}
	xSemaphoreGive(loadStateSem);
}

// Consume events produced, will be modifiying the currentState
void eventConsumerTask(void* param) {
	for(;;) {
		EventT event;
		if(xQueueReceive(eventQueue, &event, portMAX_DELAY) == pdTRUE) {
			// We got an event
			switch(event) {
			case UNSHED:
				if(tryTurnOnLoad()) {
					currentState = NORMAL;
					xTimerStop(timer, 0);
				}
				break;
			case SHED:
				tryTurnOffLoad();
				break;
			}
		}
		vTaskDelay(100);
	}
}

// This function is called when the timers reached the ticked count
// See main(), one of the EventT producers
void vTimerCallback(void *param) {
	// This function would only be called during managed mode, where user state is ignored
	EventT event = lastStability == STABLE ? UNSHED : SHED;
	xQueueSendToBack(eventQueue, &event, 1000);
	return;
}

// Update the LEDs
void updateLEDTask(void *param) {
	for(;;) {
		xSemaphoreTake(loadStateSem, 1000);
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, loadState);
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, shedState);
		xSemaphoreGive(loadStateSem);
		vTaskDelay(100);
	}
}

// Continuously monitor the state of the switch
void userSwitchMonitorTask(void *param) {
	for(;;) {
		// Read the slide switches and save it
                unsigned int usrState = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & EFFECTIVE_SWITCHES_MASK;
		// Lock the loadState
		xSemaphoreTake(loadStateSem, 1000);
		switch(currentState) {
		case NORMAL:
			// Normal operating state, userState is effective
			loadState = usrState;
			shedState = 0x0;
			break;
		case MANAGED:
			// Managed state, only switching off is effective
			if(usrState < loadState) {
				// User Switching shit off, we should allow them
				loadState = usrState;
			}
			break;
		case MAINENANCE:
			// TODO: Implement mt state
			break;
		}
		xSemaphoreGive(loadStateSem);
		vTaskDelay(100);
	}
}

// TODO: Maintenance
void pushButtonIsr() {
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7); //enable interrupt for all three push buttons (Keys 1-3 -> bits 0-2)
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7); //write 1 to edge capture to clear pending interrupts
	return;
}

int main()
{
    // Setup
        loadState = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & EFFECTIVE_SWITCHES_MASK;
        IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, loadState);

        // Create the necessary data structures
        frequencyQueue = xQueueCreate(FREQUENCY_QUEUE_SIZE, sizeof(double));
        deltaFrequencyQueue = xQueueCreate(FREQUENCY_QUEUE_SIZE, sizeof(double));
        frequencyQueue_out = xQueueCreate(FREQUENCY_QUEUE_SIZE, sizeof(double));
        deltaFrequencyQueue_out = xQueueCreate(FREQUENCY_QUEUE_SIZE, sizeof(double));
        eventQueue = xQueueCreate(FREQUENCY_QUEUE_SIZE, sizeof(EventT));
        loadStateSem = xSemaphoreCreateBinary();
	fp = fopen(CHARACTER_LCD_NAME, "w"); //open the character LCD as a file stream for write

    // Setup timers
    timer = xTimerCreate("Timer Name", 500, pdTRUE, NULL, vTimerCallback);

    // Register the ISRs
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freqIsr);

    // Create Tasks
	//xTaskCreate(GUITask, "DrawTsk", configMINIMAL_STACK_SIZE, NULL, GUITask_P, &GUITask_H);
	//xTaskCreate(frequencyCalculationTask, "FreqCalcTask", configMINIMAL_STACK_SIZE, NULL, 31, NULL);
	xTaskCreate(userSwitchMonitorTask, "UsrSwitchMonTask", 4096, NULL, 31, NULL);
	xTaskCreate(updateLEDTask, "UpdateLEDTask", 4096, NULL, 30, NULL);
	xTaskCreate(eventConsumerTask, "EventConsumerTask", 4096, NULL, 30, NULL);
	xTaskCreate(GUITask, "GUITask", 8192, NULL, 29, NULL);

	// Start the scheduler
	vTaskStartScheduler();
	while(1){
	}

  return 0;
}
