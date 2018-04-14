#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_ps2_keyboard.h"
#include "altera_up_avalon_ps2.h"
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
#define TIME_QUEUE_SIZE 10 //LL

static TimerHandle_t timer;
static QueueHandle_t frequencyQueue;
static QueueHandle_t deltaFrequencyQueue;
static QueueHandle_t eventQueue;
static QueueHandle_t timeQueue; //LL
static QueueHandle_t frequencyQueue_out;
static QueueHandle_t deltaFrequencyQueue_out;
static double lastFreq = .0;
static int hasLastFrequency = 0;
FILE* fp;
static unsigned char esc = 0x1b;

#define EFFECTIVE_SWITCHES_MASK 0x1F // 5 loads
static unsigned int loadState = 0x0;
static unsigned int shedState = 0x0;
static SemaphoreHandle_t loadStateSem;
static SemaphoreHandle_t currentStateSem;

static unsigned short ps2ISRState = 2;
static float usrps2InputBuf = .0;

TickType_t xTime1, xTime2, xTime3;
int tst = 0;


typedef enum {
	NORMAL, MANAGED, MAINENANCE
} OperatingState;

typedef enum {
	UNSTABLE, STABLE
} Stability;

typedef enum {
	SHED, UNSHED, MAINTAIN
} EventT;

static const EventT SHED_EVENT = SHED;
static const EventT UNSHED_EVENT = UNSHED;
static const EventT MAINTAIN_EVENT = MAINTAIN;

static OperatingState currentState = NORMAL;
static Stability lastStability = UNSTABLE;


static double deltaFreq;
static double unstableThreshold = 10.5f;
#define unstableInstantThreshold 49

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

//Linked list for Time Values
struct node {
   int data;
   struct node *next;

};

typedef struct Line_S{
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
} Line;


struct node *head = NULL;
struct node *last = NULL;
struct node *high = NULL;
struct node *low = NULL;
struct node *current = NULL;

//is list empty
bool isEmpty() {
   return head == NULL;
}

int length() {
   int length = 0;
   struct node *current;

   for(current = head; current != NULL; current = current->next){
      length++;
   }

   return length;
}
//Return memory of link list in bytes
void memory() {
   int mem = 0;
   mem = length() * sizeof(struct node);
   printf("\nMemory: %d bytes", mem);
}

//display the list from first to last
void displayList() {
//	printf("\nHead0: %d", head->data);
//	   printf("\nLast0: %d", last->data);
   //start from the last
   struct node *ptr = head;

   //navigate till the start of the list

   printf("\nFull List : [ ");

   while(ptr != last) {

      //print data
      printf("(%d) ",ptr->data);

      //move to next item
      ptr = ptr ->next;

   }

   printf("(%d)] ",last->data);
   printf("\nLength = %d", length());
   printf("\nHigh: %d", high->data);
   printf("\nLow: %d", low->data);
}

//Get last 5 values. Print on screen
void displayList_5() {
   //start from the last
   struct node *ptr = head;

   //navigate till the start of the list
   printf("\nLast 5: [ ");
   int i = 0;
   while(ptr != last && i < 5) {

      //print data

      printf("(%d) ",ptr->data);

      //move to next item
      ptr = ptr ->next;
      i++;
   }
   if(length() < 6){
	   printf("(%d)] ",last->data);
   }

   printf("] ");

}


void freeList() {

   //start from the head
   struct node *ptr = head;
   struct node *followptr;

   //navigate till the start of the list
   printf("\n[ ");

   while(ptr != last) {

	  followptr = ptr;
	  ptr = ptr->next;
	  printf("\n Delete: %d", ptr->data);
	  free(followptr);
   }
   head = NULL;
}


//insert link at the first location
void insertFirst(int data) {

   //create a link
   struct node *link = (struct node*) malloc(sizeof(struct node));
   link->data = data;

   if(isEmpty()) {
      //make it the last link
      last = link;
      high = link;
      low = link;
   }

   //update high || low
   if(data >= high->data){
	   high = link;
   } else if (data <= low->data){
	   low = link;
   }

   //point it to old first link
   link->next = head;

   //point first to new first link
   head = link;
}


/****** VGA display ******/

void GUITask(void *pvParameters){
	static char timerStrBuf[10];
	static char freqStrBuf[10];
	static char rocStrBuf[10];
	static char thresholdStrBuf[10];
	static char timeStrBuf[10];
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

	alt_up_char_buffer_string(char_buf, "Frequency", 8, 40);
	alt_up_char_buffer_string(char_buf, "RoC", 8, 42);
	alt_up_char_buffer_string(char_buf, "Last Time", 8, 44);
	alt_up_char_buffer_string(char_buf, "Highest Time", 8, 46);
	alt_up_char_buffer_string(char_buf, "Lowest Time", 30, 46);
	alt_up_char_buffer_string(char_buf, "RoC Threshold", 30, 40);
	alt_up_char_buffer_string(char_buf, "System Status", 30, 42);
//	alt_up_char_buffer_string(char_buf, "AA", 20, 44); //Latest times
	alt_up_char_buffer_string(char_buf, "AA", 32, 44);
	alt_up_char_buffer_string(char_buf, "AA", 44, 44);
	alt_up_char_buffer_string(char_buf, "AA", 56, 44);
	alt_up_char_buffer_string(char_buf, "AA", 68, 44);
	alt_up_char_buffer_string(char_buf, "AA", 20, 46); //Highest Times
	alt_up_char_buffer_string(char_buf, "AA", 42, 46); //Lowest Times




	double freq[100], dfreq[100];
	double time[10] = {0};
	int i = 0, j = 0, t = 0;
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
			xQueueReceive(deltaFrequencyQueue, dfreq+i, 0);
			i = ++i % 100;
		}
		//Timer Queue
		int prnt = 0;
		while(uxQueueMessagesWaiting(timeQueue) != 0){
		xQueueReceive(timeQueue, time+t, 0);
		insertFirst(time[t]);
		displayList();
		displayList_5();
		memory();


//		while (prnt < 10){
//			printf("Time%d : %.1f\n", prnt,  time[prnt]);
//			prnt++;
//		}
			t = ++t % 10;
		}



		switch(currentState) {
		case NORMAL:
			alt_up_char_buffer_string(char_buf, "NORMAL      ", 45, 42);
			break;
		case MANAGED:
			alt_up_char_buffer_string(char_buf, "MANAGED     ", 45, 42);
			break;
		case MAINENANCE:
			alt_up_char_buffer_string(char_buf, "MAINTEANENCE", 45, 42);
			break;
		}

		sprintf(freqStrBuf, "%.2f", *(freq+savedI));
		sprintf(rocStrBuf, "%.2f",  *(dfreq+savedI));
		sprintf(thresholdStrBuf, "%.2f",  unstableThreshold);
		sprintf(timeStrBuf, "%.2f\n",  *(time+(t-1)));
		//last 5 timer values
		//sprintf(timerStrBuf, "%d ", getFirst());//LL
		//printf("%.2f\n", unstableThreshold);
		alt_up_char_buffer_string(char_buf, "            ", 45, 40);
		alt_up_char_buffer_string(char_buf, freqStrBuf, 20, 40);
		alt_up_char_buffer_string(char_buf, rocStrBuf, 20, 42);
		alt_up_char_buffer_string(char_buf, thresholdStrBuf, 45, 40);
		alt_up_char_buffer_string(char_buf, timeStrBuf, 20, 44);//LL


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

// Frequency ISR
// Modifies: currentState, frequencyQueue, deltaFrequencyQueue
void freqIsr(){
	unsigned int samples;
	double freq;
	samples = IORD(FREQUENCY_ANALYSER_BASE, 0);
	freq = SAMPLE_RATE / (double)samples;
	if(hasLastFrequency) {
		// We can calculate the delta frequency
		deltaFreq = (freq - lastFreq) * 2.0 * freq * lastFreq / (freq + lastFreq);
		//deltaFreq = (lastFreq - freq);// / ((double)samples / SAMPLE_RATE);
		xQueueSendToBackFromISR(frequencyQueue, &freq, pdFALSE);
		xQueueSendToBackFromISR(deltaFrequencyQueue, &deltaFreq, pdFALSE);
		lastFreq = freq;
	} else {
		lastFreq = freq;
		hasLastFrequency = 1;
	}

        // Get the currentState sem
        xSemaphoreTakeFromISR(currentStateSem, 1000);
	if(currentState == MANAGED) {
		// We need to check the timer
		if(freq < unstableInstantThreshold || deltaFreq > unstableThreshold) {
			// Its unstable

//			printf("Unstable (Managed) : %d \n", (int)xTime1);
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
			xTime1 = xTaskGetTickCount();
			tst = 0;
			// Normal state and unstable
			currentState = MANAGED;
			xTimerStartFromISR(timer, 0);
			// Send a SHED Event to the consumer
			xQueueSendToBackFromISR(eventQueue, &SHED_EVENT, NULL);
		}
	}

	switch(currentState) {
	case NORMAL:
		fprintf(fp, "%c%sNORMAL\nFreq:%.2f\n", esc, "[2J", unstableThreshold);
		break;
	case MANAGED:
		fprintf(fp, "%c%sMANAGED\nFreq: %.2f\n", esc, "[2J", unstableThreshold);
		break;
	case MAINENANCE:
		fprintf(fp, "%c%sMAINTEANCE\nFreq:%.2f\n", esc, "[2J", unstableThreshold);
		break;
	}
        xSemaphoreGiveFromISR(currentStateSem, 0);
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
	//xTime3 = xTaskGetTickCount();
	xSemaphoreTake(loadStateSem, 1000);
	unsigned int maskedState = loadState & EFFECTIVE_SWITCHES_MASK;
	if(maskedState != 0x0) {
		int i;
		for(i = 0; i < 0xFF; i++) {
			if((maskedState & (0x1 << i)) != 0x0) {
				loadState = maskedState ^ (0x1 << i);
				shedState = shedState | (0x1 << i);
				xSemaphoreGive(loadStateSem);
				xTime3 = xTaskGetTickCount();
				double executiontime = (double)xTime3 - (double)xTime1;
				if (tst == 0) {
					//insertFirst(1, executiontime);//LL
					printf("\nExecution Time: %f ms \n", executiontime);
					xQueueSendToBackFromISR(timeQueue, &executiontime, pdFALSE);
					tst++;
				} else { tst++; }

				return;
			}
		}
	}
	xSemaphoreGive(loadStateSem);
}

// Consume events produced, will be modifiying the currentState
void eventConsumerTask(void* param) {
	for(;;) {
		//taskENTER_CRITICAL();
		EventT event;
		if(xQueueReceive(eventQueue, &event, portMAX_DELAY) == pdTRUE) {
			// We got an event
			switch (event) {
			case UNSHED:
				if (tryTurnOnLoad()) {
					xSemaphoreTake(currentStateSem, 1000);
					currentState = NORMAL;
					xSemaphoreGive(currentStateSem);
					xTimerStop(timer, 0);
				}
				break;
			case SHED:
				tryTurnOffLoad();
				break;
			case MAINTAIN:
					xSemaphoreTake(currentStateSem, 1000);
					if(currentState == MAINENANCE) {
						xTimerStart(timer, 10);
						currentState = NORMAL;
					} else {
						xTimerStop(timer, 10);
						currentState = MAINENANCE;
					}
					xSemaphoreGive(currentStateSem);
					break;
			}
		}
		//taskEXIT_CRITICAL();
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

unsigned int findMinOffBit(unsigned int usr) {
	usr = usr & EFFECTIVE_SWITCHES_MASK;
	int i = 0;
	for(; i < 5; i++) {
		if((usr & (0x1 << i)) == 0x0 && (loadState & (0x1 << i))) {
			return 0x1 << i;
		}
	}
	return 0;

}

// Continuously monitor the state of the switch
void userSwitchMonitorTask(void *param) {
	for(;;) {
		// Read the slide switches and save it
		unsigned int usrState = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & EFFECTIVE_SWITCHES_MASK;
		unsigned int minBit;
		// Lock the loadState
		xSemaphoreTake(loadStateSem, 1000);
		switch(currentState) {
		case NORMAL:
		case MAINENANCE:
			// Normal operating state, userState is effective
			loadState = usrState;
			shedState = 0x0;
			break;
		case MANAGED:
			// Managed state, only switching off is effective
			minBit = findMinOffBit(usrState);
			if(minBit) {
				loadState = loadState & (EFFECTIVE_SWITCHES_MASK ^ minBit);
			}
			break;
		}
		xSemaphoreGive(loadStateSem);
		vTaskDelay(100);
	}
}

void pushButtonIsr() {
	xQueueSendToBackFromISR(eventQueue, &MAINTAIN_EVENT, 0);
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7); //write 1 to clear all detected falling edges
	/*
        xSemaphoreTakeFromISR(currentStateSem, 1000);
        if(currentState == MAINENANCE) {
            currentState = NORMAL;
            xTimerStart(timer, 10);
        } else {
            currentState = MAINENANCE;
            xTimerStop(timer, 0);
        }
        xSemaphoreGiveFromISR(currentStateSem, 0);
        */
	return;
}
//int hexadecimal_to_decimal(int x)
//{
//      int decimal_number, remainder, count = 0;
//      while(x > 0)
//      {
//            remainder = x % 10;
//            decimal_number = decimal_number + remainder * pow(16, count);
//            x = x / 10;
//            count++;
//      }
//      return decimal_number;
//}

static unsigned short afterDec = 0;
static unsigned int decPt = 0;



void ps2_isr(void* ps2_device, alt_u32 id){
	//unsigned char byte;
	//alt_up_ps2_read_data_byte(ps2_device, &byte);
	char ascii;
	int status = 0;
	unsigned char key = 0;
	KB_CODE_TYPE decode_mode;
	status = decode_scancode(ps2_device, &decode_mode , &key , &ascii);
	if(!ps2ISRState) {
		printf("%d\n", ps2ISRState);
		ps2ISRState++;
	} else {
		printf("%d\n", ps2ISRState);
		ps2ISRState = (ps2ISRState + 1) % 3;
		return;
	}

	char offsetChar = ascii - 48;

	if(key == 0x5a) {
		unstableThreshold = usrps2InputBuf / (pow(10, decPt));
		usrps2InputBuf = .0;
		afterDec = 0;
		decPt = 0;
	} else if(key == 0x71 || key == 0x49){
		// dot
		afterDec = 1;
	} else if(offsetChar >= 0 && offsetChar < 10) {
		// num
		if(afterDec) {
			decPt++;
		}
		usrps2InputBuf *= 10;
		usrps2InputBuf += (int)offsetChar;
	}

	printf("usr : %f\n", usrps2InputBuf);
	printf("stat : %f\n", unstableThreshold);

	  if ( status == 0 ) //success
	  {
	    // print out the result
	    switch ( decode_mode )
	    {
	      case KB_ASCII_MAKE_CODE :
	        printf ( "ASCII   : %c\n", ascii ) ;
	        break ;
	      case KB_LONG_BINARY_MAKE_CODE :
	        // do nothing
	      case KB_BINARY_MAKE_CODE :
	        printf ( "MAKE CODE : %x\n", key ) ;
	        break ;
	      case KB_BREAK_CODE :
	        // do nothing
	      default :
	        printf ( "DEFAULT   : %x\n", key ) ;
	        break ;
	    }
//	    IOWR(SEVEN_SEG_BASE,0 , usrps2InputBuf);

	  }
	return;
}




int main()
{
	// Setup
	loadState = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE) & EFFECTIVE_SWITCHES_MASK;
	IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, loadState);
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7); //enable interrupt for all three push buttons (Keys 1-3 -> bits 0-2)
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7); //write 1 to edge capture to clear pending interrupts
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);
	alt_up_ps2_enable_read_interrupt(ps2_device);
	alt_up_ps2_clear_fifo (ps2_device);

	// Create the necessary data structures
	frequencyQueue = xQueueCreate(FREQUENCY_QUEUE_SIZE, sizeof(double));
	deltaFrequencyQueue = xQueueCreate(FREQUENCY_QUEUE_SIZE, sizeof(double));
	frequencyQueue_out = xQueueCreate(FREQUENCY_QUEUE_SIZE, sizeof(double));
	deltaFrequencyQueue_out = xQueueCreate(FREQUENCY_QUEUE_SIZE, sizeof(double));
	eventQueue = xQueueCreate(FREQUENCY_QUEUE_SIZE, sizeof(EventT));
	timeQueue = xQueueCreate(TIME_QUEUE_SIZE, sizeof(double)); //LL
	loadStateSem = xSemaphoreCreateBinary();
	currentStateSem = xSemaphoreCreateBinary();
	fp = fopen(CHARACTER_LCD_NAME, "w"); //open the character LCD as a file stream for write

    // Setup timers
    timer = xTimerCreate("Timer Name", 500, pdTRUE, NULL, vTimerCallback);

    // Register the ISRs
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freqIsr);
	alt_irq_register(PUSH_BUTTON_IRQ, 1, pushButtonIsr);
	alt_irq_register(PS2_IRQ, ps2_device, ps2_isr);

    // Create Tasks
	//xTaskCreate(GUITask, "DrawTsk", configMINIMAL_STACK_SIZE, NULL, GUITask_P, &GUITask_H);
	//xTaskCreate(frequencyCalculationTask, "FreqCalcTask", configMINIMAL_STACK_SIZE, NULL, 31, NULL);
	xTaskCreate(userSwitchMonitorTask, "UsrSwitchMonTask", 4096, NULL, 31, NULL);
	xTaskCreate(updateLEDTask, "UpdateLEDTask", 4096, NULL, 30, NULL);
	xTaskCreate(eventConsumerTask, "EventConsumerTask", 4096, NULL, 30, NULL);
	xTaskCreate(GUITask, "GUITask", 8192, NULL, 29, NULL);

//	insertFirst(1, 10);
//	insertFirst(2, 20);
//	insertFirst(3, 30);
//	insertFirst(4, 1);
//	insertFirst(5, 40);
//	insertFirst(6, 56);
//	insertFirst(6, 0);

//	displayList();
//	freeList();
	//displayList();


	// Start the scheduler
	vTaskStartScheduler();
	while(1){
	}

  return 0;
}
