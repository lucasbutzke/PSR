/*
 * FreeRTOS Kernel V10.3.1
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/******************************************************************************
 * NOTE: Windows will not be running the FreeRTOS demo threads continuously, so
 * do not expect to get real time behaviour from the FreeRTOS Windows port, or
 * this demo application.  Also, the timing information in the FreeRTOS+Trace
 * logs have no meaningful units.  See the documentation page for the Windows
 * port for further information:
 * http://www.freertos.org/FreeRTOS-Windows-Simulator-Emulator-for-Visual-Studio-and-Eclipse-MingW.html
 *
 * NOTE 2:  This project provides two demo applications.  A simple blinky style
 * project, and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting in main.c is used to select
 * between the two.  See the notes on using mainCREATE_SIMPLE_BLINKY_DEMO_ONLY
 * in main.c.  This file implements the simply blinky version.  Console output
 * is used in place of the normal LED toggling.
 *
 * NOTE 3:  This file only contains the source code that is specific to the
 * basic demo.  Generic functions, such FreeRTOS hook functions, are defined
 * in main.c.
 ******************************************************************************
 *
 * main_blinky() creates one queue, one software timer, and two tasks.  It then
 * starts the scheduler.
 *
 * The Queue Send Task:
 * The queue send task is implemented by the prvQueueSendTask() function in
 * this file.  It uses vTaskDelayUntil() to create a periodic task that sends
 * the value 100 to the queue every 200 milliseconds (please read the notes
 * above regarding the accuracy of timing under Windows).
 *
 * The Queue Send Software Timer:
 * The timer is a one-shot timer that is reset by a key press.  The timer's
 * period is set to two seconds - if the timer expires then its callback
 * function writes the value 200 to the queue.  The callback function is
 * implemented by prvQueueSendTimerCallback() within this file.
 *
 * The Queue Receive Task:
 * The queue receive task is implemented by the prvQueueReceiveTask() function
 * in this file.  prvQueueReceiveTask() waits for data to arrive on the queue.
 * When data is received, the task checks the value of the data, then outputs a
 * message to indicate if the data came from the queue send task or the queue
 * send software timer.
 *
 * Expected Behaviour:
 * - The queue send task writes to the queue every 200ms, so every 200ms the
 *   queue receive task will output a message indicating that data was received
 *   on the queue from the queue send task.
 * - The queue send software timer has a period of two seconds, and is reset
 *   each time a key is pressed.  So if two seconds expire without a key being
 *   pressed then the queue receive task will output a message indicating that
 *   data was received on the queue from the queue send software timer.
 *
 * NOTE:  Console input and output relies on Windows system calls, which can
 * interfere with the execution of the FreeRTOS Windows port.  This demo only
 * uses Windows system call occasionally.  Heavier use of Windows system calls
 * can crash the port.
 */

/* Standard includes. */
#include <stdio.h>
#include <conio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* The rate at which data is sent to the queue.  The times are converted from
milliseconds to ticks using the pdMS_TO_TICKS() macro. */
#define mainTASK_SEND_FREQUENCY_MS			pdMS_TO_TICKS( 200UL )
#define mainTIMER_SEND_FREQUENCY_MS			pdMS_TO_TICKS( 5000UL )

/*
 * Os protótipos das tarefas. 
 */
static void matrix_task(void* pvParameters);
void vTimerCallBack(TimerHandle_t pxTimer);
static void aperiodic_task();
static void priorityset_task(void* pvParameters);

/* Task handles para definicoes dos parametros na criacao das tarefas. */
TaskHandle_t matrix_handle = NULL;
TaskHandle_t aperiodic_handle = NULL;
TaskHandle_t priority_handle = NULL;

/* Variaveis de contagem de tempo. */
TickType_t xMatrixTime;
TickType_t xTimerTime;


/*-----------------------------------------------------------*/

#define SIZE 10
#define ROW SIZE
#define COL SIZE

/* A software timer that is started from the tick hook. */
static TimerHandle_t pxTimer = NULL;

/*-----------------------------------------------------------*/

/*
* Realizar uma funcao timer para iniciar uma funcao aperiodica a cada 5s,
* a funcao aperiodica executa concorrentemente com matrix_task, verificar 
* conclusao das tarefas dentro do periodo do timer.
*/
void main_blinky( void )
{
const TickType_t xTimerPeriod = mainTIMER_SEND_FREQUENCY_MS;

		/* Start the two tasks as described in the comments at the top of this file. */
		xTaskCreate((pdTASK_CODE)matrix_task,		/* The function that implements the task. */
					(signed char*)"Matrix", 		/* The text name assigned to the task - for debug only as it is not used by the kernel. */
					1000, 							/* The size of the stack to allocate to the task. */
					NULL, 							/* The parameter passed to the task - not used in this simple case. */
					3,								/* The priority assigned to the task. */
					&matrix_handle);				/* The task handle is not required, so NULL is passed. */

		// tarefa de gerenciamento de prioridades para executar as tarefas dentro do deadline
		xTaskCreate((pdTASK_CODE)priorityset_task, (signed char*)"Priority", configMINIMAL_STACK_SIZE, NULL, 3, &priority_handle);

		/* Create the software timer, but don't start it yet. */
		pxTimer = xTimerCreate( "Timer_Task",		/* The text name assigned to the software timer - for debug only as it is not used by the kernel. */
								xTimerPeriod,		/* The period of the software timer in ticks. */
								pdTRUE,				/* xAutoReload is set to pdTrue, so this timer keep going until xTimerStop(). */
								NULL,				/* The timer's ID is not used. */
								vTimerCallBack);	/* The function executed when the timer expires. */

		xTimerStart( pxTimer, 0 ); /* The scheduler has not started so use a block time of 0. */

		/* Start the tasks and timer running. */
		vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was insufficient FreeRTOS heap memory available for the idle and/or
	timer tasks	to be created.  See the memory management section on the
	FreeRTOS web site for more details. */
	for( ;; );
}
/*-----------------------------------------------------------*/

static void matrix_task(void* pvParameters) 
{
	//printf("comecando matrix_task \n");
	int i;
	double** a = (double**)pvPortMalloc(ROW * sizeof(double*));
	for (i = 0; i < ROW; i++) a[i] = (double*)pvPortMalloc(COL * sizeof(double));
	double** b = (double**)pvPortMalloc(ROW * sizeof(double*));
	for (i = 0; i < ROW; i++) b[i] = (double*)pvPortMalloc(COL * sizeof(double));
	double** c = (double**)pvPortMalloc(ROW * sizeof(double*));
	for (i = 0; i < ROW; i++) c[i] = (double*)pvPortMalloc(COL * sizeof(double));

	double sum = 0.0;
	int j, k, l;

	for (i = 0; i < SIZE; i++) {
		for (j = 0; j < SIZE; j++) {
			a[i][j] = 1.5;
			b[i][j] = 2.6;
		}
	}

	while (1) {
		/*
		 * In an embedded systems, matrix multiplication would block the CPU for a long time
		 * but since this is a PC simulator we must add one additional dummy delay.
		 */
		xMatrixTime = xTaskGetTickCount();
		long simulationdelay;
		for (simulationdelay = 0; simulationdelay < 1000000000; simulationdelay++);
		for (i = 0; i < SIZE; i++) {
			for (j = 0; j < SIZE; j++) {
				c[i][j] = 0.0;
			}
		}

		for (i = 0; i < SIZE; i++) {
			for (j = 0; j < SIZE; j++) {
				sum = 0.0;
				for (k = 0; k < SIZE; k++) {
					for (l = 0; l < 10; l++) {
						sum = sum + a[i][k] * b[k][j];
					}
				}
				c[i][j] = sum;
			}
		}
		vTaskDelay(100);
		printf("Tempo executando funcao matrix_task:	%ld	\r\n", xTaskGetTickCount() - xMatrixTime);
		fflush(stdout);
	}
}

long lExpireCounters = 0;
void vTimerCallBack(TimerHandle_t pxTimer) 
{
	printf("Timer Callback \n");
	xTaskCreate(aperiodic_task, "Aperiodic", configMINIMAL_STACK_SIZE, NULL, 2, &aperiodic_handle);
	long lArrayIndex;
	const long xMaxExpiryCountBeforeStopping = 10;
	/* Optionally do something if the pxTimer parameter is NULL. */
	configASSERT(pxTimer);
	/* Increment the number of times that pxTimer has expired. */
	lExpireCounters += 1;
	/* If the timer has expired 10 times then stop it from running. */
	if (lExpireCounters == xMaxExpiryCountBeforeStopping) {
		/* Do not use a block time if calling a timer API function from a
		* timer callback function, as doing so could cause a deadlock!
		*/
		xTimerStop(pxTimer, 0);
	}
}

static void aperiodic_task()
{
	xTimerTime = xTaskGetTickCount();
	printf("Aperiodic task started! \n");
	fflush(stdout);
	for(long i = 0; i < 2000000000; i++); //Dummy workload
	printf("Aperiodic task done \n");
	fflush(stdout);
	printf("Tempo executando funcao aperiodic_task:	%ld	\r\n", xTaskGetTickCount() - xTimerTime);
	fflush(stdout);
	vTaskDelete(aperiodic_handle);
}

/*
 * Trocar de prioridade caso tarefa aperiodica levar maior 
 * tempo que o periodo do timer, certificar conclusao antes
 * da proxima tarefa. Criar um "custo de execucao".
 */
static void priorityset_task(void* pvParameters) {
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = 1000;
	int custo = 0;

	while (1)
	{
		if (xTaskGetTickCount() - xTimerTime >= xFrequency)		//caso a funcao aperiodica ultrapassar um valor de execucao
		{
			if (custo >= 1)
			{
				//printf("o orcamento eh: %d	\r\n", custo);
				//fflush(stdout);
				//vTaskSuspend(matrix_handle);
				//vTaskResume(aperiodic_handle);
				vTaskPrioritySet(aperiodic_handle, 4);			// aumenta prioridade quando consumido orcamento
				custo -= 1;
				//vTaskDelayUntil(&xLastWakeTime, xFrequency/2);	// tempo de execucao
			}
			else
			{
				//printf("o orcamento eh: %d	\r\n", custo);
				//fflush(stdout);
				//vTaskResume(matrix_handle);
				//vTaskSuspend(aperiodic_handle);
				vTaskPrioritySet(aperiodic_handle, 2);			// reduz prioridade quando sem orcamento
				//vTaskDelayUntil(&xLastWakeTime, xFrequency/2);	// tempo de execucao
			}

			if (xTaskGetTickCount() - xLastWakeTime >= xFrequency)		// ciclo do orcamento
			{
				xLastWakeTime = xTaskGetTickCount();
				custo += 1;
			}
		}
		//vTaskDelayUntil(&xLastWakeTime, xFrequency / 2);
	}
}
