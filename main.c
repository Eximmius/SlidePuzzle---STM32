
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "atraso.h"
#include "defPrincipais.h"
#include "NOKIA5110_fb.h"
#include "figuras.h"
#include "PRNG_LFSR.h"
#include "sound.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

osThreadId defaultTaskHandle;
osTimerId Timer01Handle;
osSemaphoreId transiOkHandle;
osSemaphoreId printLCDHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
xQueueHandle xCommand;
xQueueHandle xTransi;
xQueueHandle xSound;

osSemaphoreId printOkHandle;

TaskHandle_t JoyHandle;
TaskHandle_t SoundSHandle;
TaskHandle_t PuzzleHandle;
TaskHandle_t TransiHandle;
TaskHandle_t PrintHandle;

uint32_t ADC_buffer[2];
uint32_t valor_ADC[2];
uint32_t Puzzle[16];
uint32_t counter;

struct figura_t* figuras[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);
void Callback01(void const * argument);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

// ----Funções-----
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void change_note (TIM_HandleTypeDef *htim, int nota);

// ----Tarefas-----
void vT_Create_Puzz (void *pv);
void vT_Joy(void *pv);
void vT_SoundS(void *pv);
void vT_Puzzle (void* pv);
void vT_Transi (void * pv);
void vT_Print (void * pv);
void vT_Restart (void * pv);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //LCD 5110
  inic_LCD();
  limpa_LCD();
  imprime_LCD();

  //ADC Joystick
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_buffer,2);
  HAL_ADC_Start_IT(&hadc1);

  //PWM Buzzer
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of transiOk */
  osSemaphoreDef(transiOk);
  transiOkHandle = osSemaphoreCreate(osSemaphore(transiOk), 1);

  /* definition and creation of printLCD */
  osSemaphoreDef(printLCD);
  printLCDHandle = osSemaphoreCreate(osSemaphore(printLCD), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  osSemaphoreDef(printOkHandle);
  printOkHandle = osSemaphoreCreate(osSemaphore(printOkHandle), 1);
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of Timer01 */
  osTimerDef(Timer01, Callback01);
  Timer01Handle = osTimerCreate(osTimer(Timer01), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  //
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  uint32_t vT_Create = pdTRUE;
  vT_Create = xTaskCreate(vT_Create_Puzz,"Create",150, NULL, 4, NULL);

  if (vT_Create == pdFALSE){
  		//abort
  		string_LCD("Erro ao criar tarefas");
  		imprime_LCD();
  		while(1);
  	}

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  xCommand = xQueueCreate(1, sizeof(uint32_t));
  xTransi = xQueueCreate(1, sizeof(uint32_t));
  xSound = xQueueCreate(1, sizeof(uint32_t));

  //vQueueAddToRegistry(transiOkHandle,"TransiQ");
  /* USER CODE END RTOS_QUEUES */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = TIM3_COUNT;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 700;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6 
                           PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc->Instance == ADC1)
	{
		valor_ADC[0]=ADC_buffer[0];
		valor_ADC[1]=ADC_buffer[1];
	}
}
void change_note (TIM_HandleTypeDef *htim, int nota){
	uint32_t prescaler;
	if (nota>0)
		prescaler = SystemCoreClock/(TIM3_COUNT*nota);
	else
		prescaler =0;
	__HAL_TIM_SET_PRESCALER(htim, prescaler);
}


//---------------------------------------------------------------------
void vT_Create_Puzz (void *pv){

	uint32_t i,j,k=0, semente_PRNG=1;
	uint32_t puzzle[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

	//iniciar vetor com todas as figuras
	figuras[0]=&zero_f;
	figuras[1]=&um_f;
	figuras[2]=&dois_f;
	figuras[3]=&tres_f;
	figuras[4]=&quatro_f;
	figuras[5]=&cinco_f;
	figuras[6]=&seis_f;
	figuras[7]=&sete_f;
	figuras[8]=&oito_f;
	figuras[9]=&nove_f;
	figuras[10]=&dez_f;
	figuras[11]=&onze_f;
	figuras[12]=&doze_f;
	figuras[13]=&treze_f;
	figuras[14]=&catorze_f;
	figuras[15]=&quinze_f;

	vTaskDelay(300);

	limpa_LCD();
	//print tela inicial
	goto_XY(0,0);
	escreve2fb((unsigned char *)start_f);
	imprime_LCD();


	//cria tarefa da música
	SoundSHandle=NULL;
	xTaskCreate(vT_SoundS, "SoundS", 100, NULL, 4, &SoundSHandle);
	i=1;
	xQueueSendToBack (xSound,&i,500);
	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)){ // enquando nao pressionar joystick fica travado

			semente_PRNG++;		// semente para o gerador de números pseudoaleatorios
			//vTaskDelay(10);

	}
	init_LFSR(semente_PRNG);	// inicializacao para geracao de numeros pseudoaleatorios
	i=0;
	xQueueSendToBack (xSound,&i,osWaitForever);

	/*
	for (i=15;i>0;i--){
		j = prng_LFSR() % (i+1); 	//random index
		k = puzzle[i];				//swap with random index
		puzzle[i]=puzzle[j];
		puzzle[j]=k;
	}
	*/

	for(i=0;i<16;i++){
		Puzzle[i]=puzzle[i];
	}

	Puzzle[0]=1;
	Puzzle[1]=0;

	//create taks
	 uint32_t vT_Create = pdTRUE;
	 vT_Create = xTaskCreate(vT_Joy, "Joy", 128, NULL, 1, &JoyHandle);
	 vT_Create = xTaskCreate(vT_Puzzle, "Puzzle", 128, NULL, 1, &PuzzleHandle);
	 vT_Create = xTaskCreate(vT_Transi, "Transi", 170, NULL, 2, &TransiHandle);
	 vT_Create = xTaskCreate(vT_Print, "Print", 150, NULL,1, &PrintHandle);
	 if (vT_Create == pdFALSE){
		//abort
		string_LCD("Erro ao criar tarefas");
		imprime_LCD();
		while(1);
	}

	//start timer
	counter=0;
	osTimerStart(Timer01Handle,1000);

	//delete task - somente executar uma vez
	vTaskDelete(NULL);
	while(1){
	}
}
//---------------------------------------------------------------------
// Read Command
void vT_Joy(void *pv){
	uint32_t command;
	while(1)
	{
		osSemaphoreWait(transiOkHandle,osWaitForever);
		vTaskDelay(100);
		if (valor_ADC[0]<MIN_JOY){
			command = DOWN;
		} else
		if (valor_ADC[0]>MAX_JOY){
			command = UP;
		} else
		if (valor_ADC[1]<MIN_JOY){
			command = RIGHT;
		} else
		if (valor_ADC[1]>MAX_JOY){
			command = LEFT;
		} else
			command = NO_COMM;

		xQueueSendToBack(xCommand,&command,osWaitForever);
	}
}
//---------------------------------------------------------------------
void vT_SoundS(void *pv){
	int i,j;

	for (i=0;i<33;i++){
		pinkpan_d[i]= (uint32_t)(1.12*(float)pinkpan_d[i]);
	}

	while(1)
	{
		xQueueReceive(xSound,&j,osWaitForever);
		if (j==1){
			//start
			for (i=0;i<33;i++){
					xQueueReceive(xSound,&j,1);
					if (j==0){
						change_note(&htim3,0);
						break;
					}
					change_note(&htim3,pinkpan_n[i]);
					vTaskDelay(pinkpan_t[i]);
					change_note(&htim3,0);
					vTaskDelay(pinkpan_d[i]+pinkpan_p[i]);
			}
			change_note(&htim3,0);
		}
		else if(j==2){
			//win
			for (i=0;i<6;i++){
				change_note(&htim3,end_n[i]);
				vTaskDelay(end_t[i]);
				change_note(&htim3,0);
				vTaskDelay(end_d[i]);
			}
			change_note(&htim3,0);
		} else if(j==3){
			//movimento inválido
			change_note(&htim3,160);
			vTaskDelay(100);
			change_note(&htim3,0);
		}
	}
}
//---------------------------------------------------------------------
void vT_Puzzle (void* pv){

	uint32_t i,cursor=0,command=0,k,j;


	for (i=0;i<16;i++){
		if (Puzzle[i]==0){
			cursor=i;
			break;
		}
	}

	while (1){

		xQueueReceive(xCommand,&command,osWaitForever);
		i=0;
		if (command==NO_COMM){
		}else
		if (command==LEFT){
			if (((cursor+1)%4)==0){
				//invalid mov
				j=3;
				xQueueSendToBack (xSound,&j,500);
			}else{
				//swap
				Puzzle[cursor]=Puzzle[cursor+1];
				Puzzle[++cursor]=0;
				i=1;
			}
		}else
		if (command==RIGHT){
			if ((cursor%4)==0){
				//invalid mov
				j=3;
				xQueueSendToBack (xSound,&j,500);
			}else{
				//swap
				Puzzle[cursor]=Puzzle[cursor-1];
				Puzzle[--cursor]=0;
				i=1;
			}
		} else
		if (command==UP){
			if (cursor>11){
				//invalid mov
				j=3;
				xQueueSendToBack (xSound,&j,500);
			} else {
				//swap
				Puzzle[cursor]=Puzzle[cursor+4];
				Puzzle[cursor+4]=0;
				cursor+=4;
				i=1;
			}
		} else
		if (command==DOWN){
			if (cursor<4){
				//invalid mov
				j=3;
				xQueueSendToBack (xSound,&j,500);
			}else{
				//swap
				Puzzle[cursor]=Puzzle[cursor-4];
				Puzzle[cursor-4]=0;
				cursor-=4;
				i=1;
			}
		}

		//testa se puzzle concluído 0,1,2,3...
		j=0;
		for (k=0;k<16;k++){
			if (Puzzle[k]!=k){
				j=0;
				break;
			}
			j=1;
		}
		//teste se concluído 1,2,3...
		if (j==0)
			for (k=1;k<16;k++){
				if (Puzzle[k-1]!=k){
					j=0;
					break;
				}
				j=1;
			}

		if (i==1){

			//mov válido, envia para transição
			xQueueSendToBack(xTransi,&command,osWaitForever);

			if (j==1){
				//fim de jogo
				osTimerStop(Timer01Handle);
				vTaskDelete(JoyHandle);
				//Restart Game
				xTaskCreate(vT_Restart,"Restart",150,NULL,5,NULL);
			}
		}else {
			//lê joy novamente
			osSemaphoreRelease(transiOkHandle);
		}
	}
}
//---------------------------------------------------------------------
void vT_Transi (void * pv){
	uint32_t i, command=0, figura,cursor=0, movm=0;

	struct pontos_t old, new, transi;
		old.x1=0; old.y1=0;
		old.x2=0; old.y2=0;
		old.x3=0; old.y3=0;

		new.x1=0; new.y1=0;
		new.x2=0; new.y2=0;
		new.x3=0; new.y3=0;

		transi.x1=0; transi.y1=0;
		transi.x2=0; transi.y2=0;
		transi.x3=0; transi.y3=0;

	//procura pontos cursor antigo
	for (i=0;i<16;i++){
		if (Puzzle[i]==0){
			cursor=i;
			break;
		}
		old.x1+=12;
		if ((i+1)%4==0){
			old.x1=0;
			old.y1+=12;
		}
	}

	osSemaphoreRelease(transiOkHandle);

	while (1){
		//só recebe movimentos válidos
		xQueueReceive(xTransi,&command,osWaitForever);
		goto_XY(55,1);
		string_LCD_Nr("",++movm,3);
		figura=Puzzle[cursor];
		if (command==LEFT){
			new.x1=old.x1+12;
			new.y1=old.y1;

			transi.x1=new.x1;
			transi.y1=new.y1;

			for (i=0;i<13;i++){
				desenha_fig(&new,figuras[0]);
				desenha_fig(&old,figuras[0]);
				desenha_fig(&transi,figuras[figura]);
				transi.x1--;
				osSemaphoreRelease(printLCDHandle);
				osSemaphoreWait(printOkHandle, osWaitForever);
			}
			cursor++;
		}else
		if (command==RIGHT){

			new.x1=old.x1-12;
			new.y1=old.y1;

			transi.x1=new.x1;
			transi.y1=new.y1;

			for (i=0;i<13;i++){
				desenha_fig(&new,figuras[0]);
				desenha_fig(&old,figuras[0]);
				desenha_fig(&transi,figuras[figura]);
				transi.x1++;
				osSemaphoreRelease(printLCDHandle);
				osSemaphoreWait(printOkHandle, osWaitForever);
			}

			cursor--;
		}else
		if (command==UP){

			new.x1=old.x1;
			new.y1=old.y1+12;

			transi.x1=new.x1;
			transi.y1=new.y1;

			for (i=0;i<13;i++){
				desenha_fig(&new,figuras[0]);
				desenha_fig(&old,figuras[0]);
				desenha_fig(&transi,figuras[figura]);
				transi.y1--;
				osSemaphoreRelease(printLCDHandle);
				osSemaphoreWait(printOkHandle, osWaitForever);
			}
			cursor+=4;
		}else
		if (command==DOWN){ //DOWN

			new.x1=old.x1;
			new.y1=old.y1-12;

			transi.x1=new.x1;
			transi.y1=new.y1;

			for (i=0;i<13;i++){
				desenha_fig(&new,figuras[0]);
				desenha_fig(&old,figuras[0]);
				desenha_fig(&transi,figuras[figura]);
				transi.y1++;
				osSemaphoreRelease(printLCDHandle);
				osSemaphoreWait(printOkHandle, osWaitForever);
			}
			cursor-=4;
		}
		osSemaphoreRelease(transiOkHandle);
		old.x1=new.x1;
		old.y1=new.y1;
	}

}
//---------------------------------------------------------------------
void vT_Print (void * pv){
	uint32_t i;

	//Desenha Puzzle
	limpa_LCD();
	struct pontos_t p;
	p.x1=0; p.y1=0;
	p.x2=0; p.y2=0;
	p.x3=0; p.y3=0;
	for (i=1; i<17; i++){
		desenha_fig(&p,figuras[Puzzle[i-1]]);
		p.x1+=12;
		if (i%4==0){
			p.y1+=12;
			p.x1=0;
		}
	}

	goto_XY(50,0);
	string_LCD("Movm:");

	goto_XY(50,2);
	string_LCD("Time:");

	goto_XY(75,3);
	string_LCD("s");
	imprime_LCD();

	while(1){
		osSemaphoreWait(printLCDHandle,osWaitForever);
		imprime_LCD();
		vTaskDelay(ANIM_DELAY);
		osSemaphoreRelease(printOkHandle);

	}

}
//---------------------------------------------------------------------
void vT_Restart (void * pv){
	uint32_t j;

	osSemaphoreWait(transiOkHandle,osWaitForever);
	j=2;
	xQueueSendToBack (xSound,&j,500);
	vTaskDelay(100);
	limpa_LCD();
	goto_XY(0,0);
	escreve2fb((unsigned char *)restart_f);
	imprime_LCD();

	while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)){ // enquando nao pressionar joystick fica travado
		vTaskDelay(10);
	}

	vTaskDelete(PuzzleHandle);
	vTaskDelete(TransiHandle);
	vTaskDelete(PrintHandle);
	vTaskDelete(SoundSHandle);


	xTaskCreate(vT_Create_Puzz,"Create",150, NULL, 4, NULL);

	vTaskDelete(NULL);


	while(1){

	}
}
//---------------------------------------------------------------------

/* USER CODE END 4 */


/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */
	goto_XY(55,3);
	string_LCD_Nr("",++counter,3);
	osSemaphoreRelease(printLCDHandle);
  /* USER CODE END Callback01 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
