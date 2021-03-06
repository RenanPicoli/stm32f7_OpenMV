
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"

#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#include "jprocess.h"
#include "usb_device.h"
#include "ov7725.h"
#include "ov7725_regs.h"
#include "stm32f7xx_hal_dcmi.h"
#include <stdlib.h>
#include "kmeans.h"
#include "artp.h"

typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern volatile uint8_t play_status;//apenas para debug no loop infinito
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

#define I2C_TIMEOUT     (1000)
extern const uint8_t default_regs[][2];

extern unsigned char inBMP2[];

//double buffer for DMA destination/JPEG source
uint8_t raw_image0[IMG_HEIGHT][IMG_WIDTH] __attribute__ ((aligned (64),section(".dtcmram")));
uint8_t raw_image1[IMG_HEIGHT][IMG_WIDTH] __attribute__ ((aligned (64),section(".dtcmram")));
uint8_t* raw_image;//intended for jpeg compression
uint8_t* dma_buffer;//intended for DMA xfers

HAL_StatusTypeDef status;
#define NSAMPLES 20
int32_t samples[NSAMPLES];

uint16_t last_jpeg_frame_size = 0;
volatile uint8_t jpeg_encode_done = 0;//1 - encode stopped flag
volatile uint8_t jpeg_encode_enabled = 0;//1 - capture and encoding enabled

double circle_x = 0;
double circle_y = 0;
double angle = 0;
uint8_t color;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void OTG_FS_IRQHandler(void);
void sensor_config();
int camera_writeb(uint8_t slv_addr, uint8_t reg_addr, uint8_t reg_data);
void switch_dma_jpeg_buffers(void);
int** filter(uint8_t* raw_image,uint8_t threshold,int* points_count);
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);
void Fail_Handler(void);
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
  int i = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //SCB_CleanDCache_by_Addr((uint32_t*)inBMP2,320*240);//flush D cache for source memory address. Do I need to wait?
  //SCB_InvalidateDCache_by_Addr((uint32_t*)raw_image,320*240);//invalidate D cache for destin. memory addr. Need to wait?

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_DMA_Init();
  MX_I2C1_Init();

  //MX_DCMI_Init();

  /* USER CODE BEGIN 2 */

  //Configura��o dos LEDs
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_0 |GPIO_PIN_1 |GPIO_PIN_2 ;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  //Os LEDS dessa placa s�o ATIVADOS EM N�VEL BAIXO
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);//red off
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);//green off
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);//blue off

  MX_USB_DEVICE_Init();
  //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);//led azul ON para DEBUG

  //camera uses I2C1, PB8=SCL and PB9=SDA
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);//camera resets
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);//camera NOT in powerdown

  HAL_Delay(400);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);//camera restarts
  HAL_Delay(40);

  uint8_t j=0;
  for(j=0x40;j<0x44;j+=2){
	if(HAL_I2C_IsDeviceReady(&hi2c1,j,10, I2C_TIMEOUT)==HAL_OK){
	  //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);//led green on if camera is present
	  break;
	}
  }
  //HAL_Delay(100);

  //test if camera is present
  uint8_t PID_REG = PID;
  uint8_t pid = 0;

  //2-phase write
  status = HAL_I2C_Master_Transmit(&hi2c1,OV7725_I2C_ADDRESS,&PID_REG,1,I2C_TIMEOUT);//qual a unidade do timeout? ms

  //2-phase read
  status = HAL_I2C_Master_Receive(&hi2c1,OV7725_I2C_ADDRESS,&pid,1,I2C_TIMEOUT);

  //testa se obteve a resposta esperada
  if(pid == OV7725_ID){
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);//led green on if camera is present
  }

  //CONFIGURAR AO MENOS COM7 E COM10 NA C�MERA
  sensor_config();//configura os registradores da c�mera
  MX_DMA_Init();
  MX_DCMI_Init();

  __HAL_RCC_TIM6_CLK_ENABLE();
  TIM_HandleTypeDef htim;
  htim.Instance	= TIM6;
  htim.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim.Init.Prescaler = 54;//TIM6CLK=2xAPB1CLK=54MHz, cada tick � 1us
  //htim.Init.ClockDivision = ;
  htim.Init.Period = 0xFFFF;//conta at� 65535 e d� o reload
  htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim);//configura o timer

  jpeg_encode_done = 1;
  raw_image = (uint8_t*)raw_image0;
  dma_buffer= (uint8_t*)raw_image1;
  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)dma_buffer, IMG_WIDTH*IMG_HEIGHT/4);//size=320*240/4

  int** points;//pontos que ser�o passados ao k-means (argumento data)
  uint8_t threshold=0xFF; //limiar inferior (m�nimo) para o ponto passar pelo filtro
  int points_stored = 0;//n�mero de pontos claros detectados

  int k=1;//k: k do K-Means
  float ** centroids=(float **)calloc(k,sizeof(float *));

  for(int i=0;i<k;i++){
	  centroids[i]=(float*)calloc(2,sizeof(float));
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //i++;
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,(GPIO_PinState)(!(play_status==2)));//red led is on when running
	  //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,(GPIO_PinState)(!(status!=HAL_OK)));//red green ON if something is NOT OK
	  //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,(GPIO_PinState)(!(DCMI->SR & DCMI_SR_FNE)));//blue led is on when running

	  if((hdcmi.DMA_Handle->State == HAL_DMA_STATE_READY) && (jpeg_encode_done==1)){//DMA ocioso e compress�o terminada
		  jpeg_encode_done = 0;
		  switch_dma_jpeg_buffers();
		  jpeg_encode_enabled = 1;
		  HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)dma_buffer, IMG_WIDTH*IMG_HEIGHT/4);//size=320*240/4
		  points = filter(raw_image,threshold,&points_stored);//espa�o � alocado para points usando malloc
	  }

	  if (jpeg_encode_enabled == 1)
		{
		  //HAL_DCMI_Suspend(&hdcmi);//para evitar que novos frames recebidos atrapalhem a compress�o

		  jpeg_encode_enabled = 0;
		  //jpeg_encode_done = 0;

		  //TESTE: pinta de preto os pontos claros detectados
		  /*for(int i=0;i<points_stored;i++){
			  raw_image[IMG_WIDTH*points[i][0]+points[i][1]]=0x00;
		  }*/

		  if(points_stored){
			  if(k_means(points,points_stored,2,k,0.001,centroids)==K_MEANS_OK){
				  //draw black point at centroid position
				  /*for(int i=0;i<k;i++){
					  raw_image[IMG_WIDTH*(int)(centroids[i][0])+(int)(centroids[i][1])]=0x00;
				  }*/
				  //draw QR code, its center is the cluster's centroid
				  for(int i=0;i<k;i++){
					  draw_QRcode(raw_image,(int)centroids[i][1],(int)centroids[i][0],IMG_WIDTH,IMG_HEIGHT);
					  //draw_QRcode(raw_image,IMG_WIDTH/2,IMG_HEIGHT/2,IMG_WIDTH,IMG_HEIGHT);
				  }
			  }
		  }

		  //come�a a contar o tempo para codifica��o de um frame
		  HAL_TIM_Base_Start(&htim);
		  int32_t microsegundos = htim.Instance->CNT;

		  last_jpeg_frame_size = jprocess();//Data source (image) for jpeg encoder can be switched in "jprocess" function.
		  microsegundos = htim.Instance->CNT - microsegundos;//pausa a contagem de tempo
		  HAL_TIM_Base_Stop(&htim);

		  samples[i%NSAMPLES] = microsegundos;//grava a nova amostra de tempo
		  i++;

		  int point_index=0;
		  while(point_index < points_stored)
		  {
			  free(points[point_index]);
			  point_index++;
		  }
		  free(points);//usamos points, precisamos liberar a mem�ria

		  jpeg_encode_done = 1;//encoding ended
		  HAL_Delay(1);//p�r menos do que isso sobrecarrega o OTG FS (max 1,5MB/s)
		  //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)dma_buffer, IMG_WIDTH*IMG_HEIGHT/4);//size=320*240/4
		  //hdcmi.DMA_Handle->Instance->CR |= DMA_SxCR_EN;//Enables DMA again (disabled in DMA2_IRQ)
		  //HAL_DCMI_Resume(&hdcmi);
		  //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_2);//Toggles blue led
		}

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/**
* @brief This function handles USB On The Go FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

void sensor_config()
{
    int i=0;
    const uint8_t (*regs)[2];

    // Reset all registers
    camera_writeb(OV7725_I2C_ADDRESS, COM7, COM7_RESET);

    // Delay 10 ms
    HAL_Delay(10);

    // Write default registers
    for (i=0, regs = default_regs; regs[i][0]; i++) {
        camera_writeb(OV7725_I2C_ADDRESS, regs[i][0], regs[i][1]);
    }

    // Delay
    HAL_Delay(30);

    return;
}

int camera_writeb(uint8_t slv_addr, uint8_t reg_addr, uint8_t reg_data)
{
    int ret=0;
    uint8_t buf[] = {reg_addr, reg_data};

    __disable_irq();
    if(HAL_I2C_Master_Transmit(&hi2c1, slv_addr, buf, 2, I2C_TIMEOUT) != HAL_OK) {
        ret = -1;
    }
    __enable_irq();
    return ret;
}

void switch_dma_jpeg_buffers(void){
	if(raw_image == (uint8_t*)raw_image0){
		raw_image = (uint8_t*)raw_image1;
		dma_buffer = (uint8_t*)raw_image0;
	}else{
		raw_image = (uint8_t*)raw_image0;
		dma_buffer = (uint8_t*)raw_image1;
	}
	return;
}

int** filter(uint8_t* image,uint8_t threshold,int* points_count){
	#define INITIAL_CAPACITY 10
	//abaixo est� definido o n�mero m�ximo de vezes que podemos dobrar o espa�o alocado
	#define MAX_RESIZES 4

	int capacity = INITIAL_CAPACITY;//capacity: n�mero de pontos de que podem ser armazenados
	int** ptr=calloc(capacity,sizeof(int*));

	int stored=0;//stored: n�mero de pontos EFETIVAMENTE armazenados
	int resizes=0;//resizes: n�mero de vezes que precisamos aumentar a capacidade do vetor
	int saturated_inc=(1<<MAX_RESIZES)*INITIAL_CAPACITY;//saturated_inc: tamanho dos incrementos de tamanho de ptr ap�s este "saturar"
	for(int i=0;i<IMG_HEIGHT;i++){//i:linha da img menor
		for(int j=0;j<IMG_WIDTH;j++){//j:coluna da img menor
			if(image[IMG_WIDTH*i+j] >= threshold){
				if(stored == capacity){//verifica se precisa de mais espa�o
					if(resizes <= MAX_RESIZES){//verifica se pode dobrar o espa�o
						int** new_ptr = realloc(ptr,2*capacity*sizeof(int*));

						if(new_ptr!=NULL)
							ptr=new_ptr;
						else
							return ptr;//se malloc falhar, retornamos o que conseguimos

						ptr[stored]=malloc(2*sizeof(int));
						ptr[stored][0]=i;//y, cf kmeans
						ptr[stored][1]=j;//x, cf kmeans
						stored++;
						capacity *= 2;//dobra capacity
						resizes++;
					}else{//a partir de agora, aumenta "linearmente" o armazenamento, o incremento "satura"
						int** new_ptr = realloc(ptr,(capacity+saturated_inc)*sizeof(int*));

						if(new_ptr!=NULL)
							ptr=new_ptr;
						else
							return ptr;//se malloc falhar, retornamos o que conseguimos

						ptr[stored]=malloc(2*sizeof(int));
						ptr[stored][0]=i;//y, cf kmeans
						ptr[stored][1]=j;//x, cf kmeans
						stored++;
						capacity += saturated_inc;//atualiza capacity
						resizes++;
					}
				}else{//se n�o precisa de mais espa�o, apenas aloca o novo ponto
					ptr[stored]=malloc(2*sizeof(int));
					ptr[stored][0]=i;//y, cf kmeans
					ptr[stored][1]=j;//x, cf kmeans
					stored++;
				}
			}
		}
	}
	if(stored < capacity && stored > 0){//libera espa�o que n�o seja necess�rio
		ptr = realloc(ptr,stored*sizeof(int*));
	}
	*points_count=stored;
	return ptr;
}

void delay_ms(uint32_t ms)
{
        volatile uint32_t nCount;
        //RCC_ClocksTypeDef RCC_Clocks;
        //RCC_GetClocksFreq (&RCC_Clocks);
        //nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
        nCount=(HAL_RCC_GetHCLKFreq()/10000)*ms;
        for (; nCount!=0; nCount--);
}
/* USER CODE END 4 */

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
