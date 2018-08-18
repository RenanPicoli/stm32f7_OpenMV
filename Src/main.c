
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
uint8_t raw_image0[IMG_HEIGHT][IMG_WIDTH] __attribute__ ((aligned (64))) ;
uint8_t raw_image1[IMG_HEIGHT][IMG_WIDTH] __attribute__ ((aligned (64))) ;
uint8_t* raw_image;

HAL_StatusTypeDef status;

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
extern void DCMI_DMAXferCplt(DMA_HandleTypeDef *hdma);
extern void DCMI_DMAError(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef DCMI_Start_DMA_DoubleBuffer(DCMI_HandleTypeDef* hdcmi, uint32_t DCMI_Mode, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t Length);
//void draw_circle(int Hcenter, int Vcenter, int radius,uint8_t color);
void sensor_config();
int camera_writeb(uint8_t slv_addr, uint8_t reg_addr, uint8_t reg_data);
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

  //Configuração dos LEDs
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_0 |GPIO_PIN_1 |GPIO_PIN_2 ;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  //Os LEDS dessa placa são ATIVADOS EM NÍVEL BAIXO
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

  //CONFIGURAR AO MENOS COM7 E COM10 NA CÂMERA
  sensor_config();//configura os registradores da câmera
  MX_DMA_Init();
  MX_DCMI_Init();

  //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)raw_image, 0x9600);//size=320*240*2/4

  hdcmi.DMA_Handle->Instance->CR |= DMA_SxCR_DBM;//habilita o modo double buffer
  //hdcmi.DMA_Handle->Instance->M1AR = (uint32_t)raw_image1;//a linha abaixo inicializa MA0 mas não inicializa MA1
  //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)raw_image0, IMG_WIDTH*IMG_HEIGHT/4);//size=320*240/4
  DCMI_Start_DMA_DoubleBuffer(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)raw_image0,(uint32_t)raw_image1, IMG_WIDTH*IMG_HEIGHT/4);//size=320*240/4

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  i++;
	  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,(GPIO_PinState)(!(play_status==2)));//red led is on when running
	  //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,(GPIO_PinState)(!(status!=HAL_OK)));//red green ON if something is NOT OK

	  if (jpeg_encode_enabled == 1)
		{
		  //HAL_DCMI_Suspend(&hdcmi);
		  jpeg_encode_enabled = 0;
		  jpeg_encode_done = 0;

		  last_jpeg_frame_size = jprocess();//Data source (image) for jpeg encoder can be switched in "jprocess" function.

/*
		  circle_x = 160 + sin(angle)*60;
		  circle_y = 120 + cos(angle)*60;
		  angle+= 0.05;
		  color+= 10;
		  draw_circle((int)circle_x, (int)circle_y, 15, color);
*/

		  jpeg_encode_done = 1;//encoding ended
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

/**
  * @brief  Enables DCMI DMA request and enables DCMI capture
  * @param  hdcmi     		pointer to a DCMI_HandleTypeDef structure that contains
  *                    		the configuration information for DCMI.
  * @param  DCMI_Mode 		DCMI capture mode snapshot or continuous grab.
  * @param  DstAddress     	The destination memory Buffer address (LCD Frame buffer).
  * @param SecondMemAddress	The second destination memory Buffer address (LCD Frame buffer).
  * @param  Length    		The length of capture to be transferred.
  * @retval HAL status
  */
HAL_StatusTypeDef DCMI_Start_DMA_DoubleBuffer(DCMI_HandleTypeDef* hdcmi, uint32_t DCMI_Mode, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t Length)
{
  /* Check function parameters */
  assert_param(IS_DCMI_CAPTURE_MODE(DCMI_Mode));

  /* Process Locked */
  __HAL_LOCK(hdcmi);

  /* Lock the DCMI peripheral state */
  hdcmi->State = HAL_DCMI_STATE_BUSY;

  /* Enable DCMI by setting DCMIEN bit */
  __HAL_DCMI_ENABLE(hdcmi);

  /* Configure the DCMI Mode */
  hdcmi->Instance->CR &= ~(DCMI_CR_CM);
  hdcmi->Instance->CR |=  (uint32_t)(DCMI_Mode);

  /* Set the DMA memory0 conversion complete callback */
  hdcmi->DMA_Handle->XferCpltCallback = DCMI_DMAXferCplt;

  /* Set the DMA error callback */
  hdcmi->DMA_Handle->XferErrorCallback = DCMI_DMAError;

  /* Set the dma abort callback */
  hdcmi->DMA_Handle->XferAbortCallback = NULL;

  /* Reset transfer counters value */
  hdcmi->XferCount = 0;
  hdcmi->XferTransferNumber = 0;

  if(Length <= 0xFFFF)
  {
    /* Enable the DMA Stream */
    //HAL_DMA_Start_IT()
    /* calculate DMA base and stream number */
    DMA_Base_Registers *regs = (DMA_Base_Registers *)hdcmi->DMA_Handle->StreamBaseAddress;

    /* Check the parameters */
    assert_param(IS_DMA_BUFFER_SIZE(Length));

    /* Process locked */
    __HAL_LOCK(hdcmi->DMA_Handle);

    if(HAL_DMA_STATE_READY == hdcmi->DMA_Handle->State)
    {
      /* Change DMA peripheral state */
      hdcmi->DMA_Handle->State = HAL_DMA_STATE_BUSY;

      /* Initialize the error code */
      hdcmi->DMA_Handle->ErrorCode = HAL_DMA_ERROR_NONE;

      /* Configure the source, destination addresses and the data length */
      /* Configure DMA Stream data length */
      hdcmi->DMA_Handle->Instance->NDTR = Length;

      /* Configure DMA Stream source address */
      hdcmi->DMA_Handle->Instance->PAR = (uint32_t)&hdcmi->Instance->DR;

      /* Configure DMA Stream destination address */
      hdcmi->DMA_Handle->Instance->M0AR = DstAddress;

      /* Configure DMA Stream SECOND destination address */
      hdcmi->DMA_Handle->Instance->M1AR = SecondMemAddress;

      /* Clear all interrupt flags at correct offset within the register */
      regs->IFCR = 0x3FU << hdcmi->DMA_Handle->StreamIndex;

      /* Enable Common interrupts*/
      hdcmi->DMA_Handle->Instance->CR  |= DMA_IT_TC | DMA_IT_TE | DMA_IT_DME;
      hdcmi->DMA_Handle->Instance->FCR |= DMA_IT_FE;

      if(hdcmi->DMA_Handle->XferHalfCpltCallback != NULL)
      {
    	  hdcmi->DMA_Handle->Instance->CR  |= DMA_IT_HT;
      }

      /* Enable the Peripheral */
      __HAL_DMA_ENABLE(hdcmi->DMA_Handle);
    }
    else
    {
      /* Process unlocked */
      __HAL_UNLOCK(hdcmi->DMA_Handle);

      /* Return error status */
      status = HAL_BUSY;
    }

  }
  else /* DCMI_DOUBLE_BUFFER Mode */
  {//TODO
	  status = HAL_ERROR;
/*     Set the DMA memory1 conversion complete callback
    hdcmi->DMA_Handle->XferM1CpltCallback = DCMI_DMAXferCplt;

     Initialize transfer parameters
    hdcmi->XferCount = 1;
    hdcmi->XferSize = Length;
    hdcmi->pBuffPtr = pData;

     Get the number of buffer
    while(hdcmi->XferSize > 0xFFFF)
    {
      hdcmi->XferSize = (hdcmi->XferSize/2);
      hdcmi->XferCount = hdcmi->XferCount*2;
    }

     Update DCMI counter  and transfer number
    hdcmi->XferCount = (hdcmi->XferCount - 2);
    hdcmi->XferTransferNumber = hdcmi->XferCount;

     Update second memory address
    SecondMemAddress = (uint32_t)(pData + (4*hdcmi->XferSize));

     Start DMA multi buffer transfer
    HAL_DMAEx_MultiBufferStart_IT(hdcmi->DMA_Handle, (uint32_t)&hdcmi->Instance->DR, (uint32_t)pData, SecondMemAddress, hdcmi->XferSize);*/
  }

  /* Enable Capture */
  hdcmi->Instance->CR |= DCMI_CR_CAPTURE;

  /* Release Lock */
  __HAL_UNLOCK(hdcmi);

  /* Return function status */
  return HAL_OK;
}

/**
  * @brief  Enables DCMI DMA request and enables DCMI capture
  * @param  hdcmi      pointer to a DCMI_HandleTypeDef structure that contains
  *                     the configuration information for DCMI.
  * @param  DCMI_Mode  DCMI capture mode snapshot or continuous grab.
  * @param  DstAddress The memory0 Buffer address (destination buffer).
  * @param  SecondMemAddress The memory1 Buffer address (also destination buffer).
  * @param  Length     The length of capture to be transferred.
  * @retval HAL status
  */

/*
void draw_circle(int Hcenter, int Vcenter, int radius,uint8_t color)
{
  int x = radius;
  int y = 0;
  int xChange = 1 - (radius << 1);
  int yChange = 0;
  int radiusError = 0;
  int i;
  //int p = 3 - 2 * radius;

  while (x >= y)
  {
    for (i = Hcenter - x; i <= Hcenter + x; i++)
    {
      raw_image[Vcenter + y][i] = color;
      raw_image[Vcenter - y][i] = color;
    }
    for (i = Hcenter - y; i <= Hcenter + y; i++)
    {
      raw_image[Vcenter + x][i] = color;
      raw_image[Vcenter - x][i] = color;
    }

    y++;
    radiusError += yChange;
    yChange += 2;
    if (((radiusError << 1) + xChange) > 0)
    {
      x--;
      radiusError += xChange;
      xChange += 2;
    }
  }
}*/

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
