/**
  ******************************************************************************
  * File Name          : dma.c
  * Description        : This file provides code for the configuration
  *                      of all the requested memory to memory DMA transfers.
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
#include "dma.h"
#include "params.h"

/* USER CODE BEGIN 0 */
extern uint8_t raw_image[IMG_HEIGHT][IMG_WIDTH];
extern const unsigned char inBMP2[];
DMA_HandleTypeDef dma;
extern HAL_StatusTypeDef status;

uint8_t contagem[IMG_HEIGHT][IMG_WIDTH];

void initContagem(void);
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure DMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

  dma.Instance = DMA2_Stream1;
  dma.Init.Channel = DMA_CHANNEL_1;
  dma.Init.Direction = DMA_PERIPH_TO_MEMORY;//TESTE: COPIAR UM BUFFER PARA OUTRO LUGAR
  dma.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  dma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  dma.Init.MemBurst = DMA_MBURST_INC4;//????????? copiei de openmv
  dma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  dma.Init.MemInc  = DMA_MINC_ENABLE;
  dma.Init.Mode = DMA_CIRCULAR;//necessário para captura contínua
  dma.Init.PeriphBurst = DMA_PBURST_SINGLE;//??????? copiei de openmv
  dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dma.Init.PeriphInc = DMA_PINC_DISABLE;//TESTE: DMA_SxPAR FUNCIONA COMO FONTE
  dma.Init.Priority = DMA_PRIORITY_VERY_HIGH;

  status = HAL_DMA_Init(&dma);

  //initContagem();//gera uma imagem com um gradiente horizontal, mais clara a direita
  //HAL_DMA_Start_IT(&dma,(uint32_t)(inBMP2+0x436),(uint32_t)raw_image,IMG_WIDTH*IMG_HEIGHT/4);
  //HAL_DMA_Start_IT(&dma,(uint32_t)contagem,(uint32_t)raw_image,IMG_WIDTH*IMG_HEIGHT/4);
  //HAL_DMA_Start_IT(&dma,(uint32_t)DCMI->DR,(uint32_t)raw_image,IMG_WIDTH*IMG_HEIGHT/4);
}

/* USER CODE BEGIN 2 */

void initContagem(void) {
		for(int i=0; i<IMG_HEIGHT; i++){//linha
			for(int j=0; j<256; j++){//coluna
				contagem[i][j] = j;
			}
			for(int j=256; j<IMG_WIDTH; j++){//coluna
				contagem[i][j] = 255;
			}
		}
};

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
