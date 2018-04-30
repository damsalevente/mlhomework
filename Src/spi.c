/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
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
#include "spi.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
uint8_t spiTxBuf[8],spiRxBuf[8];
uint8_t sensorData[12];
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();
  
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 interrupt Init */
    HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PB10     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();
  
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    /* SPI1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PB10     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
//spi2 functions
uint8_t spi2_read_register(uint8_t data)
{
	spiTxBuf[0] = data | 0x80;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,spiTxBuf,1,500);
	HAL_SPI_Receive(&hspi2,spiRxBuf,2,500);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
	return spiRxBuf[0];
}
void spi2_write_register(uint8_t reg,uint8_t data){
	//Write CTRL9_XL = 38h // Acc X, Y, Z axes enabled
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
	spiTxBuf[0]=reg;	// Accelerometer Control register 1 (10h)
	spiTxBuf[1]=data;
	HAL_SPI_Transmit(&hspi2,spiTxBuf,1,1000);
	HAL_SPI_Transmit(&hspi2,&spiTxBuf[1],1,1000);

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
	}
void spi2_startup()
{
	spi2_write_register(0x18,0x38);
	spi2_read_register(0x18);

	HAL_Delay(500);
	spi2_write_register(0x10,0x60);
	spi2_read_register(0x10);

	HAL_Delay(500);
	spi2_write_register(0x19,0x38);
	spi2_read_register(0x19);

	HAL_Delay(500);
	spi2_write_register(0x11,0x60);
	spi2_read_register(0x11);
}
void spi2_whoami(void)
{
	//spi2_read_register(0x0f);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);

	spiTxBuf[0]=0x0f | 0x80;
	spiTxBuf[1]=0;
	HAL_SPI_Transmit(&hspi2,spiTxBuf,1,500);
	HAL_SPI_Receive(&hspi2,spiRxBuf,1,500);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);

}

int spi2_read_status(void)
{
	spi2_read_register(0x1e);
	return ((spiRxBuf[0])  & 0x01);
}
void spi2_read_data()
{
	uint8_t reg = 0x22; //first reg to read (see spi motionsensor datasheet)
	for(int i = 0; i<11;i++){
		uint8_t data=spi2_read_register(reg);
		reg++;
		sensorData[i] = data;
	}


}

//spi1 functions
void spi1_read_register(uint8_t *addr)
{

	spiTxBuf[0]=0x0b;
	for(int i = 1;i<4;i++)
	{
		spiTxBuf[i]=addr[i-1];
	}
	HAL_GPIO_WritePin(GPIOA,spi1_nss_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,spiTxBuf,4,500);
	HAL_SPI_Receive(&hspi1,spiRxBuf,5,500);
	HAL_GPIO_WritePin(GPIOA,spi1_nss_Pin,GPIO_PIN_SET);
}
void spi1_write_register(uint8_t * addr,uint8_t *data,uint8_t size)
{
	HAL_GPIO_WritePin(GPIOA,spi1_nss_Pin,GPIO_PIN_RESET);
	spiTxBuf[0]=0x06; //write enable
	HAL_SPI_Transmit(&hspi1,spiTxBuf,1,500);
	spiTxBuf[0]=0x02;
	spiTxBuf[1]=addr[0];
	spiTxBuf[2]=addr[1];
	spiTxBuf[3]=addr[2];
	HAL_SPI_Transmit(&hspi1,spiTxBuf,4,500);
	HAL_SPI_Transmit(&hspi1,data,size,500);
	HAL_GPIO_WritePin(GPIOA,spi1_nss_Pin,GPIO_PIN_SET);


}
int spi1_check_ready()
{
	uint8_t data=0;
	HAL_GPIO_WritePin(GPIOA,spi1_nss_Pin,GPIO_PIN_RESET);
	spiTxBuf[0]=0x05;
	HAL_SPI_Transmit(&hspi1,spiTxBuf,1,500);
	HAL_SPI_Receive(&hspi1,data,1,500);
	HAL_GPIO_WritePin(GPIOA,spi1_nss_Pin,GPIO_PIN_SET);
	if(data & 0x01){
		return 0;
	}
	else{
		return 1;
	}
}
void spi1_chip_erase()
{
	HAL_GPIO_WritePin(GPIOA,spi1_nss_Pin,GPIO_PIN_RESET);
	spiTxBuf[0]=0x06;
	spiTxBuf[1]=0xc7;
	HAL_SPI_Transmit(&hspi1,spiTxBuf,2,500);
	HAL_GPIO_WritePin(GPIOA,spi1_nss_Pin,GPIO_PIN_SET);
}
void spi1_IDread()
{
	HAL_GPIO_WritePin(GPIOA,spi1_nss_Pin,GPIO_PIN_RESET);
	spiTxBuf[0]=0xab;
	HAL_SPI_Transmit(&hspi1,spiTxBuf,1,500);
	HAL_SPI_Receive(&hspi1,&spiRxBuf[0],1,500);
	HAL_SPI_Receive(&hspi1,&spiRxBuf[1],1,500);
	HAL_GPIO_WritePin(GPIOA,spi1_nss_Pin,GPIO_PIN_SET);
}
void spi1_whoami(void){
	HAL_GPIO_WritePin(GPIOA,spi1_nss_Pin,GPIO_PIN_RESET);
	spiTxBuf[0]=0x9f;
	HAL_SPI_Transmit(&hspi1,spiTxBuf,1,500);
	HAL_SPI_Receive(&hspi1,spiRxBuf,4,500);
	HAL_GPIO_WritePin(GPIOA,spi1_nss_Pin,GPIO_PIN_SET);
}

/*
 *1 Read STATUS
2 If XLDA = 0, then go to 1
3 Read OUTX_L_XL
4 Read OUTX_H_XL
5 Read OUTY_L_XL
6 Read OUTY_H_XL
7 Read OUTZ_L_XL
8 Read OUTZ_H_XL
9 Data processing
10 Go to 1
 */
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
