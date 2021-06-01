/* This Driver is suitable for SX1276/7/8 Lora module
 * Author: Vinod Kumar from Vinod Embedded
 * Goto: vinodembedded.wordpress.com for detailed explanation of the 
 * lora driver
 */

#include "main.h"
#include "lora.h"
#include <string.h>
#include <stdio.h>

#define TX																		// Uncomment for Transmission
//#define RX																	// Uncomment for Reception

SPI_HandleTypeDef hspi1;        							// SPI1  structure variable
UART_HandleTypeDef huart1;      							// UART1 structure variable

																							// Function Declarations
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

lora_pins_t lora_pins;												// Structure variable for lora pins
lora_t lora;																	// Structure variable for lora

char msg[64];																	// character buffer

// Main Function
int main(void)
{
	char buf[20];
  HAL_Init();																	// HAL library Initialization
  SystemClock_Config();												// System clock Initialization
  MX_GPIO_Init();															// GPIO Pins Initialization
  MX_SPI1_Init();															// SPI Communication Initialization
  MX_USART1_UART_Init(); 											// UART1 Communication Initialization
	
	//lora_pins.dio0.port  = LORA_DIO0_PORT;       
	//lora_pins.dio0.pin   = LORA_DIO0_PIN;
	lora_pins.nss.port   = LORA_SS_PORT;				// NSS pin to which port is connected
	lora_pins.nss.pin    = LORA_SS_PIN;					// NSS pin to which pin is connected
	lora_pins.reset.port = LORA_RESET_PORT;			// RESET pin to which port is connected
	lora_pins.reset.pin  = LORA_RESET_PIN;			// RESET pin to which pin is connected
	lora_pins.spi  			 = &hspi1;
	
	lora.pin = &lora_pins;											
	lora.frequency = FREQ_433MHZ;								// 433MHZ Frequency 
	//lora.frequency = FREQ_865MHZ;								// 865MHZ Frequency
	//lora.frequency = FREQ_866MHZ;								// 866MHZ Frequency
	//lora.frequency = FREQ_867MHZ;								// 867MHZ Frequency
	
	sprintf(msg,"Configuring LoRa module\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
	
	while(lora_init(&lora)){										// Initialize the lora module
		sprintf(msg,"LoRa Failed\r\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
		HAL_Delay(1000);
	}
	sprintf(msg,"Done configuring LoRaModule\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
	
	#ifdef TX
	uint16_t count =0;
	sprintf(buf,"Vinod Embedded");
	#else
	uint8_t ret;
	#endif
  while (1)																		// Inifinite Loop
  {
		#ifdef TX
		lora_begin_packet(&lora);
		lora_tx(&lora, (uint8_t *)buf, strlen(buf));
		lora_end_packet(&lora);
		sprintf(msg,"Sending packet %d\r\n",count++);
		HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
		HAL_Delay(1000);
		#endif
		
		#ifdef RX
		ret = lora_prasePacket(&lora);
		if(ret){
			uint8_t i=0;
			while(lora_available(&lora)){
				buf[i] = lora_read(&lora);
				i++;
			}
			buf[i] = '\0';
			sprintf(msg,"%s\r\n",buf);
			HAL_UART_Transmit(&huart1,(uint8_t *)msg,strlen(msg),1000);
		}
		#endif
  }
}
// END OF Main

void SystemClock_Config(void)
{
	// Initialize the system clock for delay and all
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI1_Init(void)
{
	// Initialize SPI1 for lora communication
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
	// UART1 for Print statements in the serial terminal with baud rate of 9600
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	
	// Intialize GPIOB - B11 for reset pin of lora
  GPIO_InitStruct.Pin = LORA_RESET_PIN;   
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_RESET_PORT, &GPIO_InitStruct);
	
	// Initilize GPIOA - A4 for NSS pin of lora
	GPIO_InitStruct.Pin = LORA_SS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_SS_PORT, &GPIO_InitStruct);	
	
	// Initially make the NSS lora pin HIGH
	HAL_GPIO_WritePin(LORA_SS_PORT, LORA_SS_PIN, GPIO_PIN_SET);
}

void Error_Handler(void)
{
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{ 
}
#endif 

/*   End of File
 * Author  : Vinod Kumar
 * Website : vinodembedded.wordpress.com
 */
