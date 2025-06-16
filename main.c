#include "main.h"
#include "ssd1306.h"
#include "fonts.h"
#include "string.h"


#define LED_RED_PIN     5
#define LED_YELLOW_PIN  6
#define LED_GREEN_PIN   7


I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
volatile uint8_t led_mode = 0; // 0: blink, 1: steady
volatile uint8_t led_state[3] = {0, 0, 0}; // 0: off, 1: on (RED, YELLOW, GREEN)
char last_command[8] = ""; // Biến để lưu câu lệnh cuối cùng


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_EXTI_Init(void);
void update_leds(void);
void delay(volatile uint32_t t);


void delay(volatile uint32_t t) {
  while(t--);
}
void update_leds(void) {
  // RED
  if (led_mode == 0) {
      // Blink mode handled in timer
  } else {
      if (led_state[0]) GPIOA->ODR |= (1 << LED_RED_PIN);
      else GPIOA->ODR &= ~(1 << LED_RED_PIN);
  }
  // YELLOW
  if (led_mode == 0) {
      // Blink mode handled in timer
  } else {
      if (led_state[1]) GPIOA->ODR |= (1 << LED_YELLOW_PIN);
      else GPIOA->ODR &= ~(1 << LED_YELLOW_PIN);
  }
  // GREEN
  if (led_mode == 0) {
      // Blink mode handled in timer
  } else {
      if (led_state[2]) GPIOA->ODR |= (1 << LED_GREEN_PIN);
      else GPIOA->ODR &= ~(1 << LED_GREEN_PIN);
  }
}
void USART2_IRQHandler(void) {
  if (USART2->SR & USART_SR_RXNE) {
      static char buf[8];
      static uint8_t idx = 0;
      char c = USART2->DR;
      if (c == '\r' || c == '\n') {
          buf[idx] = 0;
          if (strncmp(buf, "123R", 4) == 0) {
              led_state[0] = !led_state[0]; // RED
          } else if (strncmp(buf, "123Y", 4) == 0) {
              led_state[1] = !led_state[1]; // YELLOW
          } else if (strncmp(buf, "123B", 4) == 0) {
              led_state[2] = !led_state[2]; // GREEN
          }
          update_leds();
          strncpy(last_command, buf, 8); // Sao chép câu lệnh vào last_command
          last_command[7] = 0; // Đảm bảo null-terminated
          idx = 0;
      } else if (idx < 7) {
          buf[idx++] = c;
      }
  }
}
void TIM2_IRQHandler(void) {
  if (TIM2->SR & TIM_SR_UIF) {
      TIM2->SR &= ~TIM_SR_UIF;
      if (led_mode == 0) { // Blink mode
          static uint8_t blink = 0;
          blink = !blink;
          // RED
          if (led_state[0]) {
              if (blink) GPIOA->ODR |= (1 << LED_RED_PIN);
              else GPIOA->ODR &= ~(1 << LED_RED_PIN);
          } else {
              GPIOA->ODR &= ~(1 << LED_RED_PIN);
          }
          // YELLOW
          if (led_state[1]) {
              if (blink) GPIOA->ODR |= (1 << LED_YELLOW_PIN);
              else GPIOA->ODR &= ~(1 << LED_YELLOW_PIN);
          } else {
              GPIOA->ODR &= ~(1 << LED_YELLOW_PIN);
          }
          // GREEN
          if (led_state[2]) {
              if (blink) GPIOA->ODR |= (1 << LED_GREEN_PIN);
              else GPIOA->ODR &= ~(1 << LED_GREEN_PIN);
          } else {
              GPIOA->ODR &= ~(1 << LED_GREEN_PIN);
          }
      }
  }
}
void EXTI15_10_IRQHandler(void) {
  if (EXTI->PR & EXTI_PR_PR13) {
      EXTI->PR = EXTI_PR_PR13;
      led_mode = !led_mode;
      update_leds();
  }
}


int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_EXTI_Init();
  /* Initialize OLED display */
  SSD1306_Init();
  /* Infinite loop */
  while (1) {
      SSD1306_Clear();
      SSD1306_GotoXY(1, 0);
      SSD1306_Puts("LED Status:", &Font_7x10, 1);
      SSD1306_GotoXY(1, 10);
      SSD1306_Puts("Red: ", &Font_7x10, 1);
      SSD1306_Puts(led_state[0] ? "ON " : "OFF", &Font_7x10, 1);
      SSD1306_GotoXY(1, 20);
      SSD1306_Puts("Yellow: ", &Font_7x10, 1);
      SSD1306_Puts(led_state[1] ? "ON " : "OFF", &Font_7x10, 1);
      SSD1306_GotoXY(1, 30);
      SSD1306_Puts("Green: ", &Font_7x10, 1);
      SSD1306_Puts(led_state[2] ? "ON " : "OFF", &Font_7x10, 1);
      SSD1306_GotoXY(1, 40);
      SSD1306_Puts("Mode: ", &Font_7x10, 1);
      SSD1306_Puts(led_mode ? "Steady" : "Blink", &Font_7x10, 1);
      SSD1306_GotoXY(1, 50);
      SSD1306_Puts("Cmd: ", &Font_7x10, 1);
      SSD1306_Puts(last_command, &Font_7x10, 1);
      SSD1306_UpdateScreen();
      HAL_Delay(500);
  }
}


void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
      Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
      Error_Handler();
  }
}


static void MX_GPIO_Init(void) {
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  /* Configure LED pins (PA5, PA6, PA7) as output */
  GPIOA->MODER |= (1 << (LED_RED_PIN * 2)) | (1 << (LED_YELLOW_PIN * 2)) | (1 << (LED_GREEN_PIN * 2));
  /* Configure UART2 pins (PA2, PA3) as alternate function */
  GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2)));
  GPIOA->MODER |= (2 << (2 * 2)) | (2 << (3 * 2));
  GPIOA->AFR[0] |= (7 << (2 * 4)) | (7 << (3 * 4));
  /* Configure PC13 as input for EXTI */
  GPIOC->MODER &= ~(3 << (13 * 2));
}


static void MX_USART2_UART_Init(void) {
  __HAL_RCC_USART2_CLK_ENABLE();
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
      Error_Handler();
  }
  USART2->CR1 |= USART_CR1_RXNEIE;
  NVIC_EnableIRQ(USART2_IRQn);
}
/**
* @brief I2C1 Initialization Function
* @param None
* @retval None
*/
static void MX_I2C1_Init(void) {
  __HAL_RCC_I2C1_CLK_ENABLE();
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
      Error_Handler();
  }
}


static void MX_TIM2_Init(void) {
  __HAL_RCC_TIM2_CLK_ENABLE();
  TIM2->PSC = 16000 - 1;
  TIM2->ARR = 200 - 1;
  TIM2->DIER |= TIM_DIER_UIE;
  TIM2->CR1 |= TIM_CR1_CEN;
  NVIC_EnableIRQ(TIM2_IRQn);
}
static void MX_EXTI_Init(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
  EXTI->IMR |= EXTI_IMR_IM13;
  EXTI->FTSR |= EXTI_FTSR_TR13;
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}
void Error_Handler(void) {
  while (1) {
  }
}
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  while (1) {
  }
}
#endif 

