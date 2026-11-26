/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "LoRa.h"
#include "lwgps.h"			// To be added later when config GPS Functionality
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROLE_TAG          1        /* 1 = tag board, 0 = seeker board */
#define GPS_RX_BUF_SIZE   1024U

#define MSG_WAKE   0x01
#define MSG_ACK    0x02
#define MSG_GPS    0x03
#define MY_TAG_ID  0x01
#define SYNC_WORD  0x12

#define TAG_SLEEP_MS      1000U
#define WAKE_PREAMBLE     6000U    /* symbols: ~6.1 s at SF7/125k, must be longer than TAG_SLEEP_MS */
#define TAG_ACK_WAIT_MS   10000U
#define SEEKER_LISTEN_MS  4000U
#define GPS_SEND_MS       1000U
#define TIMEOUT_RX		7000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/***************** GPS TESTING CODE BEGIN *********************/
static lwgps_t           hgps;                        /* lwgps handle */
static uint8_t           gps_dma_buf[GPS_RX_BUF_SIZE];
static volatile uint16_t gps_head = 0;                /* written in callback (where DMA has reached) */
static uint16_t          gps_tail = 0;                /* how far main loop has parsed */

/******************** GPS TESTING CODE END *******************/
char dbg[64];				// Added for printing debug

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void UART_Print(char *message);
static void gps_print(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_Print(char *message)
{
    HAL_UART_Transmit(&huart2,
                      (uint8_t*)message,
                      strlen(message),
                      HAL_MAX_DELAY);
}

static void lora_set_preamble(LoRa* l, uint16_t n)
{
    LoRa_write(l, 0x20, n >> 8);      // RegPreambleMsb
    LoRa_write(l, 0x21, n & 0xFF);    // RegPreambleLsb
}

/* 12-byte GPS packet: [type][id][seq][fix][lat e6 (4B)][lon e6 (4B)] */
static uint8_t gps_pack(uint8_t* p, uint8_t seq)
{
    int32_t lat = (int32_t)((double)hgps.latitude  * 1000000.0);
    int32_t lon = (int32_t)((double)hgps.longitude * 1000000.0);
    p[0] = MSG_GPS; p[1] = MY_TAG_ID; p[2] = seq; p[3] = hgps.is_valid ? 1 : 0;
    memcpy(&p[4], &lat, 4);
    memcpy(&p[8], &lon, 4);
    return 12;
}
/******************************************** GPS TESTING CODE END *****************************************/

/********************* GPS TESTING FUNCTION BEGIN **************************/

/* Feed whatever new bytes DMA has dropped into the buffer to lwgps */
static void gps_poll(void)
{
    uint16_t head = gps_head;           /* single read of the volatile */

    if (head == gps_tail) {
//    	UART_Print("Inside head == gps_tail");
        return;                         /* nothing new */
    }

//    /* DEBUG: echo raw GPS bytes to serial monitor */
//    if (head > gps_tail) {
////    	UART_Print("Inside head > gps_tail... should print data\r\n");
//        HAL_UART_Transmit(&huart2, &gps_dma_buf[gps_tail], head - gps_tail, 100);
//    } else {
////    	UART_Print("Inside head < gps_tail... should print data\r\n");
//
//        HAL_UART_Transmit(&huart2, &gps_dma_buf[gps_tail], GPS_RX_BUF_SIZE - gps_tail, 100);
//        if (head > 0) HAL_UART_Transmit(&huart2, &gps_dma_buf[0], head, 100);
//    }

    if (head > gps_tail) {              /* simple case, no wrap */
        lwgps_process(&hgps, &gps_dma_buf[gps_tail], head - gps_tail);
    } else {                            /* wrapped: tail->end, then 0->head */
        lwgps_process(&hgps, &gps_dma_buf[gps_tail], GPS_RX_BUF_SIZE - gps_tail);
        if (head > 0) {
            lwgps_process(&hgps, &gps_dma_buf[0], head);
        }
    }
    gps_tail = head % GPS_RX_BUF_SIZE;
}


/****************************** DEBUG CODE ****************************/
/* continuous RX for up to timeout_ms; returns bytes of the first valid packet, 0 if none */
static uint8_t  dbg_irq, dbg_stat;
static uint32_t dbg_first_ms;

static uint8_t rx_window(LoRa* l, uint8_t* buf, uint8_t len, uint32_t timeout_ms)
{
    uint8_t n = 0;
    dbg_irq = 0; dbg_stat = 0; dbg_first_ms = 0;
    LoRa_write(l, RegIrqFlagsMask, 0x00);
    LoRa_write(l, RegIrqFlags, 0xFF);
    LoRa_startReceiving(l);
    uint32_t t0 = HAL_GetTick();
    while (n == 0 && (HAL_GetTick() - t0) < timeout_ms) {
        gps_poll();
        uint8_t st = LoRa_read(l, 0x18);                  // RegModemStat
        if ((st & 0x01) && dbg_first_ms == 0) dbg_first_ms = HAL_GetTick() - t0 + 1;
        dbg_stat |= st;
        dbg_irq  |= LoRa_read(l, 0x12);                   // read before checkRx clears it
        n = LoRa_checkRx(l, buf, len);
    }
    LoRa_gotoMode(l, STNBY_MODE);
    return n;
}

/************************** DEBUG CODE ***********************************/

/* Print one line on USART2 */
static void gps_print(void)
{
    char msg[200];
    int  n;
//    int validity = hgps.is_valid;
//    snprintf(dbg, sizeof(dbg), "hgps.is_valid = %d\r\n", validity);
//    UART_Print(dbg);
    if (!hgps.is_valid) {
//    	UART_Print("Still inside IF statement\r\n");
        n = snprintf(msg, sizeof(msg),
                     "No fix yet | sats in view: %d | sats in use: %d\r\n",
                     (int)hgps.sats_in_view, (int)hgps.sats_in_use);
    } else {
//    	UART_Print("Inside Else statement... Should be working now\r\n");
        n = snprintf(msg, sizeof(msg), "Lat: %.6f  Lon: %.6f  Alt: %.1f m  Speed: %.1f km/h  " "Sats: %d  UTC: %02d:%02d:%02d\r\n",(double)hgps.latitude,(double)hgps.longitude,(double)hgps.altitude,(double)lwgps_to_speed(hgps.speed, LWGPS_SPEED_KPH),(int)hgps.sats_in_use,(int)hgps.hours, (int)hgps.minutes, (int)hgps.seconds);
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)n, 100);
}

/******************************************** GPS TESTING CODE END *****************************************/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  LoRa myLora;

  myLora = newLoRa();

  myLora.CS_pin = LORA_NSS_Pin;
  myLora.CS_port = LORA_NSS_GPIO_Port;
  myLora.reset_pin = LORA_RST_Pin;
  myLora.reset_port = LORA_RST_GPIO_Port;
  myLora.DIO0_pin = DIO0_Pin;
  myLora.DIO0_port = DIO0_GPIO_Port;
  myLora.hSPIx = &hspi1;

  uint16_t st = LoRa_init(&myLora);
  sprintf(dbg, "LoRa_Status: %d\r\n", st);
  UART_Print(dbg);
  if (st != 200) { UART_Print("LoRa init failed, stopping\r\n"); while (1); }

  LoRa_write(&myLora, 0x39, SYNC_WORD);   // RegSyncWord, set after init, same on both boards

  uint8_t rx[16];
  uint8_t n;

//  uint16_t LoRa_status = LoRa_init(&myLora);
//
//  char buffer[50];
//
//  uint16_t value = 123;
//
//  sprintf(buffer, "Value = %d\r\n", value);
//
//  UART_Print(buffer);
//
//  sprintf(buffer, "LoRa_Status: %d\r\n", LoRa_status);
//
//  UART_Print(buffer);
//
//  if(LoRa_status != LORA_OK){
//  		//This is a basic version of Cad_Mode that runs it in an infinite loop
//	  UART_Print("ERROR: The LoRa Module is not able to initialize !\r\n");
//
//  	}

  /************************** GPS TESTING CODE BEGIN ******************************/
  lwgps_init(&hgps);

  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gps_dma_buf, GPS_RX_BUF_SIZE) != HAL_OK) {
      const char *e = "DMA start FAILED\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)e, strlen(e), 100);
  }

  const char *hello = "GPS demo started, waiting for NMEA data...\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)hello, strlen(hello), 100);

  uint32_t last_print = HAL_GetTick();

  /************************ GPS TESTING CODE END *******************************/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//	  sleep_5s();
//
//	  int8_t cad = LoRa_startcadmode(&myLora, 100);
//
//	  if (cad == 1) {
//		  // Look for syncword using rxsingle mode.
//	  }
//	  else if (cad == -1) {
//		  UART_Print("CAD timeout, check SPI/wiring\r\n");
//	  }
//	      // cad == 0: nothing heard, loop back to sleep
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//  }
  	while (1)
  	{
#if ROLE_TAG
			UART_Print("[TAG] sleep\r\n");
			LoRa_gotoMode(&myLora, SLEEP_MODE);
			uint32_t t0 = HAL_GetTick(); // instant value
			while ((HAL_GetTick() - t0 ) < TAG_SLEEP_MS) gps_poll();   // placeholder for RTC + Stop mode later

			//after 1sec we go into CAD Mode
			int8_t cad = LoRa_startcadmode(&myLora, 200);
			//200ms max >

			if (cad != 1) continue;                                  // clear: back to sleep
			UART_Print("[TAG] CAD detected, checking sync word\r\n");

			n = rx_window(&myLora, rx, sizeof(rx), TIMEOUT_RX);			// DEBUG CODE //received data in rx from RxBuffer with n as size of data

			sprintf(dbg, "[TAG] n=%d irq=0x%02X stat=0x%02X first=%lums\r\n",
			        n, dbg_irq, dbg_stat, (unsigned long)dbg_first_ms);
			UART_Print(dbg);


			snprintf(dbg, sizeof(dbg), "Datalength ==> %d | Rx values--> rx[0]: %d rx[1]: %d rx[2]: %d \r\n", n, rx[0],rx[1],rx[2]);
			UART_Print(dbg);


//			n = LoRa_receive(&myLora, rx, sizeof(rx), 7000);
			if (!(n >= 3 && rx[0] == MSG_WAKE && rx[1] == MY_TAG_ID)) {
				UART_Print("[TAG] no valid WAKE, back to sleep\r\n");
				continue;
			}

			uint8_t seq = rx[2];
			while (1) {
				HAL_Delay(20);                                       // let the seeker switch TX to RX
				uint8_t ack[3] = { MSG_ACK, MY_TAG_ID, seq };
				LoRa_transmit(&myLora, ack, 3, 500);
				UART_Print("[TAG] ACK sent, waiting\r\n");
/********************* DEBUG CODE *********************/
				uint8_t woke = 0;
				t0 = HAL_GetTick();
				while (!woke && (HAL_GetTick() - t0) < TAG_ACK_WAIT_MS) {
				    n = rx_window(&myLora, rx, sizeof(rx), TAG_ACK_WAIT_MS - (HAL_GetTick() - t0));
				    if (n >= 3 && rx[0] == MSG_WAKE && rx[1] == MY_TAG_ID) { seq = rx[2]; woke = 1; }
				}
/********************* DEBUG CODE *******************/

//				uint8_t woke = 0;
//				t0 = HAL_GetTick();
//				while (HAL_GetTick() - t0 < TAG_ACK_WAIT_MS) {
//					n = LoRa_receive(&myLora, rx, sizeof(rx), 7000);
//					if (n >= 3 && rx[0] == MSG_WAKE && rx[1] == MY_TAG_ID) { seq = rx[2]; woke = 1; break; }
//				}
				if (!woke) break;                                    // silence: seeker got the ACK
				UART_Print("[TAG] WAKE again, resending ACK\r\n");
			}

			UART_Print("[TAG] lost mode\r\n");
			seq = 0;
			while (1) {
				uint8_t pkt[12];
				gps_poll();
				sprintf(dbg, "GPS valid=%d view=%d use=%d head=%u\r\n",
				        hgps.is_valid, hgps.sats_in_view, hgps.sats_in_use, gps_head);
				UART_Print(dbg);
				gps_print();
				gps_pack(pkt, seq++);
				LoRa_transmit(&myLora, pkt, 12, 500);
				t0 = HAL_GetTick();
				while (HAL_GetTick() - t0 < GPS_SEND_MS) gps_poll();
			}

#else   /* ---------------- SEEKER ---------------- */
			uint8_t found = 0;

			UART_Print("[SEEKER] boot listen\r\n");
			uint32_t t0 = HAL_GetTick();
			while (HAL_GetTick() - t0 < SEEKER_LISTEN_MS) {
				n = LoRa_receive(&myLora, rx, sizeof(rx), 500);
				if (n >= 12 && rx[0] == MSG_GPS && rx[1] == MY_TAG_ID) { found = 1; break; }   // tag already in lost mode
			}

			uint8_t seq = 0;
			while (!found) {
				uint8_t wake[3] = { MSG_WAKE, MY_TAG_ID, ++seq };
				UART_Print("[SEEKER] TX WAKE (long preamble)\r\n");
				lora_set_preamble(&myLora, WAKE_PREAMBLE);
				LoRa_transmit(&myLora, wake, 3, 8000);
				lora_set_preamble(&myLora, 8);

				n = LoRa_receive(&myLora, rx, sizeof(rx), 1000);
				if (n >= 3 && rx[0] == MSG_ACK && rx[1] == MY_TAG_ID) {
					UART_Print("[SEEKER] ACK received\r\n");
					found = 1;
				} else {
					UART_Print("[SEEKER] no ACK, retrying\r\n");
				}
			}

			UART_Print("[SEEKER] lost mode\r\n");
			LoRa_startReceiving(&myLora);                            // continuous RX
			while (1) {
				gps_poll();
				n = LoRa_checkRx(&myLora, rx, sizeof(rx));
				if (n >= 12 && rx[0] == MSG_GPS && rx[1] == MY_TAG_ID) {
					int32_t lat, lon;
					char msg[96];
					memcpy(&lat, &rx[4], 4);
					memcpy(&lon, &rx[8], 4);
					sprintf(msg, "Tag seq=%d fix=%d lat=%ld lon=%ld rssi=%d\r\n",
							rx[2], rx[3], (long)lat, (long)lon, LoRa_getRSSI(&myLora));
					UART_Print(msg);
				}
			3}
#endif
  	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//void HAL_GPIO_EXTI_Callback(uint16_t pin)
//{
//    if (pin == DIO1_Pin) cad_detected = 1;   // flag only, no SPI in here
//}
//
//static void set_alarm_in(uint32_t secs)
//{
//    RTC_TimeTypeDef t; RTC_DateTypeDef d;
//    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
//    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);   // must be read after GetTime
//
//    uint32_t total = (t.Hours * 3600 + t.Minutes * 60 + t.Seconds + secs) % 86400;
//
//    RTC_AlarmTypeDef a = {0};
//    a.Alarm = RTC_ALARM_A;
//    a.AlarmTime.Hours   = total / 3600;
//    a.AlarmTime.Minutes = (total % 3600) / 60;
//    a.AlarmTime.Seconds = total % 60;
//    HAL_RTC_SetAlarm_IT(&hrtc, &a, RTC_FORMAT_BIN);
//}
//
//static void sleep_5s(void)
//{
//    rtc_flag = 0;
//    set_alarm_in(5);
//    LoRa_sleep(&myLora);
//
//    while (!rtc_flag) {                        // guards against spurious wakes
//        HAL_SuspendTick();
//        HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
//        SystemClock_Config();                  // HSE/PLL are off after Stop
//        HAL_ResumeTick();
//    }
//}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
