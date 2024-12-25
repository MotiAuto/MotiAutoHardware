/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "robomaster_utils.h"
#include "can_utils.h"
#include "pid_utils.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>

CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

//　受信したエンコーダ値を格納する構造体
RoboMasterFeedBack fb;

// モーターに送信する電流値を格納する構造体
RoboMasterCmd tx_packet;

// シリアル受信時に文字列を格納するバッファ
uint8_t buffer[64];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);

//　プロトタイプ宣言
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

int main(void)
{
	// シリアル送信用の関数（ほぼ呪文）
	setbuf(stdout, NULL);

	// 「.ioc」ファイルによって生成された初期化関数たち。絶対消したらだめ。
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_CAN1_Init();

	// CANのスタート
	// hcan1という変数は「.ioc」ファイルにより生成されたCANの設定が反映された構造体
	HAL_CAN_Start(&hcan1);

	//　CANの受信コールバックを有効にする
	//　CAN_IT_RX_FIFO0_MSG_PENDING以外の設定まだ知らない。
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	// モーターの数だけPID構造体を初期化する
	// pidInitialize(比例ゲイン、積分ゲイン、微分ゲイン)
	PID pid_1 = pidInitialize(1.0, 0.0, 0.03);
	PID pid_2 = pidInitialize(1.0, 0.0, 0.03);
	PID pid_3 = pidInitialize(1.0, 0.0, 0.03);

	//　bufferの要素番号指定用
	int index = 0;

	while (1)
	{
		// HAL_OKは多分、関数の成功か否かだと思われる。
		//　一文字ずつバッファに書き込んでいく
		if (HAL_UART_Receive(&huart2, &buffer[index], 1, 1000) == HAL_OK) {
			// 読み込んだ値が開業ならモーター回す。
			if (buffer[index] == '\n') {
				buffer[index] = '\0';

				// １つ目の値を取得する
				// strtokが区切り文字を見つけると\0で置換し、文字列として終端させるが、NULLを引数とすることでその先を引き続き読み出している
				char *token = strtok((char *)buffer, ",");
				// atoiで文字列を整数に変換
				int16_t target_1 = atoi(token);

				//　文字列操作のエラーによる不正な指令値を遮断
				if(target_1 <= 2000 && target_1 >= 1000)
				{
					// +1500されて送られているため引く　＆　RPMにギア比をかける
					int16_t parse_target = (target_1 - 1500) * 19;

					// pidCompute(PID構造体ポインタ, ターゲット, 現在, delta time)
					// 返り値がそのまま指令電流値
					int16_t out_1 = pidCompute(&pid_1, parse_target, fb.rpm[0], 0.02);

					// setCurrent(モーターID, モーターのタイプ（最大電流値が変わるため必要）, 送信したい電流値, RoboMasterCmd構造体のポインタ);
					// RoboMasterCmdの指定したIDに送信したい電流値を登録する。
					setCurrent(1, ROBOMASTER_M3508, out_1, &tx_packet);
				}

				// 以下２つ目、３つ目と繰り返す
				token = strtok(NULL, ",");
				int16_t target_2 = atoi(token);
				if(target_2 <= 2000 && target_2 >= 1000)
				{
					int16_t parse_target = (target_2 - 1500) * 19;

					int16_t out_2 = pidCompute(&pid_2, parse_target, fb.rpm[1], 0.02);
					setCurrent(2, ROBOMASTER_M3508, out_2, &tx_packet);
				}

				token = strtok(NULL, ",");
				int16_t target_3 = atoi(token);
				if(target_3 <= 2000 && target_3 >= 1000)
				{
					int16_t parse_target = (target_3 - 1500) * 19;

					int16_t out_3 = pidCompute(&pid_3, parse_target, fb.rpm[2], 0.02);
					setCurrent(3, ROBOMASTER_M3508, out_3, &tx_packet);
				}

				// "can_utils.h"よりCAN_TX(ID、送信バッファ、CAN設定)
				//　ロボマスタのID１〜４は0x200に送る
				//　buf_1はID1~4の送信内容
				CAN_TX(0x200, tx_packet.buf_1, &hcan1);

				// ロボマスターデータシートより20ms間隔で制御する
				HAL_Delay(20);

				//　各エンコーダ値のうちRPMをシリアルで送信
				printf("%d, %d, %d \r\n", fb.rpm[0] , fb.rpm[1], fb.rpm[2]);

				index = 0;
			// 読み取った文字が開業じゃないなら次の文字を待つ
			} else {
				index++;
			}
		}
	}
}

// CAN受信コールバック関数。関数名は固定。引数も固定
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	// 受信した際に送信元の情報とかが入る
	CAN_RxHeaderTypeDef RxHeader;

	//　受信したCANのデーターを格納する配列
	uint8_t RxData[8];

	// HAL_OKならRxDataに情報が入る。
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData)== HAL_OK) {

		// モーターのIDが１番なら0x201が来るので関数で扱うため0x200を引くと1になる
		uint32_t id = RxHeader.StdId - 0x200;

		// parseRoboMasterFeedBack(モーターID、受信内容、RoboMasterFeedBack構造体ポインタ)
		//受信内容の中から各エンコーダ値を取り出す。
		parseRoboMasterFeedBack(id, RxData, &fb);
	}
}

// シリアル送信用関数をオーバーライド
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
  return len;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef filter;
   filter.FilterIdHigh         = 0x201 << 5;                  // フィルターID1
   filter.FilterIdLow          = 0x202 << 5;                  // フィルターID2
   filter.FilterMaskIdHigh     = 0x203 << 5;                  // フィルターID3
   filter.FilterMaskIdLow      = 0x204 << 5;    // フィルターID4
   filter.FilterScale          = CAN_FILTERSCALE_16BIT; // 16モード
   filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;      // FIFO0へ格納
   filter.FilterBank           = 0;
   filter.FilterMode           = CAN_FILTERMODE_IDLIST; // IDリストモード
   filter.SlaveStartFilterBank = 14;
   filter.FilterActivation     = ENABLE;

    HAL_CAN_ConfigFilter(&hcan1, &filter);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
