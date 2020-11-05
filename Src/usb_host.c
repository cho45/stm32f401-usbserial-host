/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v1.0_Cube
  * @brief           : This file implements the USB Host
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_cdc.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "main.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
USBH_HandleTypeDef hUsbHostFS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */
extern uint8_t usbRxBuffer[RX_BUFFER_SIZE];
extern uint8_t usbTxBuffer[TX_BUFFER_SIZE];
extern uint8_t uartRxBuffer[RX_BUFFER_SIZE];
extern uint8_t uartTxBuffer[TX_BUFFER_SIZE];

/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

void USBH_CDC_ReceiveCallback(USBH_HandleTypeDef * phost)
{
    uint16_t len = USBH_CDC_GetLastReceivedDataSize(&hUsbHostFS);
    /*
    uint8_t* ptr = usbRxBuffer;
    for(int i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    */

    if (len > 0) {
      while ( (HAL_UART_GetState(&huart2) & HAL_UART_STATE_BUSY_TX) == HAL_UART_STATE_BUSY_TX) {}
      memcpy(uartTxBuffer, usbRxBuffer, len);
      if (HAL_UART_Transmit_DMA(&huart2, uartTxBuffer, len) != HAL_OK) {
          printf("ng to transmit DMA %u\r\n", len);
      } else {
          // printf("ok to transmit DMA %u\r\n", len);
      }
    }

    USBH_CDC_Receive(&hUsbHostFS, usbRxBuffer, RX_BUFFER_SIZE);
}

/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */
  
  /* USER CODE END USB_HOST_Init_PreTreatment */
  
  /* Init host Library, add supported class and start the library. */
  if (USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_CDC_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_Start(&hUsbHostFS) != USBH_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */
  
  /* USER CODE END USB_HOST_Init_PostTreatment */
}

/*
 * Background task
 */
void MX_USB_HOST_Process(void)
{
  /* USB Host Background task */
  USBH_Process(&hUsbHostFS);
}
/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
  /* USER CODE BEGIN CALL_BACK_1 */
    switch(id)
    {
        case HOST_USER_SELECT_CONFIGURATION:
            break;

        case HOST_USER_DISCONNECTION:
            Appli_state = APPLICATION_DISCONNECT;
            USBH_CDC_Stop(&hUsbHostFS);
            break;

        case HOST_USER_CLASS_ACTIVE:
            if (Appli_state != APPLICATION_READY) {
                Appli_state = APPLICATION_READY;
                USBH_CDC_Receive(&hUsbHostFS, usbRxBuffer, RX_BUFFER_SIZE);
            }
            break;

        case HOST_USER_CONNECTION:
            Appli_state = APPLICATION_START;
            break;

        default:
            break;
    }
  /* USER CODE END CALL_BACK_1 */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
