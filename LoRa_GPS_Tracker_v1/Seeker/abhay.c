#include "stm32f1xx_hal.h"
#include "LoRa.h"
#include "lwgps.h" // Lightweight GPS parser library
#include <string.h>
#include <stdio.h>

int main()
{
       uint32_t lastTxTick = 0;
       uint32_t currentTxTick = Hal_GetTick();
       ack = false;

       while (!(ack))
       {
              // run loRa.transmit function (from time 0 -> 5sec)
              // start sending out data for tag ( which is in CAD MODE for 5 secs )
              currentTxTick = Hal_GetTick();
              if (currentTxTick - lastTxTick >= 6000)
              {
                     
                     // now get into rx.mode after 6 secs
                     // configure a GPIO as EXTI for rising ( DIO0 pushsed Low -> High for pulse whenever a message is received )
                     // the interrupt handler function gets called....< void EXTI_ DIO0_Handler() >
                     lastTxTick = HAL_GetTick();
              }
       }
}


void EXTI_DIO0_Hanlder()
{
       ack = true; // so now the while loop for CAD mode (pairing mode) will not get triggered since they are paired
       // call another function  <GPS_Process() > to process ( descale the lat-long integers) the incoming data packets.
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
       if (huart->Instance == USART1)
       {
              lwgps_process(&gps, &rxByte, 1); // Re-arm UART interpt for the nxt byte
              HAL_UART_Receive_IT(&huart1, &rxByte, 1);
       }
}

void GPS_Process()
{ // this function has now both the data of receivced gps and its own - so we can caculate the distance between tag&seeker
       // Receiver unpacking logic:
       float lat = pkt.latScaled / 10000000.0f;
       float lon = pkt.lonScaled / 10000000.0f;

       if (gps.is_valid)
       {
              struct TrackerPacket pkt;
              pkt.packetId = packetCounter++;

              // Scale floats to int32 to save bytes and airtime
              pkt.latScaled = (int32_t)(gps.latitude * 10000000.0f);
              pkt.lonScaled = (int32_t)(gps.longitude * 10000000.0f);

              // Send packet over       LoRa P2P (1000ms timeout)
              uint8_t txSuccess = LoRa_transmit(&myLoRa, (uint8_t *)&pkt, sizeof(pkt), 1000);

              if (!txSuccess)
              {
                     // Handle transmission timeout or bus error
              }
       }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        lwgps_process(&gps, &rxByte, 1);
        // Re-arm UART interrupt for the next byte
        HAL_UART_Receive_IT(&huart1, &rxByte, 1);
    }
}