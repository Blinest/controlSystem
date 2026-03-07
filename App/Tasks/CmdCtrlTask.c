/**
*  @file CmdCtrlTask.c
 * @brief 指令控制任务：从队列取串口数据，交给 cmd_parse 解析并执行
 * @author blin
 *
 * 串口2 接收字节经 CmdCtrlQueueHandle 送入本任务，每字节调用 cmd_parse_feed_byte()，
 * 指令解析与电机/传感器控制逻辑在 Common/cmd_parse 中实现。
 */

#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "usart.h"
#include "Common/cmd_parse.h"
void StartCmdCtrlTask(void *argument)
{
    // 测试串口用
    // uint8_t test_msg[] = "send to usart1\r\n";
    uint8_t receive;

    for (;;)
    {
        // 取出从串口中断2中得到的数据
        if (osMessageQueueGet(CmdCtrlQueueHandle, &receive, 0, osWaitForever) == osOK)
        {
            Usart_SendString(&huart1, &receive, 1);
            // 进入指令解析函数
        	cmd_parse_feed_byte(receive);
        }

    }
}