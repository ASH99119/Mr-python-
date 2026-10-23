// 1. 禁用 MSVC 关于 inet_addr 和 sprintf 的安全警告 (必须放在最顶部)
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS

// 2. 网络头文件必须在 windows.h 之前包含，防止头文件定义冲突
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <windows.h>

// 3. ZLG 官方头文件
#include "zlgcan.h"

#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "zlgcan.lib")


#define PYTHON_IP "127.0.0.1"       // 你的 Python 脚本运行在本机
#define PYTHON_PORT 50009           // 与 Python udp 绑定的端口一致
#define MY_DEVICE_TYPE 31           // USBCAN-4E-U 的真实设备类型号

// 如果你的头文件没有 MAKE_CAN_ID 和 GET_ID 宏，这里提供标准后备定义
#ifndef MAKE_CAN_ID
#define MAKE_CAN_ID(id, eff, rtr, err) (id | (eff << 31) | (rtr << 30) | (err << 29))
#endif
#ifndef GET_ID
#define GET_ID(can_id) (can_id & 0x1FFFFFFF)
#endif


/**
 * @brief 向 Python 中枢发送 CAN 车速 (UDP 无连接极速推送)
 */
void Send_Speed_To_Python(unsigned char speed)
{
    // 1. 改为 UDP 套接字 (SOCK_DGRAM)
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock != INVALID_SOCKET)
    {
        struct sockaddr_in destAddr;
        memset(&destAddr, 0, sizeof(destAddr));
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PYTHON_PORT);
        destAddr.sin_addr.s_addr = inet_addr(PYTHON_IP);

        char buf[16];
        // 2. 必须只发纯数字字符串，例如 "60"，否则 Python 的 float() 会报错崩溃
        sprintf(buf, "%d", speed);

        // 3. UDP 使用 sendto 直接将数据砸向 Python，无需 connect
        sendto(sock, buf, (int)strlen(buf), 0, (struct sockaddr*)&destAddr, sizeof(destAddr));
        closesocket(sock);
    }
}
int main()
{
    printf("==================================================\n");
    printf("     ZLG 64位 HIL 车速采集与 MR 分发系统就绪      \n");
    printf("==================================================\n\n");

    /* 0. 初始化 Socket 网络库（全局只开一次） */
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        printf("[-] Winsock 初始化失败！\n");
        return -1;
    }

    /* 1. 打开设备 */
    printf("[*] 正在尝试连接 USBCAN-4E-U (类型: 31)...\n");
    DEVICE_HANDLE dev = ZCAN_OpenDevice(MY_DEVICE_TYPE, 0, 0);
    if (INVALID_DEVICE_HANDLE == dev)
    {
        printf("[-] 错误：设备打开失败！请检查USB连接或重插拔硬件。\n");
        WSACleanup();
        system("pause");
        return -1;
    }
    printf("[+] 设备打开成功！句柄: %p\n", dev);

    /* 2. 强制设置波特率（USBCAN-4E-U 必须在此处设置 500Kbps） */
    if (ZCAN_SetValue(dev, "0/baud_rate", "500000") != 1)
    {
        printf("[-] 设置波特率 500Kbps 失败\n");
        ZCAN_CloseDevice(dev);
        WSACleanup();
        return -1;
    }
    printf("[+] 波特率成功设置为 500Kbps\n");

    /* 3. 初始化CAN通道 (通道0) */
    ZCAN_CHANNEL_INIT_CONFIG cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.can_type = 0;              // 0 = 普通CAN
    cfg.can.acc_code = 0;          // 过滤验收码
    cfg.can.acc_mask = 0xFFFFFFFF; // 过滤屏蔽码（全接收）
    cfg.can.filter = 0;            // 双滤波
    cfg.can.mode = 0;              // 0 = 正常模式 (必须是正常模式才能发送请求)

    CHANNEL_HANDLE ch = ZCAN_InitCAN(dev, 0, &cfg);
    if (INVALID_CHANNEL_HANDLE == ch)
    {
        printf("[-] 通道 0 初始化失败\n");
        ZCAN_CloseDevice(dev);
        WSACleanup();
        return -1;
    }
    printf("[+] 通道 0 初始化成功\n");

    /* 4. 启动CAN */
    if (ZCAN_StartCAN(ch) != 1)
    {
        printf("[-] 启动通道失败\n");
        ZCAN_CloseDevice(dev);
        WSACleanup();
        return -1;
    }
    printf("[+] CAN通道已启动\n");

    /* 5. 配置硬件级“定时发送” OBD-II 车速请求报文 */
    ZCAN_AUTO_TRANSMIT_OBJ auto_can;
    memset(&auto_can, 0, sizeof(auto_can));
    auto_can.enable = 1;      // 使能定时发送
    auto_can.index = 0;       // 列表索引 0
    auto_can.interval = 100;  // 间隔 100ms (1秒问10次车速)

    // 构造标准的 OBD-II 请求帧 (ID: 0x7DF)
    auto_can.obj.frame.can_id = MAKE_CAN_ID(0x07DF, 0, 0, 0);
    auto_can.obj.frame.can_dlc = 8;
    auto_can.obj.transmit_type = 0;

    auto_can.obj.frame.data[0] = 0x02; // 长度: 2 bytes
    auto_can.obj.frame.data[1] = 0x01; // Mode 01
    auto_can.obj.frame.data[2] = 0x0D; // PID: 0x0D (车速)

    if (ZCAN_SetValue(dev, "0/auto_send", (const char*)&auto_can) == 1)
    {
        printf("[+] 硬件定时请求已启动 (ID: 0x7DF, 间隔: 100ms)\n");
    }
    else
    {
        printf("[-] 硬件定时请求设置失败，请检查固件版本\n");
    }

    printf("\n>>> 正在持续接收总线数据，请观察车速变化 (按 Ctrl+C 退出) <<<\n\n");

    /* 6. 循环接收并解析数据 */
    while (1)
    {
        int count = ZCAN_GetReceiveNum(ch, 0);
        if (count > 0)
        {
            ZCAN_Receive_Data recv[100];
            memset(recv, 0, sizeof(recv));

            int recvCount = ZCAN_Receive(ch, recv, 100, 50);

            for (int i = 0; i < recvCount; i++)
            {
                unsigned int id = GET_ID(recv[i].frame.can_id);

                // 检查是否为 ECU 响应 (ID: 0x7E8)
                if (id == 0x07E8)
                {
                    // 验证 Mode 01 (回复 0x41) 且 PID 为 0x0D (车速)
                    if (recv[i].frame.data[1] == 0x41 && recv[i].frame.data[2] == 0x0D)
                    {
                        unsigned char speed = recv[i].frame.data[3];
                        printf(">>> [CAN 车速]: %3d km/h | 正在 UDP 推流至 %s:%d <<<\n", speed, PYTHON_IP, PYTHON_PORT);

                        // 实时发送给 Python 中枢 (格式: 60)
                        Send_Speed_To_Python(speed);
                    }
                }
            }
        }
        Sleep(50); // 休息 50ms 防止占用过高 CPU
    }

    /* 7. 清理并关闭 */
    ZCAN_SetValue(dev, "0/clear_auto_send", "0");
    ZCAN_CloseDevice(dev);
    WSACleanup(); // 全局网络清理
    return 0;
}