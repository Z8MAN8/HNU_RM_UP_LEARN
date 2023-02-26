# HNU_RMUL_UP_2022

#### 介绍
海南大学2022赛季联盟赛，重构的步兵云台代码

#### 软件架构

1. APP文件中为主要的任务函数
2. Transmission.c中主要包含云台和底盘以及上位机之间的通信
3. 云台和底盘的通信的接受回调部分在bsp_can.c中
4. Shoot.c主要控制摩擦轮和拨弹电机
5. Gimbal.c主要控制yaw轴和pitch轴两电机
6. Detect.c主要负责异常检测

#### 使用说明

1. 开机前，将遥控器右SW2拨杆拨至最上

2. 出现异常情况，关闭遥控器或将遥控器右SW2拨杆拨至最下，可以使云台失能，进入离线模式

3. 云台每次从离线模式恢复运行时，都会先初始化归中，才能进入后续的控制

4. SW2_UP为普通模式，SW2_MI为自瞄模式，SW2_DW为离线模式

5. 客户端控制时，将遥控器拨至SW2_UP，长按鼠标右键可以进入自瞄模式，松开返回普通模式

6. 遥控摩擦轮的操作：

   | 打开摩擦轮 | 将遥控器左SW1拨杆从不是SW1_UP的位置切到SW1_UP |
   | ---------- | --------------------------------------------- |
   | 关闭摩擦轮 | 将遥控器左SW1拨杆从不是SW1_UP的位置切到SW1_UP |

7. 遥控射击操作：

   | 单发 | 打开摩擦轮后，将遥控器左SW1拨杆从不是SW1_MI的位置切到SW1_MI |
   | ---- | ----------------------------------------------------------- |
   | 连发 | 打开摩擦轮后，将遥控器左SW1拨杆停留在SW1_MI的位置           |

8. 客户端摩擦轮的操作：

   | 打开摩擦轮 | 键盘Q键         |
   | ---------- | --------------- |
   | 关闭摩擦轮 | 键盘SHIFT + Q键 |

9. 客户端射击操作：

   | 单发   | 点击鼠标左键            |
   | ------ | ----------------------- |
   | 三连发 | 键盘SHIFT键             |
   | 连发   | 键盘SHIFT键 +  鼠标右键 |

10. （考虑加入键盘B键重置imu值）





#### 通讯协议：

由于23赛季哨兵的改动，各模块之间通讯更加重要，于是采用了 [BCP通讯协议](https://birdiebot.github.io/bubble_documentation/guide/%E6%9C%A8%E9%B8%A2%E9%80%9A%E8%AE%AF%E5%8D%8F%E8%AE%AE.html) ，并在此基础上根据实际需求进行改动。

下位机与上位机之间通过USB虚拟串口进行连接，下位机通过 CDC_Receive_FS 进行接收，通过 CDC_Transmit_FS 进行发送，在 transmission_task 中装填需要发送的数据。

##### 相关 API：

```
/**
  * @brief  通讯帧结构体 （BCP通讯协议） 此为最大DATA长度的帧，用于接收中转
  */
typedef  struct
{
    uint8_t HEAD;  				    /*! 帧头 */
    uint8_t D_ADDR;                 /*! 目标地址 */
    uint8_t ID;                     /*! 功能码 */
    uint8_t LEN;                    /*! 数据长度 */
    int8_t DATA[FRAME_MAX_LEN];     /*! 数据内容 */
    uint8_t SC;                     /*! 和校验 */
    uint8_t AC;                     /*! 附加校验 */
}__attribute__((packed)) BCPFrameTypeDef;

/**
 * @brief BCP和校验算法
 * @param frame
 * @retval 合并为一个数据的校验码 SC:和校验（高8位） AC:附加和校验（低8位）
 */
uint16_t Sumcheck_Cal(BCPFrameTypeDef frame);

/**
 * @brief 将要发送的数据类型填入到对应的数据帧中
 * @param send_mode 功能码表
 * @param data_buf 填装好的DATA
 */
void Add_Frame_To_Upper(uint16_t send_mode, int8_t* data_buf);
```

##### 具体改动：

- ```
  #define GIMBAL                  0x20        /* 欧拉角rpy方式控制 */
  #define FRAME_RPY_LEN 17        /* 欧拉角rpy方式控制长度 */
  /* rpy帧数据长度增加到17，在末尾增加与射击目标之间的距离 */
  ```