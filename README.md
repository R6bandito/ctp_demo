# EasyCantp

> A lightweight CAN TP (ISO 15765-2) protocol stack for resource-constrained microcontrollers 
> No AUTOSAR.
>
> 基于 ISO 15765-2 的 CANTP 嵌入式轻量化协议栈，No AUTOSAR. 适合资源受限的微控制器.

---

## 特性

- **静态连接池管理**，无动态内存（`malloc`/`free`）分配操作.
- **ISR解耦**：中断只做确定性操作（拷贝、状态更新），用户注册回调均运行在任务上下文.
- **支持双寻址模式**：NORMAL（11-bit CAN ID）+ EXT（TA 在数据场）、功能寻址
- **双物理层**：支持 Classic CAN（8 字节）/ CAN FD（12~64 字节）.
- **完整的流控**：实现BS 块大小、STmin 帧间隔、OVERFLOW 中止.
- **可移植**：协议栈内部解耦具体硬件操作，纯C代码，可移植多种类型MCU.
- **RTOS 支持**：根据具体RTOS实现几个必须接口 整个协议栈即可运行于 OS 环境.

---

## 架构

```
ctp.h                     ← 用户只需 #include 这一个
 ├── ctp_types.h          类型、枚举、回调、ASSERT、时序常量
 ├── ctp_frame.h / .c     帧组装 & 解析 (SF / FF / CF / FC)
 ├── ctp_timer.h / .c     软件定时器 (Start / Stop / Expired)
 ├── ctp_channel.h / .c   寻址 & DLC 转换
 ├── ctp_conn.h / .c      连接池 & 生命周期 (Create / Release / Find)
 ├── ctp_fsm.h / .c       主状态机 (StartTransmit / FeedFrame / TxConfirm / MainFunction)
 ├── ctp_os.h / .c        OS 抽象层（内核线程 & 同步 API）
 └── ctp_os_port_freertos.c  FreeRTOS 移植（裸机编译需排除）
```

---

## 运行模式

EasyCantp 支持两种运行模式：

| 模式 | 入口 API | MainFunction 驱动 | 编译要求 |
|------|----------|------------------|----------|
| **裸机** | `Cus_Cantp_CreateTxConn` / `StartTransmit` 等 | 用户主轮询 | 排除 `ctp_os.c` 和 `ctp_os_port_freertos.c` |
| **RTOS** | `Cus_Cantp_OS_CreateTxConn` / `OS_StartTransmit` 等 | 内部Kernel线程自动驱动 | 包含 OS 层，启动 `Cus_CANTP_OS_Init()` |

**裸机模式**下所有协议栈 API 可直接调用，临界区使用 `__disable_irq()` / `__enable_irq()` 保护 MainFunction 与 ISR 的共享数据。

**RTOS 模式**下所有任务的协议栈相关操作经 mailbox 投递给唯一的内核线程进行处理，多任务安全由 具体RTOS 队列保证，MainFunction 与 CAN ISR 之间通过临界区进行访存保护。

> **RTOS 时序提示**：内核线程内部通过 `Cus_Cantp_OS_MailboxFetch` 轮询 mailbox（参数按 **tick** 解释，非毫秒），MainFunction 的驱动周期 = 轮询 tick 数 / `configTICK_RATE_HZ`。调高 `configTICK_RATE_HZ`（如 1000 → 4000）可压缩多帧传输的 CF 帧间隔（从 ~1ms 到 ~250μs）。用户侧 API（`MailboxSend` 等）的毫秒超时语义不受影响。也就是说，若需更快的通讯时序，可通过改变所用 RTOS 的 Tick 频率基准来达到目的(OS环境下)。

> **注意事项**：
>
> 1.裸机编译时务必将 ctp_os.h 中的 \#if (0) 保持关闭状态，且不能链接 `ctp_os_port_freertos.c` ！！！若错误链接将导致 FreeRTOS 的临界区强定义覆盖掉裸机的临界区操作，而 FreeRTOS 在启动调度器之前 `uxCriticalNesting` 被初始化为哨兵值 `0xaaaaaaaa`，会导致在未启动调度器的裸机环境下 `taskEXIT_CRITICAL` 永远无法清零 BASEPRI，所有优先级 ≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 的中断被永久静默屏蔽，从而造成协议栈运行异常。 
>
> 2.RTOS模式下，调度器启动之前不允许使用任何 Cus_Cantp_OS_xxx 相关操作API（除了 OS_Init）. 若如此操作，在某些情况下会导致协议栈运行异常。

---

## 快速开始

### 1. 配置

在 `#include "ctp.h"` 之前定义（可选，不定义则使用默认值）：

```c
#define CUS_CANTP_MAX_TX   2    /* TxConn 池大小            */
#define CUS_CANTP_MAX_RX   2    /* RxConn 池大小            */
#define CUS_CANTP_TIMEOUT_N_AS  1000U
#define CUS_CANTP_TIMEOUT_N_BS  5000U
#define CUS_CANTP_TIMEOUT_N_AR  1000U
#define CUS_CANTP_TIMEOUT_N_CR  1000U
```

### 2. 实现三个回调

```c
/* 此处以 STM32F1 移植作为示例. */
/* CAN 发送 — 返回识别tag(STM32中为邮箱号)（≥0）或 -1（失败） */
int8_t MySendFunc(void *ctx, uint32_t canId, const uint8_t *data, uint8_t dlc) {
    uint32_t mailBox = 0;
    if ( HAL_CAN_AddTxMessage( ..., &mailBox ) != HAL_OK )
        return -1;
    return (int8_t)mailBox;
}

/* 接收完成通知 */
void MyDataInd(void *rxConn, const uint8_t *data, uint32_t len) {
    /* 处理 data[0..len-1] */
}

/* 错误通知 */
void MyErrCb(void *conn, Cus_CANTP_ErrCode_t err) {
    /* 处理 err */
}
```

### 3. 创建连接

```c
Cus_CANTP_ChannelCfg_t chtx = {
    .addrMode = CUS_CANTP_ADDR_MODE_NORMAL,		/* 普通寻址模式 */
    .SA       = 0x02,						  /* 自身连接地址(对于Tx) */ 
    .TA       = 0x01,						  /* 目标连接地址(对于Tx) */
    .fSize    = CUS_CANTP_SIZE_8,			   /* TX_DLC. 必须>=8. 对于Classic CAN为8. */ 
};

const Cus_CANTP_TxConn_t *tx = Cus_Cantp_CreateTxConn(chtx, &hcan1, &app,
                                                        MySendFunc, MyErrCb);

Cus_CANTP_ChannelCfg_t chrx = {
    .addrMode = CUS_CANTP_ADDR_MODE_NORMAL,		/* 普通寻址模式 */
    .SA       = 0x00,						  /* 监听发送方地址(非0表示只接收来自指定发送方的报文) (对于Rx)*/ 
    .TA       = 0x01,						  /* 自身连接地址(对于Rx) */
    .fSize    = CUS_CANTP_SIZE_8,			   /* TX_DLC. 必须>=8. 对于Classic CAN为8. */ 
};
const Cus_CANTP_RxConn_t *rx = Cus_Cantp_CreateRxConn(chrx, &hcan1, &app,
                                                        MySendFunc, MyErrCb);
uint8_t rxBuf[256];
Cus_Cantp_RxConnBindBuf((Cus_CANTP_RxConn_t *)rx, rxBuf, sizeof(rxBuf),
                         8, 0, MyDataInd);   /* BS=8, STmin=0 */
```

### 4. 中断

```c
/* SysTick — 1ms 调用一次 */
void SysTick_Handler(void) { Cus_Cantp_TimerTickInc(); }

/* CAN RX 中断 */
void CAN_RX_IRQHandler(void) {
    uint8_t data[64]; uint32_t canId; uint8_t dlc;
    HAL_CAN_GetRxMessage(&hcan1, ..., &canId, data, &dlc);
    Cus_Cantp_FeedFrame(canId, data, dlc);
}

/* CAN TX 完成中断 */
void CAN_TX_IRQHandler(void) {
    uint32_t mb = HAL_CAN_GetTxMailbox(...);
    Cus_Cantp_TxConfirm(&hcan1, mb);
}
```

### 5. 任务循环

```c
void Task_CANTP(void) {
    while (1) {
        Cus_Cantp_MainFunction();		/* 推动 CANTP 主状态机. */
        vTaskDelay(pdMS_TO_TICKS(5));   /* 5ms 周期 */
    }
}
```

### 6. 发送数据

```c
uint8_t payload[] = "Hello CAN TP!";
int8_t rc = Cus_Cantp_StartTransmit((Cus_CANTP_TxConn_t *)tx, payload, sizeof(payload));
if (rc == 1) {
    while (tx->state != CUS_CANTP_STA_IDLE) {
        vTaskDelay(pdMS_TO_TICKS(1));   /* 等发送完成 */
    }
}
```

### 7. RTOS 模式（可选）

此处以 FreeRTOS 为例，将 `ctp_os.c` 和 `ctp_os_port_freertos.c` 加入工程，调度器启用后，在任务中使用的 API 改为带 `OS_` 前缀的版本：

```c
#include "ctp_os.h"   /* 替換裸机的 ctp.h */

void main(void) {
    /* 硬件初始化 + 连接池 + 定时器（同裸机） */
    Cus_Cantp_ConnPoolInit();
    Cus_Cantp_TimerInit();

    /* 启动 OS 层 — 创建 mailbox + 内核线程 */
    Cus_CANTP_OS_Init();
    ....
}

/* 调度器启动后，由任务调用 OS-safe API */
void MyTask(void *arg) {
    Cus_Cantp_OS_CreateTxConn(chTx, (void *)CAN1, NULL, sendFn, errFn,
                               5000, &tx);                          /* 同步阻塞 */
    Cus_Cantp_OS_StartTransmit((Cus_CANTP_TxConn_t *)tx, data, len,
                               5000);                                /* 异步投递 */
    vTaskSuspend(NULL);
}
```

所有 `Cus_Cantp_OS_*` API 经 mailbox 串行化到唯一的内核线程，多任务并发安全。

---

## API 参考

### 帧操作 (`ctp_frame.h`)

| 函数 | 说明 |
|---|---|
| `Cus_Cantp_BuildSF(frame, fs, off, data, len)` | 组装单帧 |
| `Cus_Cantp_BuildFF(frame, fs, off, data, totLen)` | 组装首帧 |
| `Cus_Cantp_BuildCF(frame, fs, off, data, rem, sn, &copied)` | 组装连续帧 |
| `Cus_Cantp_BuildFC(frame, fs, off, flowState, bs, stmin)` | 组装流控帧 |
| `Cus_Cantp_ParseSF/FF/CF/FC(...)` | 解析各帧类型 |
| `Cus_Cantp_GetPciType(frame, off)` | 获取帧类型（SF/FF/CF/FC） |

### 定时器 (`ctp_timer.h`)

| 函数 | 说明 |
|---|---|
| `Cus_Cantp_TimerInit()` | 初始化全局 tick（可选，静态变量默认 0） |
| `Cus_Cantp_TimerTickInc()` | 驱动 tick（SysTick ISR 中调用） |
| `Cus_Cantp_TimerStart(t, ms)` | 启动一次性定时器 |
| `Cus_Cantp_TimerStop(t)` | 停止定时器 |
| `Cus_Cantp_TimerExpired(t)` | 查询是否超时（回绕安全） |
| `Cus_Cantp_TimerActive(t)` | 查询是否已启动 |
| `Cus_Cantp_TimerNow()` | 获取当前 tick 值 |

### 寻址 (`ctp_channel.h`)

| 函数 | 说明 |
|---|---|
| `Cus_Cantp_GenerateId(mode, ta, sa, taType, funcId)` | 构造 CAN ID |
| `Cus_Cantp_ExtractSA(mode, canId)` | 从 CAN ID 提取 SA |
| `Cus_Cantp_WriteAddrPrefix(frame, cfg)` | EXT 模式写 TA 到数据场 |
| `Cus_Cantp_GetPciOffset(mode)` | 获取 PCI 偏移量（内联） |
| `Cus_Cantp_SizeToLinkLayerDLC(fs)` | 帧字节数 → DLC |
| `Cus_Cantp_LinkLayerDLCToSize(dlc)` | DLC → 帧字节数 |

### 连接池 (`ctp_conn.h`)

| 函数 | 说明 |
|---|---|
| `Cus_Cantp_ConnPoolInit()` | 初始化连接池 |
| `Cus_Cantp_CreateTxConn(cfg, dev, ctx, send, err)` | 创建发送连接 |
| `Cus_Cantp_CreateRxConn(cfg, dev, ctx, send, err)` | 创建接收连接 |
| `Cus_Cantp_RxConnBindBuf(rx, buf, size, bs, stmin, ind)` | 绑定接收缓冲区 |
| `Cus_Cantp_ReleaseConn(conn, type)` | 释放连接回池 |
| `Cus_Cantp_FindTxByTag / FindRxByTag(dev, tag)` | 按 指定设备的 tag 查找（TxConfirm） |
| `Cus_Cantp_FindTxById / FindRxById(canId, frame)` | 按 CAN ID 查找（FeedFrame） |

### 状态机 (`ctp_fsm.h`)

| 函数 | 上下文 | 说明 |
|---|---|---|
| `Cus_Cantp_StartTransmit(tx, data, len)` | Task | 发起传输 |
| `Cus_Cantp_MainFunction()` | Task | 主状态机驱动 |
| `Cus_Cantp_FeedFrame(canId, data, dlc)` | ISR | 喂入接收帧 |
| `Cus_Cantp_TxConfirm(dev, tag)` | ISR | 发送完成确认 |
| `Cus_Cantp_TimerTickInc()` | ISR | 系统 tick |

### 回调

| 回调 | 调用上下文 | 说明 |
|---|---|---|
| `Cus_Cantp_SendFunc_t` | Task / MainFunction | 发送 CAN 帧，返回 tag |
| `Cus_Cantp_DataInd_t` | Task / MainFunction | 完整消息接收通知 |
| `Cus_Cantp_ErrCb_t` | Task / MainFunction | 协议错误通知 |

---

## 配置宏

| 宏 | 默认值 | 说明 |
|:-:|:-:|:-:|
| `CUS_CANTP_MAX_TX` | 2 | 最大 TxConn 数量 |
| `CUS_CANTP_MAX_RX` | 2 | 最大 RxConn 数量 |
| `CUS_CANTP_TIMEOUT_N_AS` | 1000 | 发送确认超时 (ms) |
| `CUS_CANTP_TIMEOUT_N_BS` | 5000 | 等待流控超时 (ms) |
| `CUS_CANTP_TIMEOUT_N_AR` | 1000 | FC 发送确认超时 (ms) |
| `CUS_CANTP_TIMEOUT_N_CR` | 1000 | 等待连续帧超时 (ms) |

---

## 错误码

| 枚举 | 值 | 含义 |
|------|----|------|
| `CUS_CANTP_ERR_NONE` | 0 | 无错误 |
| `CUS_CANTP_ERR_NBS_TIMEOUT` | 1 | 发送方等待 Flow Control 超时 |
| `CUS_CANTP_ERR_NCR_TIMEOUT` | 2 | 接收方等待 Consecutive Frame 超时 |
| `CUS_CANTP_ERR_NAS_TIMEOUT` | 3 | TX mailbox 发送确认超时 |
| `CUS_CANTP_ERR_FLOW_OVFLW` | 4 | 收到 Flow Control OVERFLOW |
| `CUS_CANTP_ERR_SN_MISMATCH` | 5 | 连续帧 Sequence Number 不匹配 |
| `CUS_CANTP_ERR_TX_FAILED` | 6 | 硬件发送失败 |
| `CUS_CANTP_ERR_RX_BUFFER_FULL` | 7 | 接收缓冲区不足 |

---

## Benchmark

### 裸机模式性能

​	example示例：`stm32f1_loopback_demo` 测试用例运行示意：

 <img src="./image/pic1.png" style="zoom:80%;" />

​	逻辑分析仪捕获帧概览图：

![](./image/pic2.png)

<img src="./image/pic3.png" style="zoom:80%;" />

协议栈性能计算 ( 500K CAN baudrate. )

<img src="./image/pic4.png" style="zoom:80%;" />

<img src="./image/pic5.png" style="zoom:80%;" />

在 500kbps 经典 CAN 总线上，完成 4095 字节多帧传输的实测耗时：NORMAL 寻址模式（有效载荷 7 字节/帧）为 155.55ms（587 帧），EXT 寻址模式（有效载荷 6 字节/帧）为 180.00ms（684 帧）。理论无填充极限分别为 126.6ms 和 147.7ms，实测差值主要来自 CAN 协议固有的位填充开销（随机数据下约 22%）及帧间隔（IFS），折算后两种模式的有效吞吐量分别为 26.3KB/s 和 22.8KB/s，总线利用率均稳定在 82% 左右。

实测数据表明裸机环境下，协议栈本身未引入显著软件开销，性能接近 CAN 物理层极限。

### RTOS 模式性能（内核线程驱动）

`stm32f1_rtos_demo.c` 运行示意：

<img src="./image/pic6.png" style="zoom:67%;" />

`configTICK_RATE_HZ`为 3000Hz 时，整包1024字节数据（不含FF）发送耗时:

<img src="./image/pic10.png" style="zoom: 80%;" />

`configTICK_RATE_HZ`为 3500Hz 时，整包1024字节数据（不含FF）发送耗时:

<img src="./image/pic9.png" style="zoom:80%;" />

RTOS 模式下多帧传输的帧间隔受内核线程驱动周期（tick）影响，实测（1024 字节，NORMAL 寻址，500kbps，帧起始到帧起始）：

| `configTICK_RATE_HZ` | tick 周期 | 实测帧间隔 | 备注 |
|----------------------|-----------|-----------|------|
| 2000 | 500μs | ~500μs | 间隔 = 1 tick |
| 3000 | 333μs | ~298μs | 整包 48.8879ms（146 帧 不考虑FF.） |
| 3500 | 286μs | ~292μs | 整包 46.802ms (同样不考虑FF) ，接近物理极限 |
| 4000 | 250μs | ~500μs | 帧发送时间 ≈ tick 周期，触发 2 tick 相位锁定 |

CF 推进依赖内核线程周期性调用 MainFunction，间隔被 tick 节拍主导。

以 RTOS Tick 频率为 3500Hz 为例进行性能计算。标准帧 8 字节位结构：SOF(1) + 仲裁(12) + 控制(6) + 数据(64) + CRC(16) + ACK(2) + EOF(7) = 108 位。

| 层级 | 位/时间 @500kbps | 累计 |
|------|-----------------|------|
| 基础帧（无填充） | 108 位 / 216μs | 216μs |
| 位填充（随机数据 ~20%，范围 SOF~CRC 共 98 位） | ~20 位 / +40μs | ~256μs |
| IFS（3 位） | +6μs | **~262μs 物理极限帧间隔** |

3500Hz 实测帧间隔约为292μs左右 → 调度开销仅 **~30μs/帧（~10%）**。

1024 字节整包（3500Hz，146 帧 CF）：

```
实测整包：46.802ms
有效吞吐量：1024B / 46.802ms ≈ 21.9 KB/s
总线占用：146 帧 × ~262μs ≈ 38.25ms
总线利用率：38.25ms / 46.802ms ≈ 81.72%
```

**RTOS 内核线程驱动未引入额外损耗**：将 `configTICK_RATE_HZ` 从默认 1000 提升至 3500 后，帧间隔压缩至 ~292μs，调度开销仅 ~30μs/帧，总线利用率 82% 与裸机模式一致。**该结论的前提是通过调整 RTOS tick 频率消除了驱动周期瓶颈**——若保持默认 1ms tick，帧间隔会被节拍锁死在 ~1ms；内核线程驱动本身并不额外耗时，但驱动节奏必须跟上物理层。

------

## 许可证

MIT License. Copyright (c) 2026 R6bandito.
