# AT_handler 设计模式

## 概览
AT_handler 采用表驱动与分层架构，解耦 AT 指令定义、UART 传输与 OS 原语，实现可复用且与硬件无关的处理器。

## 设计模式与结构
- 表驱动指令分发：`at_cmd_set_table_t` 将 `at_func` 映射到格式化字符串与回调集合，新增命令只需扩展表。
- 策略式接收解析：UART 协议层注入解析算法，handler 安装自身的 `at_parse_algo`；同时提供 `at_recv_hook_register` 作为前置钩子。
- 状态机：`AT_STATE_UNINIT`、`AT_STATE_IDLE`、`AT_STATE_SENDING`、`AT_STATE_WAITING_RESPONSE`、`AT_STATE_ERROR`，通过 `at_state_transition` 校验合法跳转。
- 发送-接收关联：`send_info_t` 入队，后续接收按队列上下文选择正确回调。
- 信号量发送门：`send_feedback_sema_handle` 确保同一时刻只有一个未完成发送。
- 依赖注入：`at_input_arg_t` 提供 OS 与 UART 接口，隔离平台差异。

## 数据流概要
1. `AT_CMD_SEND` 从命令表格式化指令并进入发送状态。
2. UART 发送后启动超时定时器。
3. 接收中断触发 `at_parse_algo`，按队列上下文分发到对应解析回调。
4. 完成后停止定时器、释放信号量并回到 IDLE。

## 扩展点
- 新增 AT 指令：添加 `at_cmd_set_t` 表项。
- 修改解析策略：注册接收钩子或更换 UART 协议解析算法。
- 迁移平台：提供新的 `at_os_interface_t` 实现即可。
