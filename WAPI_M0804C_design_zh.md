# WAPI_M0804C 设计模式

## 概览
WAPI_M0804C 在 AT_handler 之上构建模块级流程，采用流程表、状态机与观察者事件系统组织复杂操作。

## 设计模式与结构
- 外观模式封装：对外暴露初始化、连接、发送、证书上传等能力，内部隐藏 AT 细节。
- 流程表驱动：`wapi_process_t` 数组定义初始化、认证、连接等步骤，统一由 `table_process` 执行。
- 状态机：`wapi_state_transition` 约束 `WAPI_STATE_*` 合法迁移。
- 分层重试：指令级 (`AT_ERR_REPEAT_CNT`)、流程表级 (`WAPI_PROCESS_RETRY_MAX`)、线程级失败计数 (`WAPI_PROCESS_FAIL_MAX`)。
- 观察者模式：`wapi_subject_attach`/`detach`/`notify` 管理观察者列表并分发 `wapi_event_t`。
- 依赖注入：电源控制、数据提供、OS 延时与 AT 输入参数均通过 `wapi_m0804c_input_arg_t` 注入。
- 回调式解析：各 AT 命令绑定专用解析回调（如 `at_recv_parse_ok`、`at_recv_parse_tcp_connect`）。

## 数据流概要
1. 线程选择流程表并按顺序执行步骤。
2. 每一步发送 AT 命令并等待回调确认。
3. 成功推进状态，失败触发重试或进入错误状态。
4. 关键状态变更通过事件系统通知外部观察者。

## 扩展点
- 新增模块动作：编写处理函数并插入流程表。
- 新增事件监听：注册 `wapi_observer_t`。
- 调整连接策略：配置 `IS_USE_CONN_BY_CERT` / `IS_USE_CONN_BY_PWD` 或修改流程表。
