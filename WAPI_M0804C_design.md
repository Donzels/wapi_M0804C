# WAPI_M0804C Design Patterns

## Overview
WAPI_M0804C builds a higher-level device workflow on top of AT_handler. It organizes behavior as process tables, uses a finite state machine, and exposes an observer event system for external listeners.

## Patterns And Structure
- Facade on top of AT_handler: the WAPI handler presents device-level operations (init, connect, send, cert upload) while hiding AT details.
- Table-driven workflow: `wapi_process_t` arrays define ordered steps for init, connect, and upload. `table_process` executes them uniformly with retries.
- State machine: `wapi_state_transition` enforces legal transitions between `WAPI_STATE_*` values (initing, configured, connected, error).
- Retry hierarchy: command-level retries (`AT_ERR_REPEAT_CNT`), process-table retries (`WAPI_PROCESS_RETRY_MAX`), and thread-level fail count (`WAPI_PROCESS_FAIL_MAX`).
- Observer pattern for events: `wapi_subject_attach`, `wapi_subject_detach`, and `wapi_subject_notify` manage a list of observers that receive `wapi_event_t` changes.
- Dependency injection: power ops, data providers, OS delay, and AT handler input are supplied via `wapi_m0804c_input_arg_t`.
- Callback-based parsing: specific receive callbacks (e.g., `at_recv_parse_ok`, `at_recv_parse_tcp_connect`) validate responses for each command.

## Data Flow Summary
1. The process thread selects a process table and runs steps in order.
2. Each step sends one or more AT commands and waits for response via callbacks.
3. The state machine advances on success or transitions to error on repeated failure.
4. Event notifications inform observers about major lifecycle milestones.

## Key Extensibility Points
- Add a new device action: add a process function and insert it into a process table.
- Add a new event consumer: attach a `wapi_observer_t` with an `on_notify` callback.
- Change connectivity policy: switch `IS_USE_CONN_BY_CERT` / `IS_USE_CONN_BY_PWD` or update the connection process tables.
