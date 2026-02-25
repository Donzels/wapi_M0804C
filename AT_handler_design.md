# AT_handler Design Patterns

## Overview
AT_handler uses a table-driven, layered architecture to decouple AT command definitions, UART transport, and OS primitives. The design enables reuse across modules and keeps the handler stateful but hardware-agnostic.

## Patterns And Structure
- Table-driven command dispatch: `at_cmd_set_table_t` maps `at_func` to a format string and a callback set. This separates command definitions from logic and makes extension additive.
- Strategy for receive parsing: the UART protocol parse algorithm is injected, and the handler installs its own `at_parse_algo` as the transparent parse function. The parse hook can be swapped via `at_recv_hook_register`.
- State machine: explicit states (`AT_STATE_UNINIT`, `AT_STATE_IDLE`, `AT_STATE_SENDING`, `AT_STATE_WAITING_RESPONSE`, `AT_STATE_ERROR`) with validation in `at_state_transition` to enforce legal transitions.
- Command/response sequencing with queue: a `send_info_t` object is queued and reused to correlate subsequent responses with the original command or transparent send.
- Semaphore-based send gate: `send_feedback_sema_handle` ensures only one outstanding send path is active and prevents overlap.
- Observer-like raw hook: `at_recv_hook_register` provides a pre-parse callback for unsolicited frames without changing the core parser.
- Dependency injection for OS and UART: `at_input_arg_t` provides OS and UART interfaces so the handler remains platform-agnostic.

## Data Flow Summary
1. `AT_CMD_SEND` formats a command from the table and enters the sending state.
2. UART write sends the buffer and a timeout timer starts.
3. RX data triggers `at_parse_algo`, which selects the right callback based on the queued `send_info_t` and remaining count.
4. Completion stops the timer, releases the send semaphore, and returns to IDLE.

## Key Extensibility Points
- Add a new AT command: append a new `at_cmd_set_t` entry in the table.
- Change parsing behavior: register a receive hook or swap UART protocol parse algorithm.
- Change OS implementation: provide a new `at_os_interface_t` without touching handler logic.
