# grblhal-pneumaseal

A grblHAL plugin that controls a solenoid valve for a spindle air seal.

The air seal keeps contamination (chips, coolant mist) out of the spindle bearing while the machine is in use. This plugin drives a configurable auxiliary digital output that opens or closes the solenoid based on machine state, with automatic shut-off after a configurable idle period.

---

## Behavior

| Machine state | Solenoid |
|---|---|
| Homing, cycle running, jogging, tool change | **ON** — stays on indefinitely |
| Feed hold, safety door | **ON** — turns off after *pause timeout* (default 5 min) |
| Idle, sleep, alarm, E-stop | **ON** — turns off after *idle timeout* (default 2 min) |
| Spindle programmed on (`M3`/`M4`) regardless of motion state | **ON** |

The solenoid is never cut immediately — it always stays on through the relevant timeout. This ensures airflow continues after an E-stop or alarm, giving the operator time to assess and clear the machine before the air seal closes.

---

## Settings

| Setting | Description | Default |
|---|---|---|
| `$<N+0>` | Auxiliary output port number for the solenoid | First free port |
| `$<N+1>` | Idle timeout in seconds (fully stopped: IDLE, SLEEP, ALARM, ESTOP) | `120` |
| `$<N+2>` | Pause timeout in seconds (paused but likely resuming: HOLD, SAFETY_DOOR) | `300` |
| `$<N+3>` | Active states bitmask — solenoid stays ON indefinitely | `STATE_HOMING\|STATE_CYCLE\|STATE_JOG\|STATE_TOOL_CHANGE` |
| `$<N+4>` | Pause states bitmask — solenoid uses the longer pause timeout | `STATE_HOLD\|STATE_SAFETY_DOOR` |

Setting numbers (`N`) are assigned at registration time and depend on what other plugins are loaded. Check `$$` output after enabling the plugin.

---

## Integration

### 1. Copy the plugin files

Place `pneumaseal.c` and `pneumaseal.h` into your driver's source tree, or add this repository as a submodule:

```sh
git submodule add https://github.com/youruser/grblhal-pneumaseal plugins/pneumaseal
```

### 2. Enable in `my_machine.h`

```c
#define PNEUMASEAL_ENABLE 1
```

### 3. Add to build

In `CMakeLists.txt`:
```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE plugins/pneumaseal/pneumaseal.c)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE plugins/pneumaseal)
```

Or in your `Makefile` / IDE project, add `pneumaseal.c` to the source list.

### 4. Register in `plugins_init.h`

```c
#if PNEUMASEAL_ENABLE
extern void pneumaseal_init(void);
pneumaseal_init();
#endif
```

### 5. Wire the solenoid

Connect the solenoid driver circuit to any free auxiliary digital output on your board. Set `$<N+0>` to that port number. The output is **active high** — the pin goes high when the solenoid is open.

---

## Requirements

- grblHAL core with ioports support (`IOPORTS_ENABLE`)
- At least one unclaimed auxiliary digital output port
- Tested on STM32F4xx; should work on any grblHAL driver with ioports

---

## License

LGPL-3.0 — same as grblHAL core.
