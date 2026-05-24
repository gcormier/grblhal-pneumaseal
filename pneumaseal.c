/*
  pneumaseal.c — grblHAL plugin for spindle air seal solenoid control

  Controls a configurable auxiliary digital output that drives a solenoid
  valve keeping the spindle air seal pressurized while the machine is in use.
  Turns off automatically after a configurable idle period.

  License: LGPL-3.0 (same as grblHAL core)
*/

#include "driver.h"

#if PNEUMASEAL_ENABLE

#include "grbl/hal.h"
#include "grbl/ioports.h"
#include "grbl/nvs_buffer.h"
#include "grbl/report.h"
#include "grbl/settings.h"
#include "grbl/system.h"
#include "grbl/state_machine.h"
#include "grbl/task.h"

#include "pneumaseal.h"

// ── defaults ──────────────────────────────────────────────────────────────────

#define PS_DEFAULT_IDLE_TIMEOUT   120U   // seconds — IDLE / SLEEP / ALARM / ESTOP
#define PS_DEFAULT_PAUSE_TIMEOUT  300U   // seconds — HOLD / SAFETY_DOOR

// sys_state_t bitmasks for default categories.
// STATE_HOMING=bit(2)=4, STATE_CYCLE=bit(3)=8, STATE_JOG=bit(5)=32,
// STATE_TOOL_CHANGE=bit(9)=512  →  default 556
#define PS_DEFAULT_ACTIVE_STATES  (STATE_HOMING | STATE_CYCLE | STATE_JOG | STATE_TOOL_CHANGE)

// STATE_HOLD=bit(4)=16, STATE_SAFETY_DOOR=bit(6)=64  →  default 80
#define PS_DEFAULT_PAUSE_STATES   (STATE_HOLD | STATE_SAFETY_DOOR)

// Setting ID base — increment if another plugin claims Setting_UserDefined_0.
#define PS_SETTING_BASE  Setting_UserDefined_0   // IDs 450..454

// ── NVS block ─────────────────────────────────────────────────────────────────

typedef struct {
    uint8_t  port;
    uint32_t idle_timeout;
    uint32_t pause_timeout;
    uint32_t active_states;
    uint32_t pause_states;
} ps_nvs_t;

static ps_nvs_t cfg;
static nvs_address_t nvs_addr = 0;
static io_port_cfg_t d_out;

// ── runtime state ─────────────────────────────────────────────────────────────

static bool    solenoid_on  = false;
static uint8_t active_port  = IOPORT_UNASSIGNED;  // post-claim remapped index; cfg.port is the NVS value

// ── chained callbacks ─────────────────────────────────────────────────────────

static on_state_change_ptr        prev_on_state_change        = NULL;
static on_spindle_programmed_ptr  prev_on_spindle_programmed  = NULL;
static on_program_completed_ptr   prev_on_program_completed   = NULL;
static on_report_options_ptr      prev_on_report_options      = NULL;
static driver_reset_ptr           prev_driver_reset           = NULL;

// ── solenoid output ───────────────────────────────────────────────────────────

static void solenoid_set (bool on)
{
    if (active_port == IOPORT_UNASSIGNED)
        return;
    solenoid_on = on;
    ioport_digital_out(active_port, (uint32_t)on);
}

// ── timer ─────────────────────────────────────────────────────────────────────

// Identified by function pointer — task_delete(off_timer_fired, NULL) cancels it.
static void off_timer_fired (void *data)
{
    solenoid_set(false);
}

static void cancel_off_timer (void)
{
    task_delete(off_timer_fired, NULL);
}

static void schedule_off (uint32_t delay_s)
{
    cancel_off_timer();
    if (delay_s == 0)
        solenoid_set(false);
    else
        task_add_delayed(off_timer_fired, NULL, delay_s * 1000UL);
}

// ── state classification ──────────────────────────────────────────────────────

typedef enum { CAT_ACTIVE, CAT_PAUSE, CAT_IDLE } ps_cat_t;

static ps_cat_t classify (sys_state_t state)
{
    if (state & cfg.active_states) return CAT_ACTIVE;
    if (state & cfg.pause_states)  return CAT_PAUSE;
    return CAT_IDLE;
}

// Central logic: evaluate machine + spindle state and update solenoid.
static void apply_state (sys_state_t state)
{
    cancel_off_timer();
    ps_cat_t cat = classify(state);

    spindle_ptrs_t *sp = spindle_get(0);
    bool spindle_running = sp != NULL && sp->get_state != NULL && sp->get_state(sp).on;

    if (cat == CAT_ACTIVE || spindle_running) {
        // Active motion or spindle running — ON indefinitely, no timer.
        solenoid_set(true);
    } else if (cat == CAT_PAUSE) {
        // Paused but likely resuming — ON for the longer pause timeout.
        solenoid_set(true);
        schedule_off(cfg.pause_timeout);
    } else {
        // Fully stopped (IDLE, SLEEP, ALARM, ESTOP) — ON for idle timeout.
        // Solenoid is NOT cut immediately after an alarm or e-stop.
        solenoid_set(true);
        schedule_off(cfg.idle_timeout);
    }
}

// ── event callbacks ───────────────────────────────────────────────────────────

static void on_state_change (sys_state_t state)
{
    if (prev_on_state_change)
        prev_on_state_change(state);

    apply_state(state);
}

static void on_spindle_programmed (spindle_ptrs_t *spindle, spindle_state_t state, float rpm, spindle_rpm_mode_t mode)
{
    if (prev_on_spindle_programmed)
        prev_on_spindle_programmed(spindle, state, rpm, mode);

    if (state.on) {
        // Spindle turning on — cancel any pending off timer, keep solenoid on.
        // Note: rpm may be non-zero even for M5 (grblHAL passes last S-value), so
        // state.on is the only reliable indicator of commanded spindle direction.
        cancel_off_timer();
        solenoid_set(true);
    } else {
        // Spindle programmed off (M5).  This callback fires inside spindle_set_state_synced
        // BEFORE the hardware set_state call, so get_state() still reads on.
        // Use the programmed state here; apply_state uses live query for all other paths.
        ps_cat_t cat = classify(state_get());
        if (cat != CAT_ACTIVE) {
            uint32_t delay = (cat == CAT_PAUSE) ? cfg.pause_timeout : cfg.idle_timeout;
            schedule_off(delay);
        }
        // CAT_ACTIVE: still in motion; on_state_change re-evaluates when it ends.
    }
}

static void on_program_completed (program_flow_t program_flow, bool check_mode)
{
    if (prev_on_program_completed)
        prev_on_program_completed(program_flow, check_mode);

    // spindle_all_off() was called before this fires. Re-evaluate with live spindle
    // state — needed when machine stays in STATE_IDLE throughout (no on_state_change).
    apply_state(state_get());
}

static void on_driver_reset (void)
{
    prev_driver_reset();     // always call original — never skip this
    // Don't cut the solenoid immediately — apply the idle timeout so ESTOP/ALARM
    // behaviour matches the design requirement (same as CAT_IDLE in apply_state).
    if (solenoid_on)
        schedule_off(cfg.idle_timeout);
}

static void on_report_options (bool newopt)
{
    if (prev_on_report_options)
        prev_on_report_options(newopt);

    if (!newopt)
        report_plugin("PneumaSeal", "1.3");
}

// ── settings ──────────────────────────────────────────────────────────────────

// Port: uses io_port_cfg_t so the system enumerates and validates available
// aux output ports (same pattern as the fans plugin).
static status_code_t set_port (setting_id_t id, float value)
{
    return d_out.set_value(&d_out, &cfg.port, (pin_cap_t){}, value);
}

static float get_port (setting_id_t id)
{
    return d_out.get_value(&d_out, cfg.port);
}

// Integer settings — the settings system passes uint_fast16_t for Format_Integer.
static status_code_t set_idle_timeout (setting_id_t id, uint_fast16_t value)
{
    cfg.idle_timeout = (uint32_t)value;
    return Status_OK;
}

static uint32_t get_idle_timeout (setting_id_t id)
{
    return cfg.idle_timeout;
}

static status_code_t set_pause_timeout (setting_id_t id, uint_fast16_t value)
{
    cfg.pause_timeout = (uint32_t)value;
    return Status_OK;
}

static uint32_t get_pause_timeout (setting_id_t id)
{
    return cfg.pause_timeout;
}

static status_code_t set_active_states (setting_id_t id, uint_fast16_t value)
{
    cfg.active_states = (uint32_t)value;
    return Status_OK;
}

static uint32_t get_active_states (setting_id_t id)
{
    return cfg.active_states;
}

static status_code_t set_pause_states (setting_id_t id, uint_fast16_t value)
{
    cfg.pause_states = (uint32_t)value;
    return Status_OK;
}

static uint32_t get_pause_states (setting_id_t id)
{
    return cfg.pause_states;
}

static const setting_detail_t ps_settings[] = {
    { PS_SETTING_BASE + 0, Group_AuxPorts, "PneumaSeal solenoid port", NULL,
      Format_Decimal, "-#0", "-1", d_out.port_maxs,
      Setting_NonCoreFn, set_port, get_port, NULL,
      { .reboot_required = On } },
    { PS_SETTING_BASE + 1, Group_Unknown, "PneumaSeal idle timeout", "s",
      Format_Integer, "####", "0", "3600",
      Setting_NonCoreFn, set_idle_timeout, get_idle_timeout, NULL },
    { PS_SETTING_BASE + 2, Group_Unknown, "PneumaSeal pause timeout", "s",
      Format_Integer, "####", "0", "3600",
      Setting_NonCoreFn, set_pause_timeout, get_pause_timeout, NULL },
    { PS_SETTING_BASE + 3, Group_Unknown, "PneumaSeal active states bitmask", NULL,
      Format_Integer, "#####", "0", "65535",
      Setting_NonCoreFn, set_active_states, get_active_states, NULL },
    { PS_SETTING_BASE + 4, Group_Unknown, "PneumaSeal pause states bitmask", NULL,
      Format_Integer, "#####", "0", "65535",
      Setting_NonCoreFn, set_pause_states, get_pause_states, NULL },
};

static const setting_descr_t ps_descriptions[] = {
    { PS_SETTING_BASE + 0,
      "Aux output port number for the air seal solenoid. Set to -1 to disable." },
    { PS_SETTING_BASE + 1,
      "Seconds before solenoid turns off when machine is idle, asleep, alarmed, "
      "or e-stopped. 0 = off immediately." },
    { PS_SETTING_BASE + 2,
      "Seconds before solenoid turns off during a feed hold or safety door event. "
      "0 = off immediately." },
    { PS_SETTING_BASE + 3,
      "sys_state_t bitmask: states where solenoid stays on with no timeout. "
      "Default 556 = HOMING(4) + CYCLE(8) + JOG(32) + TOOL_CHANGE(512)." },
    { PS_SETTING_BASE + 4,
      "sys_state_t bitmask: states that use the longer pause timeout. "
      "Default 80 = HOLD(16) + SAFETY_DOOR(64)." },
};

static void ps_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_addr, (uint8_t *)&cfg, sizeof(ps_nvs_t), true);
}

static void ps_settings_restore (void)
{
    cfg.port          = d_out.get_next(&d_out, IOPORT_UNASSIGNED, "Air Seal Solenoid", (pin_cap_t){});
    cfg.idle_timeout  = PS_DEFAULT_IDLE_TIMEOUT;
    cfg.pause_timeout = PS_DEFAULT_PAUSE_TIMEOUT;
    cfg.active_states = PS_DEFAULT_ACTIVE_STATES;
    cfg.pause_states  = PS_DEFAULT_PAUSE_STATES;
    ps_settings_save();
}

static void ps_setup (void)
{
    prev_on_state_change = grbl.on_state_change;
    grbl.on_state_change = on_state_change;

    prev_on_spindle_programmed = grbl.on_spindle_programmed;
    grbl.on_spindle_programmed = on_spindle_programmed;

    prev_on_program_completed = grbl.on_program_completed;
    grbl.on_program_completed = on_program_completed;

    prev_driver_reset = hal.driver_reset;
    hal.driver_reset = on_driver_reset;
}

static void ps_settings_load (void)
{
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&cfg, nvs_addr, sizeof(ps_nvs_t), true) != NVS_TransferResult_OK)
        ps_settings_restore();

    active_port = cfg.port;
    if (active_port != IOPORT_UNASSIGNED && d_out.claim(&d_out, &active_port, "Air Seal Solenoid", (pin_cap_t){}))
        ps_setup();
    else {
        active_port = IOPORT_UNASSIGNED;
        task_run_on_startup(report_warning, "PneumaSeal: configured port not available");
    }
}

// ── init ──────────────────────────────────────────────────────────────────────

void pneumaseal_init (void)
{
    static setting_details_t setting_details = {
        .settings       = ps_settings,
        .n_settings     = sizeof(ps_settings) / sizeof(setting_detail_t),
        .descriptions   = ps_descriptions,
        .n_descriptions = sizeof(ps_descriptions) / sizeof(setting_descr_t),
        .save           = ps_settings_save,
        .load           = ps_settings_load,
        .restore        = ps_settings_restore,
    };

    if (!ioports_cfg(&d_out, Port_Digital, Port_Output)->n_ports) {
        task_run_on_startup(report_warning, "PneumaSeal: no digital output ports available");
        return;
    }

    if (!(nvs_addr = nvs_alloc(sizeof(ps_nvs_t)))) {
        task_run_on_startup(report_warning, "PneumaSeal: NVS allocation failed");
        return;
    }

    settings_register(&setting_details);

    prev_on_report_options = grbl.on_report_options;
    grbl.on_report_options = on_report_options;
}

#endif // PNEUMASEAL_ENABLE
