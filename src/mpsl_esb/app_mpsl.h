#ifndef __MPSL_TIMESLOT_H__
#define __MPSL_TIMESLOT_H__

typedef enum {
    APP_TS_STARTED, 
    APP_TS_STOPPED
} app_ts_state_t;

typedef void (*app_mpsl_timeslot_cb_t)(app_ts_state_t);

void app_mpsl_timeslot_start(void);
void app_mpsl_timeslot_isr_register(app_mpsl_timeslot_cb_t cb);
// void app_mpsl_radio_isr_register(app_mpsl_timeslot_cb_t cb);

#endif // __MPSL_TIMESLOT_H__