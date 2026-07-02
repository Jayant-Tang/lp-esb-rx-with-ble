# Low Power ESB PRX with BLE

This project demonstrates a low-power ESB PRX running together with Bluetooth LE.

The receiver does not keep ESB RX on all the time. Instead, it opens a short ESB
listen window at a fixed interval through MPSL timeslots.

```text
        ---         ---         ---         ---
        | |         | |         | |         | |         BLE time window
        | |         | |         | |         | |
Time -------------------------------------------------------------
                                                    
              -           -           -           -  
              |           |           |           |     small ESB PRX listen window
              |           |           |           |
Time -------------------------------------------------------------

=======================================================================================

                              |||||||||||||             PTX button burst
Time -------------------------------------------------------------
```

The intended product model is:

1. The PRX is a BLE peripheral and a low-power ESB receiver. Both BLE timeslot and ESB timeslot are short.
2. The PTX is a battery-powered remote control. It normally stays in System OFF. When a button is pressed, the PTX wakes up and sends ESB packets continuously for a short burst.
3. The burst must be longer than the PRX listen interval, so at least one packet overlaps the short PRX window.

> This project doesn't contain PTX project implementation.

## NCS Version

Use **NCS v3.3.0 or later**.

## Difference from Official `esb_prx_ble`

NCS v3.3.0 adds official ESB + Bluetooth LE coexistence support through
`CONFIG_ESB_MPSL_TIMESLOT`, and also adds the official `nrf/samples/esb/esb_prx_ble`
sample  [Enhanced ShockBurst: Receiver with Bluetooth LE — nRF Connect SDK 3.3.0 documentation](https://nrfconnectdocs.nordicsemi.com/ncs/3.3.0/nrf/samples/esb/esb_prx_ble/README.html). 

The v3.3.0 ESB implementation also handles the nRF54L15 TIMER10 sharing
between MPSL and ESB, which was the major issue for this project on NCS v2.9.0.

The official `esb_prx_ble` sample proves that ESB PRX can coexist with Bluetooth
LE. It enables `CONFIG_ESB_MPSL_TIMESLOT=y`, then ESB PRX requests MPSL timeslots
as early as possible and extends them repeatedly. This is good for packet
delivery rate, but it is close to continuous ESB scanning and is not suitable for
a low-power always-on receiver.

This project keeps the v3.3.0 ESB/MPSL integration, but changes the PRX scheduling
policy:

- Official sample: PRX uses available MPSL time aggressively.
- This project: PRX uses fixed-interval NORMAL timeslots.
- Official sample: optimized for coexistence and throughput.
- This project: optimized for low average current.
- Official sample: a PTX that sends every 100 ms is usually received.
- This project: a PTX must send a burst long enough to cover the PRX interval.

```text
Official NCS v3.3.0 esb_prx_ble

        ---         ---         ---         ---
        | |         | |         | |         | |           BLE time window
        | |         | |         | |         | |
Time -------------------------------------------------------------
                         
           ---------   ---------   ---------   ---------   
           |       |   |       |   |       |   |       |   ESB PRX uses most available time
           |       |   |       |   |       |   |       |
Time -------------------------------------------------------------

=======================================================================================

                |       |       |       |             PTX packets
Time -------------------------------------------------------------


```

```text
This project: low-power ESB PRX with BLE

        ---         ---         ---         ---
        | |         | |         | |         | |         BLE time window
        | |         | |         | |         | |
Time -------------------------------------------------------------
                                                    
              -           -           -           -  
              |           |           |           |     small ESB PRX listen window
              |           |           |           |
Time -------------------------------------------------------------

=======================================================================================

                              |||||||||||||             PTX button burst
Time -------------------------------------------------------------
```

The low-power behavior is configured in `src/mpsl_esb/mpsl-esb-overlay.conf`:

```conf
CONFIG_APP_ESB_MPSL_TIMESLOT_RX_INTERVAL_US=500000
CONFIG_APP_ESB_MPSL_TIMESLOT_RX_WINDOW_US=5000
```

This means the PRX listens for about 5 ms every 500 ms.

## SDK Patch

NCS v3.3.0 supports ESB + BLE, but its upstream PRX timeslot behavior is not a
low-power short-window receiver. This project therefore applies a small patch to
only one SDK file:

```text
nrf/subsys/esb/esb.c
```

The patch is stored in:

```text
sdk_changes/v3.3.0/nrf/subsys/esb/esb.c
```

No SDK Kconfig file is modified. The extra configuration symbols are application
symbols defined in `src/mpsl_esb/Kconfig.mpsl_esb`, using the `APP_ESB_` prefix.
If another ESB sample does not define these symbols, the patched `esb.c` keeps
the upstream behavior.

## Supported Hardware

- nRF54L15DK (`nrf54l15dk/nrf54l15/cpuapp`)
- nRF52840DK (`nrf52840dk/nrf52840`)

## How to Build

1. Install NCS v3.3.0 or later.
2. Copy `sdk_changes/v3.3.0/nrf/subsys/esb/esb.c` to the same path in the NCS tree.
3. Build the application:

```powershell
nrfutil sdk-manager toolchain launch --ncs-version=v3.3.0 --chdir D:\ncs\v3.3.0 -- `
  west build -p always -b nrf54l15dk/nrf54l15/cpuapp -d build_nrf54l15 .
```

For nRF52840DK:

```powershell
nrfutil sdk-manager toolchain launch --ncs-version=v3.3.0 --chdir D:\ncs\v3.3.0 -- `
  west build -p always -b nrf52840dk/nrf52840 -d build_nrf52840 .
```

## How to Test

1. Flash this project to the PRX board.
2. Test the BLE part with the nRF Connect mobile app. The device name is `Nordic_LBS`.
3. Flash an ESB PTX sample to another Nordic DK.
4. Modify the PTX to send packets continuously during the active burst.
5. Use a PPK II or current meter to verify the receiver current.

The PTX needs to continuously transmit during the wake window. For example, modify
the official ESB PTX sample like this:

```c
K_SEM_DEFINE(sem_esb_tx, 1, 1);
```

```c
void event_handler(struct esb_evt const *event)
{
	ready = true;

	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		k_sem_give(&sem_esb_tx);
		LOG_DBG("TX SUCCESS EVENT");
		break;
	case ESB_EVENT_TX_FAILED:
		k_sem_give(&sem_esb_tx);
		LOG_DBG("TX FAILED EVENT");
		break;
	case ESB_EVENT_RX_RECEIVED:
		/* Optional: read ACK payload. */
		break;
	}
}
```

```c
tx_payload.noack = true;

while (1) {
	k_sem_take(&sem_esb_tx, K_FOREVER);

	if (ready) {
		ready = false;
		esb_flush_tx();

		err = esb_write_payload(&tx_payload);
		if (err) {
			LOG_ERR("Payload write failed, err %d", err);
		}

		tx_payload.data[1]++;
	}
}
```

Expected PRX log with the default 500 ms interval:

```text
<inf> app_esb_prx: ESB started (MPSL timeslot mode)
<inf> app_esb_prx: Packet received, len 8 : ...
<inf> app_esb_prx: Packet received, len 8 : ...
```

Packets should arrive in short groups approximately every 500 ms.

## Notes

- A PTX that sends only once every 100 ms is not a valid test for this project.
  It will often miss the 5 ms PRX window.
- The upstream `CONFIG_ESB_MPSL_TIMESLOT` behavior is intentionally preserved for
  other ESB samples unless `CONFIG_APP_ESB_MPSL_TIMESLOT_RX_INTERVAL_US` and
  `CONFIG_APP_ESB_MPSL_TIMESLOT_RX_WINDOW_US` are defined by the application.
