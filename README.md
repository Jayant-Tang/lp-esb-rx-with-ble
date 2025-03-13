# Low power ESB PRX with BLE

This project is an example that shows how to let a ESB PRX work together with BLE.

It uses [MPSL](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrfxlib/mpsl/README.html#mpsl) to request small time windows at fixed intervals.

```
        ---         ---         ---         ---         ---           
        | |         | |         | |         | |         | |         BLE time window
        | |         | |         | |         | |         | |             
Time -------------------------------------------------------------

              ---         ---         ---         ---         ---          
              | |         | |         | |         | |         | |   ESB time window
              | |         | |         | |         | |         | |         
Time --------------------------------------------------------------
```

The system is low power during the idle state.

The development of BLE part is not affected at all. The `main.c` is totally same as the [LBS examble](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/samples/bluetooth/peripheral_lbs/README.html).

# Supported Hardware

- nRF52840DK

# How to build

1. Copy the files in `sdk_changes` folder to NCS.
2. Build this project in NCS v2.9.0. 

# How to test

1. Use a [PPK II](https://www.nordicsemi.com/Products/Development-hardware/Power-Profiler-Kit-2) or current meter to measure the current
2. Power on the board.
3. Test the BLE Part by nRF Connect APP.
4. Test the ESB part by another Nordic DK, which runs [ESB PTX](https://docs.nordicsemi.com/bundle/ncs-2.9.0/page/nrf/samples/esb/esb_ptx/README.html).

> You should modify the ESB PTX as below, to let the PTX send packets continuously, which can make sure to match the small PRX time window.
>
> ```c
> K_SEM_DEFINE(sem_esb_tx, 1, 1);
> ```
>
> ```c
> void event_handler(struct esb_evt const *event)
> {
> 	ready = true;
> 
> 	switch (event->evt_id) {
> 	case ESB_EVENT_TX_SUCCESS:
> 		k_sem_give(&sem_esb_tx);
> 		LOG_DBG("TX SUCCESS EVENT");
> 		break;
> 	case ESB_EVENT_TX_FAILED:
> 		k_sem_give(&sem_esb_tx);
> 		LOG_DBG("TX FAILED EVENT");
> 		break;
> 	case ESB_EVENT_RX_RECEIVED:
> 		...
> 		break;
> 	}
> }
> ```
>
> ```c
> int main(void)
> {
>     ...
>     
> 	tx_payload.noack = true;
> 	while (1) {
> 		k_sem_take(&sem_esb_tx, K_FOREVER);
> 		if (ready) {
> 			ready = false;
> 			esb_flush_tx();
> 			leds_update(tx_payload.data[1]);
> 
> 			err = esb_write_payload(&tx_payload);
> 			if (err) {
> 				LOG_ERR("Payload write failed, err %d", err);
> 			}
> 			tx_payload.data[1]++;
> 		}
> 		// k_sleep(K_MSEC(100));
> 	}
> }
> 
> ```

# 