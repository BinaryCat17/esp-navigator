use esp_idf_svc::hal::gpio::*;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::uart::*;
use esp_idf_sys::*;
use log::*;
use std::sync::atomic::{AtomicU16, Ordering};

static BT_HANDLE: AtomicU16 = AtomicU16::new(0);

extern "C" fn esp_spp_cb(event: esp_spp_cb_event_t, param: *mut esp_spp_cb_param_t) {
    let param = unsafe { &*param };
    #[allow(non_upper_case_globals)]
    match event {
        esp_spp_cb_event_t_ESP_SPP_INIT_EVT => {
            info!("SPP Init Event: configuring GAP and starting server...");
            unsafe {
                let ret = esp_bt_dev_set_device_name(b"ESP32-Navigator\0".as_ptr() as *const _);
                if ret != 0 { error!("set_device_name failed: {}", ret); }

                let ret = esp_bt_gap_set_scan_mode(
                    esp_bt_connection_mode_t_ESP_BT_CONNECTABLE,
                    esp_bt_discovery_mode_t_ESP_BT_GENERAL_DISCOVERABLE,
                );
                if ret != 0 { error!("set_scan_mode failed: {}", ret); }

                let ret = esp_spp_start_srv(
                    ESP_SPP_SEC_NONE as u16,
                    esp_spp_role_t_ESP_SPP_ROLE_SLAVE,
                    0,
                    b"SPP_SERVER\0".as_ptr() as *const _,
                );
                if ret != 0 { error!("esp_spp_start_srv failed: {}", ret); }
            }
        }
        esp_spp_cb_event_t_ESP_SPP_START_EVT => {
            info!("SPP Server Started! DEVICE IS DISCOVERABLE.");
        }
        esp_spp_cb_event_t_ESP_SPP_SRV_OPEN_EVT => {
            let handle = unsafe { param.srv_open.handle };
            info!("Bluetooth client connected, handle: {}", handle);
            BT_HANDLE.store(handle as u16, Ordering::SeqCst);
        }
        esp_spp_cb_event_t_ESP_SPP_CLOSE_EVT => {
            info!("Bluetooth client disconnected");
            BT_HANDLE.store(0, Ordering::SeqCst);
        }
        _ => {}
    }
}

fn init_bluetooth() -> anyhow::Result<()> {
    unsafe {
        info!("Bluetooth initialization started (Dual Mode configuration)...");

        let ret = nvs_flash_init();
        if ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND {
            let _ = nvs_flash_erase();
            let _ = nvs_flash_init();
        }
        
        esp_event_loop_create_default();

        let mut cfg = esp_bt_controller_config_t {
            controller_task_stack_size: ESP_TASK_BT_CONTROLLER_STACK as u16,
            controller_task_prio: ESP_TASK_BT_CONTROLLER_PRIO as u8,
            hci_uart_no: 0, // 0 for Internal VHCI (MUST BE 0 FOR VISIBILITY)
            hci_uart_baudrate: 115200,
            scan_duplicate_mode: 0,
            scan_duplicate_type: 0,
            normal_adv_size: 20,
            mesh_adv_size: 0,
            send_adv_reserved_size: 1000,
            controller_debug_flag: 0,
            mode: esp_bt_mode_t_ESP_BT_MODE_BTDM as u8, // Back to Dual Mode
            ble_max_conn: 3, // Back to 3
            bt_max_acl_conn: 3,
            bt_sco_datapath: 0,
            auto_latency: false,
            bt_legacy_auth_vs_evt: true,
            bt_max_sync_conn: 0,
            ble_sca: 0,
            pcm_role: 0,
            pcm_polar: 0,
            pcm_fsyncshp: 0,
            hli: false,
            dup_list_refresh_period: 0,
            ble_scan_backoff: false,
            magic: ESP_BT_CONTROLLER_CONFIG_MAGIC_VAL,
        };
        
        info!("Step 2: BT Controller Init");
        let ret = esp_bt_controller_init(&mut cfg);
        if ret != 0 { return Err(anyhow::anyhow!("BT Controller Init failed: {}", ret)); }
        
        info!("Step 3: BT Controller Enable");
        let ret = esp_bt_controller_enable(esp_bt_mode_t_ESP_BT_MODE_BTDM);
        if ret != 0 { return Err(anyhow::anyhow!("BT Controller Enable failed: {}", ret)); }

        info!("Step 4: Bluedroid Init");
        let ret = esp_bluedroid_init();
        if ret != 0 { return Err(anyhow::anyhow!("Bluedroid Init failed: {}", ret)); }

        info!("Step 5: Bluedroid Enable");
        let ret = esp_bluedroid_enable();
        if ret != 0 { return Err(anyhow::anyhow!("Bluedroid Enable failed: {}", ret)); }

        info!("Step 6: Registering SPP Callback...");
        let ret = esp_spp_register_callback(Some(esp_spp_cb));
        if ret != 0 { return Err(anyhow::anyhow!("SPP Register Callback failed: {}", ret)); }

        let ret = esp_spp_init(esp_spp_mode_t_ESP_SPP_MODE_CB);
        if ret != 0 { return Err(anyhow::anyhow!("SPP Init failed: {}", ret)); }
        
        info!("Bluetooth setup request sent.");
    }
    Ok(())
}

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    unsafe {
        esp_log_level_set(b"*\0".as_ptr() as *const _, esp_log_level_t_ESP_LOG_INFO);
    }

    info!("Starting ESP32 Navigator Relay...");

    let peripherals = Peripherals::take()?;
    
    let uart_config = config::Config::new().baudrate(460_800.into());
    let uart = UartDriver::new(
        peripherals.uart2,
        peripherals.pins.gpio17,
        peripherals.pins.gpio16,
        Option::<Gpio0>::None,
        Option::<Gpio0>::None,
        &uart_config,
    )?;

    // Инициализируем Bluetooth прямо здесь, на Core 0. 
    // ESP-IDF сам создаст нужные задачи на Core 1 согласно sdkconfig.
    if let Err(e) = init_bluetooth() {
        error!("Bluetooth Error: {:?}", e);
    }

    let mut buffer = [0u8; 512];
    loop {
        if let Ok(len) = uart.read(&mut buffer, 10) {
            if len > 0 {
                let handle = BT_HANDLE.load(Ordering::SeqCst);
                if handle != 0 {
                    unsafe {
                        esp_spp_write(handle as u32, len as i32, buffer.as_ptr() as *mut u8);
                    }
                }
            }
        }
        esp_idf_svc::hal::delay::FreeRtos::delay_ms(1);
    }
}
