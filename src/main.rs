use esp_idf_svc::hal::gpio::*;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::uart::*;
use esp_idf_sys::*;
use log::*;
use std::sync::atomic::{AtomicU16, Ordering};

static BT_HANDLE: AtomicU16 = AtomicU16::new(0);

#[repr(C)]
struct CompatibleBTConfig {
    controller_task_stack_size: u16,
    controller_task_prio: u8,
    hci_uart_no: u8,
    hci_uart_baudrate: u32,
    scan_duplicate_mode: u8,
    scan_duplicate_type: u8,
    normal_adv_size: u16,
    mesh_adv_size: u16,
    send_adv_reserved_size: u16,
    controller_debug_flag: u32,
    mode: u8,
    ble_max_conn: u8,
    bt_max_acl_conn: u8,
    bt_sco_datapath: u8,
    auto_latency: bool,
    bt_legacy_auth_vs_evt: bool,
    bt_max_sync_conn: u8,
    ble_sca: u8,
    pcm_role: u8,
    pcm_polar: u8,
    pcm_fsyncshp: u8,
    hli: bool,
    enc_key_sz_min: u8,
    dup_list_refresh_period: u16,
    ble_scan_backoff: bool,
    ble_llcp_disc_flag: u8,
    ble_aa_check: bool,
    ble_chan_ass_en: u8,
    ble_ping_en: u8,
    magic: u32,
}

extern "C" fn esp_spp_cb(event: esp_spp_cb_event_t, param: *mut esp_spp_cb_param_t) {
    let param = unsafe { &*param };
    #[allow(non_upper_case_globals)]
    match event {
        esp_spp_cb_event_t_ESP_SPP_INIT_EVT => {
            unsafe {
                esp_bt_dev_set_device_name(b"ESP32-Navigator\0".as_ptr() as *const _);
                esp_bt_gap_set_scan_mode(
                    esp_bt_connection_mode_t_ESP_BT_CONNECTABLE,
                    esp_bt_discovery_mode_t_ESP_BT_GENERAL_DISCOVERABLE,
                );
                esp_spp_start_srv(
                    ESP_SPP_SEC_NONE as u16,
                    esp_spp_role_t_ESP_SPP_ROLE_SLAVE,
                    0,
                    b"SPP_SERVER\0".as_ptr() as *const _,
                );
            }
        }
        esp_spp_cb_event_t_ESP_SPP_SRV_OPEN_EVT => {
            let handle = unsafe { param.srv_open.handle };
            BT_HANDLE.store(handle as u16, Ordering::SeqCst);
        }
        esp_spp_cb_event_t_ESP_SPP_CLOSE_EVT => {
            BT_HANDLE.store(0, Ordering::SeqCst);
        }
        _ => {}
    }
}

fn init_bluetooth() -> anyhow::Result<()> {
    unsafe {
        info!("Step 1: Releasing BLE memory...");
        esp_bt_controller_mem_release(esp_bt_mode_t_ESP_BT_MODE_BLE);

        let mut cfg = CompatibleBTConfig {
            controller_task_stack_size: 4096,
            controller_task_prio: 23,
            hci_uart_no: 0, 
            hci_uart_baudrate: 115200,
            scan_duplicate_mode: 0,
            scan_duplicate_type: 0,
            normal_adv_size: 20,
            mesh_adv_size: 0,
            send_adv_reserved_size: 1000,
            controller_debug_flag: 0,
            mode: 2, // Classic Only
            ble_max_conn: 0,
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
            enc_key_sz_min: 7,
            dup_list_refresh_period: 0,
            ble_scan_backoff: false,
            ble_llcp_disc_flag: 0,
            ble_aa_check: false,
            ble_chan_ass_en: 1,
            ble_ping_en: 1,
            magic: ESP_BT_CONTROLLER_CONFIG_MAGIC_VAL,
        };
        
        info!("Step 2: BT Controller Init");
        let ret = esp_bt_controller_init(&mut cfg as *mut _ as *mut esp_bt_controller_config_t);
        if ret != 0 { return Err(anyhow::anyhow!("BT Controller Init failed: {}", ret)); }
        
        info!("Step 3: BT Controller Enable");
        let ret = esp_bt_controller_enable(esp_bt_mode_t_ESP_BT_MODE_CLASSIC_BT);
        if ret != 0 { return Err(anyhow::anyhow!("BT Controller Enable failed: {}", ret)); }

        info!("Step 4: Bluedroid Init");
        let ret = esp_bluedroid_init();
        if ret != 0 { return Err(anyhow::anyhow!("Bluedroid Init failed: {}", ret)); }

        info!("Step 5: Bluedroid Enable");
        let ret = esp_bluedroid_enable();
        if ret != 0 { return Err(anyhow::anyhow!("Bluedroid Enable failed: {}", ret)); }

        info!("Step 6: SPP Init");
        esp_spp_register_callback(Some(esp_spp_cb));
        let ret = esp_spp_init(esp_spp_mode_t_ESP_SPP_MODE_CB);
        if ret != 0 { return Err(anyhow::anyhow!("SPP Init failed: {}", ret)); }
    }
    Ok(())
}

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    info!("Starting ESP32 Navigator Relay...");

    // Общая инициализация системы на Core 0
    unsafe {
        let ret = nvs_flash_init();
        if ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND {
            let _ = nvs_flash_erase();
            let _ = nvs_flash_init();
        }
        esp_event_loop_create_default();
    }

    if let Err(e) = init_bluetooth() {
        error!("Bluetooth Error: {:?}", e);
    }

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