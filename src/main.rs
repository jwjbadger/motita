use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{self, PinDriver},
    ledc,
    units,
    pcnt::*,
    peripherals::Peripherals,
};
use std::{
    cmp::min,
    sync::{
        atomic::{AtomicI32, Ordering},
        Arc,
    },
};

const CPR: f32 = 500.0;
const LOW_LIMIT: i16 = -1000;
const HIGH_LIMIT: i16 = 1000;

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    //let mut ch_a = PinDriver::input(peripherals.pins.gpio5).unwrap();
    //let mut ch_b = PinDriver::input(peripherals.pins.gpio4).unwrap();
    let mut high = PinDriver::output(peripherals.pins.gpio25).unwrap();
    high.set_high().unwrap();

    let ledc = peripherals.ledc;
    let mut motor_driver = ledc::LedcDriver::new(
        ledc.channel0,
        ledc::LedcTimerDriver::new(
            ledc.timer0,
            &ledc::config::TimerConfig {
                frequency: units::Hertz(1000),
                resolution: ledc::Resolution::Bits8, 
            },
        )
        .unwrap(),
        peripherals.pins.gpio13,
    ).unwrap();

    motor_driver.set_duty(motor_driver.get_max_duty() / 2).unwrap();

    println!("duty: {}", motor_driver.get_duty());

    motor_driver.enable().unwrap();

    let mut driver = PcntDriver::new(
        peripherals.pcnt0,
        Some(peripherals.pins.gpio5),
        Some(peripherals.pins.gpio4),
        Option::<gpio::AnyInputPin>::None,
        Option::<gpio::AnyInputPin>::None,
    )
    .unwrap();

    driver
        .channel_config(
            PcntChannel::Channel0,
            PinIndex::Pin0,
            PinIndex::Pin1,
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Decrement,
                neg_mode: PcntCountMode::Increment,
                counter_h_lim: HIGH_LIMIT,
                counter_l_lim: LOW_LIMIT,
            },
        )
        .unwrap();

    driver
        .channel_config(
            PcntChannel::Channel1,
            PinIndex::Pin1,
            PinIndex::Pin0,
            &PcntChannelConfig {
                lctrl_mode: PcntControlMode::Reverse,
                hctrl_mode: PcntControlMode::Keep,
                pos_mode: PcntCountMode::Increment,
                neg_mode: PcntCountMode::Decrement,
                counter_h_lim: HIGH_LIMIT,
                counter_l_lim: LOW_LIMIT,
            },
        )
        .unwrap();

    driver.set_filter_value(min(1023, 0 /*10 * 80*/)).unwrap();
    driver.filter_enable().unwrap();

    let approx_value = Arc::new(AtomicI32::new(0));

    unsafe {
        let approx_value = approx_value.clone();
        driver
            .subscribe(move |status| {
                let status = PcntEventType::from_repr_truncated(status);
                if status.contains(PcntEvent::HighLimit) {
                    approx_value.fetch_add(HIGH_LIMIT as i32, Ordering::SeqCst);
                }
                if status.contains(PcntEvent::LowLimit) {
                    approx_value.fetch_add(LOW_LIMIT as i32, Ordering::SeqCst);
                }
            })
            .unwrap();
    }

    driver.event_enable(PcntEvent::HighLimit).unwrap();
    driver.event_enable(PcntEvent::LowLimit).unwrap();
    driver.counter_pause().unwrap();
    driver.counter_clear().unwrap();
    driver.counter_resume().unwrap();

    let mut last_value: i32 = 0;
    loop {
        let value =
            approx_value.load(Ordering::Relaxed) + driver.get_counter_value().unwrap() as i32;

        let speed = (value - last_value) as f32 / CPR / (100f32 * 10f32.powi(-3));

        last_value = value;
        FreeRtos::delay_ms(100u32);
    }
}
