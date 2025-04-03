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
    let mut speed_control: f32 = 1.0;
    let k_i: f32 = 0.005;
    let k_p: f32 = 0.11;
    let k_d: f32 = 0.0001;

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
        peripherals.pins.gpio18,
    ).unwrap();

    let max_duty = motor_driver.get_max_duty();

    motor_driver.set_duty((max_duty as f32 * speed_control.clamp(0.0, 1.0)) as u32).unwrap();
    motor_driver.enable().unwrap();

    let mut pcnt_driver = PcntDriver::new(
        peripherals.pcnt0,
        Some(peripherals.pins.gpio5),
        Some(peripherals.pins.gpio4),
        Option::<gpio::AnyInputPin>::None,
        Option::<gpio::AnyInputPin>::None,
    )
    .unwrap();

    pcnt_driver
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

    pcnt_driver
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

    pcnt_driver.set_filter_value(min(1023, 0 /*10 * 80*/)).unwrap();
    pcnt_driver.filter_enable().unwrap();

    let approx_value = Arc::new(AtomicI32::new(0));

    unsafe {
        let approx_value = approx_value.clone();
        pcnt_driver
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

    pcnt_driver.event_enable(PcntEvent::HighLimit).unwrap();
    pcnt_driver.event_enable(PcntEvent::LowLimit).unwrap();
    pcnt_driver.counter_pause().unwrap();
    pcnt_driver.counter_clear().unwrap();
    pcnt_driver.counter_resume().unwrap();

    let mut last_value: i32 = 0;

    let target_velocity: f32 = 4.5;

    let mut integral: f32 = 0.0;
    let mut last_error: f32 = 0.0;
    loop {
        let value =
            approx_value.load(Ordering::Relaxed) + pcnt_driver.get_counter_value().unwrap() as i32;

        let velocity = (value - last_value) as f32 / CPR / (0.05);

        let error = target_velocity - velocity;
        integral += error * 0.05;
        let u = k_p * error + k_i * integral + k_d * ((error - last_error) / 0.05);
        speed_control += u;

        motor_driver.set_duty((max_duty as f32 * speed_control.clamp(0.0, 1.0)) as u32).unwrap();

        println!("velocity: {:.2}", velocity);

        last_value = value;
        last_error = error;

        FreeRtos::delay_ms(50u32); // 0.1 s
    }
}
