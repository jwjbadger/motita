use esp_idf_hal::gpio::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::gpio::InterruptType;
use std::num::NonZeroU32;

const CPR: f32 = 500.0;

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let notification = esp_idf_hal::task::notification::Notification::new();
    let notifier = notification.notifier();

    let mut ch_a = PinDriver::input(peripherals.pins.gpio5).unwrap();
    let mut high = PinDriver::output(peripherals.pins.gpio25).unwrap();

    let mut timer = esp_idf_hal::timer::TimerDriver::new(peripherals.timer00, &esp_idf_hal::timer::config::Config::default()).unwrap();
    
    ch_a.set_pull(Pull::Down).unwrap();
    high.set_high().unwrap();

    ch_a.set_interrupt_type(InterruptType::NegEdge).unwrap();

    timer.enable(true).unwrap();

    unsafe {
        ch_a.subscribe(move || {
            notifier.notify_and_yield(NonZeroU32::new(1).unwrap());
        }).unwrap();
    }

    loop {
        ch_a.enable_interrupt().unwrap();
        notification.wait(esp_idf_hal::delay::BLOCK);

        let elapsed = (timer.counter().unwrap() as f32) / 10.0f32.powi(6) / 60f32;
        timer.set_counter(0).unwrap();

        println!("speed: {:#?}!", 1.0 / CPR / elapsed);
    }
}
