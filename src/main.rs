use esp_idf_hal::{delay::FreeRtos, gpio::PinDriver, peripherals::Peripherals};

use motita::motor::*;

const SET_SPEED: f32 = 4.0;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    // to be removed
    let mut high = PinDriver::output(peripherals.pins.gpio25).unwrap();
    high.set_high().unwrap();

    let mut motor_a = MotorController::new(
        peripherals.ledc.timer0,
        peripherals.ledc.channel0,
        MotorControllerConfig {
            pwm_pin: peripherals.pins.gpio18.into(),
            pcnt: peripherals.pcnt0,
            ch_a: peripherals.pins.gpio4.into(),
            ch_b: peripherals.pins.gpio5.into(),
            dir: peripherals.pins.gpio15.into(),
        },
    );

    let mut motor_b = MotorController::new(
        peripherals.ledc.timer1,
        peripherals.ledc.channel1,
        MotorControllerConfig {
            pwm_pin: peripherals.pins.gpio19.into(),
            pcnt: peripherals.pcnt1,
            ch_a: peripherals.pins.gpio22.into(),
            ch_b: peripherals.pins.gpio23.into(),
            dir: peripherals.pins.gpio21.into(),
        },
    );

    let mut motor_a_controller = PIDController::new(&mut motor_a);
    let mut motor_b_controller = PIDController::new(&mut motor_b);

    loop {
        for i in 0..200 {
            let dir = if i > 100 { 1.0 } else { -1.0 };
            motor_a_controller.step_towards(SET_SPEED * dir, 0.05);
            motor_b_controller.step_towards(SET_SPEED * dir, 0.05);

            FreeRtos::delay_ms(50u32); // 0.05 s
        }
    }
}
