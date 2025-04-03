use esp_idf_hal::{
    delay::FreeRtos,
    gpio::PinDriver,
    peripherals::Peripherals,
};

use motita::motor::*;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    // to be removed
    let mut high = PinDriver::output(peripherals.pins.gpio25).unwrap();
    high.set_high().unwrap();

    let mut motor_a = MotorController::new(
        peripherals.ledc,
        MotorControllerConfig {
            pwm_pin: peripherals.pins.gpio18.into(),
            pcnt: peripherals.pcnt0,
            ch_a: peripherals.pins.gpio5.into(),
            ch_b: peripherals.pins.gpio4.into(),
        } 
    );

    let mut motor_a_controller = PIDController::new(&mut motor_a);

    loop {
        let velocity = motor_a_controller.get_velocity(0.05);

        println!("velocity: {:.2}", velocity);

        motor_a_controller.step_towards(5.0, 0.05);

        FreeRtos::delay_ms(50u32); // 0.05 s
        break;
    }
}
