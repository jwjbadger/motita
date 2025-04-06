use esp_idf_hal::{
    gpio::{InterruptType, PinDriver, Pull},
    peripherals::Peripherals,
    task::notification::Notification,
    timer::*,
};

use std::num::NonZeroU32;

use motita::motor::*;

const SET_SPEED: f32 = 8.0;
const FREQUENCY: u64 = 60;
const PERIOD: f32 = 1.0 / FREQUENCY as f32;
const BITSET: NonZeroU32 = NonZeroU32::new(0b10001010101).unwrap();

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let timer_conf = config::Config::new().auto_reload(true);
    let mut timer = TimerDriver::new(peripherals.timer00, &timer_conf).unwrap();

    // 60 Hz PID
    timer.set_alarm(timer.tick_hz() / FREQUENCY).unwrap();

    let notification = Notification::new();
    let notifier = notification.notifier();

    unsafe {
        timer
            .subscribe(move || {
                notifier.notify_and_yield(BITSET);
            })
            .unwrap();
    }

    // used as control voltage throughout the circuit to drive transistors, etc.
    let mut control = PinDriver::output(peripherals.pins.gpio25).unwrap();
    control.set_high().unwrap();

    let mut cw_trigger = PinDriver::input(peripherals.pins.gpio32).unwrap();
    let mut ccw_trigger = PinDriver::input(peripherals.pins.gpio33).unwrap();

    cw_trigger.set_pull(Pull::Down).unwrap();
    ccw_trigger.set_pull(Pull::Down).unwrap();

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

    timer.enable_interrupt().unwrap();
    timer.enable_alarm(true).unwrap();
    timer.enable(true).unwrap();

    let mut dir = Option::<Direction>::None;
    let mut integral: f32 = 0.0;

    loop {
        if notification.wait(esp_idf_hal::delay::BLOCK).unwrap() != BITSET {
            continue;
        }

        if cw_trigger.is_high() {
            dir = Some(Direction::CW);
        } else if ccw_trigger.is_high() {
            dir = Some(Direction::CCW);
        } else {
            dir = None;
        } 

        let dir = match dir {
            Some(Direction::CW) => 1.0,
            Some(Direction::CCW) => -1.0,
            None => 0.0,
        };

        integral += (motor_a_controller.get_velocity_weak(PERIOD)
            - motor_b_controller.get_velocity_weak(PERIOD))
            * PERIOD;

        let integral = if integral.abs() < 0.01 {integral} else {0.0};

        motor_a_controller.step_towards_with_integral(
            SET_SPEED * dir,
            PERIOD,
            -1.0 * integral,
        );
        motor_b_controller.step_towards_with_integral(
            SET_SPEED * dir,
            PERIOD,
            integral,
        );
    }
}
