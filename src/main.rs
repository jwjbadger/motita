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

enum Message {
    CW,
    CCW,
    TIMER,
    ERR,
}

impl From<NonZeroU32> for Message {
    fn from(bitset: NonZeroU32) -> Self {
        match bitset.get() {
            0b10000000000 => Message::CW,
            0b01000000000 => Message::CCW,
            0b00010000000 => Message::TIMER,
            _ => Message::ERR,
        }
    }
}

impl From<Message> for NonZeroU32 {
    fn from(bitset: Message) -> Self {
        Self::new(match bitset {
            Message::CW => 0b10000000000,
            Message::CCW => 0b01000000000,
            Message::TIMER => 0b00010000000,
            Message::ERR => 0b00000000000,
        }).unwrap()
    }
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let timer_conf = config::Config::new().auto_reload(true);
    let mut timer = TimerDriver::new(peripherals.timer00, &timer_conf).unwrap();

    // 60 Hz PID
    timer.set_alarm(timer.tick_hz() / FREQUENCY).unwrap();

    let notification = Notification::new();

    let motor_notifier = notification.notifier();
    let cw_notifier = motor_notifier.clone();
    let ccw_notifier = motor_notifier.clone();


    unsafe {
        timer
            .subscribe(move || {
                motor_notifier.notify_and_yield(Message::TIMER.into());
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

    cw_trigger
        .set_interrupt_type(InterruptType::AnyEdge)
        .unwrap();
    ccw_trigger
        .set_interrupt_type(InterruptType::AnyEdge)
        .unwrap();

    unsafe {
        cw_trigger
            .subscribe(move || {
                cw_notifier.notify_and_yield(
                        Message::CW.into()
                );
            })
            .unwrap();
        ccw_trigger
            .subscribe(move || {
                ccw_notifier.notify_and_yield(
                        Message::CCW
                    .into(),
                );
            })
            .unwrap();
    }

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

    let mut dir = Direction::CW;
    let mut enable = false;
    let mut integral: f32 = 0.0;

    loop {
        let bitset = notification.wait(esp_idf_hal::delay::BLOCK).unwrap();

        match Message::from(bitset) {
            Message::CW => {
                dir = Direction::CW;
                enable = cw_trigger.is_high();
            }
            Message::CCW => {
                dir = Direction::CCW;
                enable = ccw_trigger.is_high();
            }
            Message::TIMER => {
                let dir = match dir {
                    Direction::CW => 1.0,
                    Direction::CCW => -1.0,
                };

                integral += (motor_a_controller.get_velocity_weak(PERIOD) - motor_b_controller.get_velocity_weak(PERIOD)) * PERIOD;

                if enable {
                    motor_a_controller.step_towards_with_integral(SET_SPEED * dir, PERIOD, -1.0 * integral);
                    motor_b_controller.step_towards_with_integral(SET_SPEED * dir, PERIOD, integral);
                } else {
                    // TODO: does this actually stop the motor
                    motor_a_controller.halt();
                    motor_b_controller.halt();
                }
            }
            _ => {}
        }

    }
}
