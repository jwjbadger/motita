use esp_idf_hal::{
    gpio::{AnyOutputPin, AnyInputPin},
    ledc,
    units,
    pcnt::*,
    peripheral::Peripheral,
};

use std::{
    sync::{
        atomic::{AtomicI32, Ordering},
        Arc,
    },
    cmp::min
};

const LOW_LIMIT: i16 = -1000;
const HIGH_LIMIT: i16 = 1000;

pub struct MotorControllerConfig<N> where N: Peripheral<P: Pcnt> {
    pub pwm_pin: AnyOutputPin,
    pub pcnt: N,
    pub ch_a: AnyInputPin,
    pub ch_b: AnyInputPin
}

pub struct MotorController<'a> {
    max_duty: u32,
    pwm_driver: ledc::LedcDriver<'a>,
    pcnt_driver: PcntDriver<'a>,
    counter: Arc<AtomicI32>,
}

impl<'a> MotorController<'a> {
    pub fn new(ledc: ledc::LEDC, config: MotorControllerConfig<impl Peripheral<P = impl Pcnt> + 'a>) -> Self {
        let mut pwm_driver = ledc::LedcDriver::new(
            ledc.channel0,
            ledc::LedcTimerDriver::new(
                ledc.timer0,
                &ledc::config::TimerConfig {
                    frequency: units::Hertz(1000),
                    resolution: ledc::Resolution::Bits8, 
                },
            )
            .unwrap(),
            config.pwm_pin,
        ).unwrap();

        let max_duty = pwm_driver.get_max_duty();

        pwm_driver.set_duty(0).unwrap();
        pwm_driver.enable().unwrap();

        let mut pcnt_driver = PcntDriver::new(
            config.pcnt,
            Some(config.ch_a),
            Some(config.ch_b),
            Option::<AnyInputPin>::None,
            Option::<AnyInputPin>::None,
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
        let counter = Arc::new(AtomicI32::new(0));

        unsafe {
            let counter = counter.clone();
            pcnt_driver
                .subscribe(move |status| {
                    let status = PcntEventType::from_repr_truncated(status);
                    if status.contains(PcntEvent::HighLimit) {
                        counter.fetch_add(HIGH_LIMIT as i32, Ordering::SeqCst);
                    }
                    if status.contains(PcntEvent::LowLimit) {
                        counter.fetch_add(LOW_LIMIT as i32, Ordering::SeqCst);
                    }
                })
            .unwrap();
            }

        pcnt_driver.event_enable(PcntEvent::HighLimit).unwrap();
        pcnt_driver.event_enable(PcntEvent::LowLimit).unwrap();
        pcnt_driver.counter_pause().unwrap();
        pcnt_driver.counter_clear().unwrap();
        pcnt_driver.counter_resume().unwrap();

        Self {
            max_duty,
            pwm_driver,
            pcnt_driver,
            counter
        }
    }

    pub fn set_speed(&mut self, speed: f32) {
        self.pwm_driver.set_duty((self.max_duty as f32 * speed.clamp(0.0, 1.0)) as u32).unwrap();
    }

    pub fn get_counter(&self) -> i32 {
        self.counter.load(Ordering::SeqCst) + self.pcnt_driver.get_counter_value().unwrap() as i32
    }
}

pub struct PIDController<'a, 'b> {
    motor_controller:  &'a mut MotorController<'b>,
    last_value: i32,
    last_error: f32,
    speed_control: f32,
    k_i: f32,
    k_d: f32,
    k_p: f32,
}

impl<'a, 'b> PIDController<'a, 'b> {
    pub fn new(motor_controller: &'a mut MotorController<'b>) -> Self {
        Self {
            motor_controller,
            last_value: 0,
            last_error: 0.0,
            speed_control: 0.0,
            k_i: 0.005,
            k_d: 0.00003,
            k_p: 0.09,
        }
    }

    pub fn get_velocity(&mut self, dt: f32) -> f32 {
        let value = self.motor_controller.get_counter();
        let velocity = (value - self.last_value) as f32 / dt;

        self.last_value = self.motor_controller.get_counter();

        velocity
    }

    pub fn step_towards(&mut self, target_velocity: f32, dt: f32) {
        let velocity = self.get_velocity(dt);

        let error = target_velocity - velocity;
        self.speed_control += self.k_p * error + self.k_i * error * dt + self.k_d * ((error - self.last_error) / dt);
        self.last_error = error;

        self.motor_controller.set_speed(self.speed_control);
    }
}
