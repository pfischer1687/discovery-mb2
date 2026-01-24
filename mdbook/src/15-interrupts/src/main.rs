#![no_main]
#![no_std]

use cortex_m::asm;
use cortex_m_rt::entry;
use critical_section_lock_mut::LockMut;
use embedded_hal::delay::DelayNs;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

use microbit::{
    hal::{
        gpio,
        pac::{self, interrupt},
        pwm::{self, Pwm},
        time::Hertz,
        timer::{self, Timer},
    },
    Board,
};

//
// ===========================
// Configurable Siren Parameters
// ===========================
//

#[derive(Clone, Copy)]
struct SirenConfig {
    base_freq_hz: u32,
    freq_rise_hz: u32,
    rise_time_us: u32,
    control_period_us: u32,
}

impl Default for SirenConfig {
    fn default() -> Self {
        Self {
            base_freq_hz: 440,
            freq_rise_hz: 220,
            rise_time_us: 500_000,
            control_period_us: 20_000,
        }
    }
}

//
// ===========================
// Type Aliases
// ===========================
//

type ControlTimer = timer::Timer<pac::TIMER0>;
type SirenPwm = Pwm<pac::PWM0>;

//
// ===========================
// Siren Driver
// ===========================
//

/// PWM-driven siren with timer-based envelope control.
///
/// Responsibilities:
/// - PWM peripheral generates the square wave
/// - Timer interrupt updates pitch envelope
///
/// Non-responsibilities:
/// - GPIO manipulation
/// - Logging or allocation
/// - Ownership of unrelated peripherals
///
/// Safety & Concurrency:
/// - All mutable state is protected by `LockMut`
/// - ISR execution is bounded and non-blocking
struct Siren {
    timer: ControlTimer,
    pwm: SirenPwm,
    config: SirenConfig,

    /// Elapsed time within the current envelope cycle (µs)
    cur_time_us: u32,
}

impl Siren {
    /// Construct a new siren driver.
    ///
    /// # Preconditions
    /// - PWM peripheral must be fully configured
    /// - TIMER0 interrupt must be mapped to `TIMER0`
    fn new(timer: ControlTimer, pwm: SirenPwm, config: SirenConfig) -> Self {
        Self {
            timer,
            pwm,
            config,
            cur_time_us: 0,
        }
    }

    /// Start siren playback.
    ///
    /// Safe to call multiple times; restarts envelope.
    fn start(&mut self) {
        self.cur_time_us = 0;
        self.apply_frequency();
        self.timer.enable_interrupt();
        self.arm_timer();
    }

    /// Stop siren playback.
    ///
    /// Leaves PWM disabled and control timer inactive.
    fn stop(&mut self) {
        self.timer.disable_interrupt();
        self.pwm.disable();
    }

    /// Control-loop tick handler.
    ///
    /// Called exclusively from the TIMER0 ISR.
    fn on_control_tick(&mut self) {
        self.advance_envelope();
        self.apply_frequency();
        self.arm_timer();
    }

    /// Advance envelope time, wrapping cleanly.
    #[inline(always)]
    fn advance_envelope(&mut self) {
        self.cur_time_us =
            (self.cur_time_us + self.config.control_period_us) % (2 * self.config.rise_time_us);
    }

    /// Compute and apply the current PWM frequency and duty.
    ///
    /// IMPORTANT:
    /// `set_period()` mutates `max_duty`, so duty must
    /// always be reapplied afterward.
    fn apply_frequency(&mut self) {
        let cycle_time = if self.cur_time_us < self.config.rise_time_us {
            self.cur_time_us
        } else {
            2 * self.config.rise_time_us - self.cur_time_us
        };

        let frequency_hz = self.config.base_freq_hz
            + (self.config.freq_rise_hz * cycle_time) / self.config.rise_time_us;

        self.pwm.set_period(Hertz(frequency_hz));

        // Maintain 50% duty cycle invariant
        let duty = self.pwm.max_duty() / 2;
        self.pwm.set_duty_on_common(duty);
    }

    /// Arm the control timer for the next tick.
    #[inline(always)]
    fn arm_timer(&mut self) {
        self.timer.reset_event();
        self.timer.start(self.config.control_period_us);
    }
}

//
// ===========================
// Global State
// ===========================
//

static SIREN: LockMut<Siren> = LockMut::new();

//
// ===========================
// Interrupt Handler
// ===========================
//

#[interrupt]
fn TIMER0() {
    SIREN.with_lock(|siren| siren.on_control_tick());
}

//
// ===========================
// Entry Point
// ===========================
//

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let board = Board::take().unwrap();

    // Speaker GPIO
    let speaker_pin = board
        .speaker_pin
        .into_push_pull_output(gpio::Level::Low)
        .degrade();

    let control_timer = Timer::new(board.TIMER0);
    let mut delay_timer = Timer::new(board.TIMER1);

    // PWM configuration (done once, before handing off)
    let pwm = {
        let pwm = Pwm::new(board.PWM0);
        pwm.set_output_pin(pwm::Channel::C0, speaker_pin)
            .set_prescaler(pwm::Prescaler::Div1);
        pwm.enable();
        pwm
    };

    // Enable TIMER0 interrupt
    //
    // SAFETY:
    // - ISR is defined
    // - Global siren initialized before use
    unsafe { pac::NVIC::unmask(pac::Interrupt::TIMER0) };
    pac::NVIC::unpend(pac::Interrupt::TIMER0);

    // Initialize global siren
    SIREN.init(Siren::new(control_timer, pwm, SirenConfig::default()));

    // Demonstration countdown
    SIREN.with_lock(|s| s.start());
    for t in (1..=10).rev() {
        rprintln!("{}", t);
        delay_timer.delay_ms(1_000);
    }
    rprintln!("launch!");
    SIREN.with_lock(|s| s.stop());

    loop {
        asm::wfi();
    }
}
