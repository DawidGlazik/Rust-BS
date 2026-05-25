#![deny(unsafe_code)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

pub const SPEED_LIMIT: f32 = 25.0;
pub const TICK_RATE_MS: u32 = 10;
const BATTERY_DEGRADATION_TICKS: u32 = 100_000 / TICK_RATE_MS;
const PASSIVE_DECREASE_TICKS: u32 = 5_000 / TICK_RATE_MS;
const MAX_POWER_LIMIT: u8 = 100;
const MAX_PAS_VALUE: u8 = 100;
const MAX_BRAKE_VALUE: u8 = 100;
const PAS_FACTOR_SCALE: f32 = 100.0;
const BRAKE_FACTOR_SCALE: f32 = 100.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemState {
    Idle,
    AssistOn,
    AssistPassive,
    EmergencyBraking,
    Fault,
}

#[derive(Debug, Clone, Copy)]
pub struct SensorInputs {
    pub pas_value: u8,
    pub pas_err: bool,
    pub brake_value: u8,
    pub brake_err: bool,
    pub speed_value: f32,
    pub speed_err: bool,
    pub battery_ok: bool,
    pub battery_err: bool,
}

impl Default for SensorInputs {
    fn default() -> Self {
        Self {
            pas_value: 0,
            pas_err: false,
            brake_value: 0,
            brake_err: false,
            speed_value: 0.0,
            speed_err: false,
            battery_ok: true,
            battery_err: false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SystemOutputs {
    pub motor_power: u8,
    pub status_led: bool,
}

pub trait Logger {
    fn log_event(&mut self, message: &'static str);
    fn log_state_change(&mut self, from: SystemState, to: SystemState);
}

pub struct BikeEngineController {
    state: SystemState,
    current_motor_power: u8,
    max_motor_power: u8,
    battery_timer_ticks: u32,
    passive_timer_ticks: u32,
}

impl BikeEngineController {
    pub fn current_state(&self) -> SystemState {
        self.state
    }
    pub fn max_motor_power(&self) -> u8 {
        self.max_motor_power
    }
    pub fn battery_timer_ticks(&self) -> u32 {
        self.battery_timer_ticks
    }
    pub fn passive_timer_ticks(&self) -> u32 {
        self.passive_timer_ticks
    }
}

impl Default for BikeEngineController {
    fn default() -> Self {
        Self::new()
    }
}

impl BikeEngineController {
    pub fn new() -> Self {
        Self {
            state: SystemState::Idle,
            current_motor_power: 0,
            max_motor_power: MAX_POWER_LIMIT,
            battery_timer_ticks: 0,
            passive_timer_ticks: 0,
        }
    }

    pub fn update(&mut self, inputs: &SensorInputs, logger: &mut dyn Logger) -> SystemOutputs {
        let valid_inputs = self.validate_inputs(inputs, logger);

        if self.check_global_fault(&valid_inputs) {
            self.transition_to(SystemState::Fault, logger);
        } else if valid_inputs.brake_value == MAX_BRAKE_VALUE && self.state != SystemState::EmergencyBraking {
            self.transition_to(SystemState::EmergencyBraking, logger);
        } else {
            self.evaluate_normal_transitions(&valid_inputs, logger);
        }

        self.execute_state_actions(&valid_inputs);

        SystemOutputs {
            motor_power: self.current_motor_power,
            status_led: self.state == SystemState::Fault,
        }
    }

    fn validate_inputs(&self, inputs: &SensorInputs, logger: &mut dyn Logger) -> SensorInputs {
        let mut clean = *inputs;
        if clean.pas_value > MAX_PAS_VALUE {
            logger.log_event("Ostrzezenie: Przekroczono zakres PAS_Value, obcieto do MAX_PAS_VALUE");
            clean.pas_value = MAX_PAS_VALUE;
        }
        if clean.brake_value > MAX_BRAKE_VALUE {
            logger.log_event("Ostrzezenie: Przekroczono zakres Brake_Value, obcieto do MAX_BRAKE_VALUE");
            clean.brake_value = MAX_BRAKE_VALUE;
        }
        if clean.speed_value < 0.0 || !clean.speed_value.is_finite() {
            logger.log_event("Ostrzezenie: Nieprawidlowa predkosc, obcieto do 0.0");
            clean.speed_value = 0.0;
        }
        clean
    }

    fn check_global_fault(&self, inputs: &SensorInputs) -> bool {
        inputs.pas_err || inputs.brake_err || inputs.speed_err || inputs.battery_err
    }

    fn transition_to(&mut self, new_state: SystemState, logger: &mut dyn Logger) {
        if self.state != new_state {
            logger.log_state_change(self.state, new_state);
            self.state = new_state;
            self.passive_timer_ticks = 0;
        }
    }

    fn evaluate_normal_transitions(&mut self, inputs: &SensorInputs, logger: &mut dyn Logger) {
        match self.state {
            SystemState::Idle => {
                if inputs.pas_value > 0 && inputs.brake_value < MAX_BRAKE_VALUE && inputs.speed_value < SPEED_LIMIT {
                    self.transition_to(SystemState::AssistOn, logger);
                }
            }
            SystemState::AssistOn => {
                if inputs.speed_value >= SPEED_LIMIT || inputs.pas_value == 0 {
                    self.transition_to(SystemState::AssistPassive, logger);
                }
            }
            SystemState::AssistPassive => {
                if self.current_motor_power == 0 {
                    self.transition_to(SystemState::Idle, logger);
                } else if inputs.speed_value < SPEED_LIMIT && inputs.pas_value > 0 {
                    self.transition_to(SystemState::AssistOn, logger);
                }
            }
            SystemState::EmergencyBraking => {
                if inputs.brake_value == 0 {
                    self.transition_to(SystemState::Idle, logger);
                }
            }
            SystemState::Fault => {}
        }
    }

    fn execute_state_actions(&mut self, inputs: &SensorInputs) {
        if !inputs.battery_ok && (self.state == SystemState::AssistOn || self.state == SystemState::AssistPassive) {
            self.battery_timer_ticks = self.battery_timer_ticks.saturating_add(1);
            if self.battery_timer_ticks >= BATTERY_DEGRADATION_TICKS {
                self.max_motor_power = self.max_motor_power.saturating_sub(1);
                self.battery_timer_ticks = 0;
            }
        } else {
            self.battery_timer_ticks = 0;
        }

        match self.state {
            SystemState::Idle => {
                self.current_motor_power = 0;
            }
            SystemState::AssistOn => {
                let p_max = self.max_motor_power as f32;
                let pas_factor = inputs.pas_value as f32 / PAS_FACTOR_SCALE;
                let brake_factor = (BRAKE_FACTOR_SCALE - inputs.brake_value as f32) / BRAKE_FACTOR_SCALE;
                self.current_motor_power = (p_max * pas_factor * brake_factor) as u8;
            }
            SystemState::AssistPassive => {
                self.passive_timer_ticks = self.passive_timer_ticks.saturating_add(1);
                if self.passive_timer_ticks >= PASSIVE_DECREASE_TICKS {
                    self.current_motor_power = self.current_motor_power.saturating_sub(1);
                    self.passive_timer_ticks = 0;
                }
                let p_max = self.max_motor_power as f32;
                let brake_factor = (BRAKE_FACTOR_SCALE - inputs.brake_value as f32) / BRAKE_FACTOR_SCALE;
                let braked_power = (p_max * brake_factor) as u8;
                self.current_motor_power = self.current_motor_power.min(braked_power);
            }
            SystemState::EmergencyBraking | SystemState::Fault => {
                self.current_motor_power = 0;
            }
        }
    }
}

struct ConsoleLogger;

impl Logger for ConsoleLogger {
    fn log_event(&mut self, message: &'static str) {
        println!("[ZDARZENIE] {}", message);
    }

    fn log_state_change(&mut self, from: SystemState, to: SystemState) {
        println!("[ZMIANA STANU] {:?} -> {:?}", from, to);
    }
}

fn bool_str(v: bool) -> &'static str { if v { "tak" } else { "nie" } }

fn print_virtual_time_row(label: &str, ticks: u64) {
    let total_ms = ticks * TICK_RATE_MS as u64;
    print!("  | {:<30} : ", label);
    if total_ms < 1_000 {
        print!("{:<22} ms |", total_ms);
    } else if total_ms < 60_000 {
        print!("{:<22.3} s |", total_ms as f64 / 1_000.0);
    } else {
        let mins = total_ms / 60_000;
        let secs = (total_ms % 60_000) as f64 / 1_000.0;
        print!("{:<4} min {:<13.1} s |", mins, secs);
    }
    println!();
}

fn print_status(
    step: u32,
    inputs: &SensorInputs,
    outputs: &SystemOutputs,
    controller: &BikeEngineController,
    total_ticks: u64,
    state_since_ticks: u64,
) {
    let state_label = match controller.current_state() {
        SystemState::Idle             => "Idle",
        SystemState::AssistOn         => "AssistOn",
        SystemState::AssistPassive    => "AssistPassive",
        SystemState::EmergencyBraking => "EmergencyBraking",
        SystemState::Fault            => "Fault",
    };

    let led_str = if outputs.status_led { "SWIECI (awaria)" } else { "nie swieci (OK)" };

    println!("  +------------------------------------------------------------+");
    println!("  | RAPORT STANU  -  krok {:<33} |", step);
    println!("  +--------------------[ WEJSCIA ]-----------------------------+");
    println!("  | PAS value                      : {:<25} |", inputs.pas_value);
    println!("  | PAS error                      : {:<25} |", bool_str(inputs.pas_err));
    println!("  | Hamulec value                  : {:<25} |", inputs.brake_value);
    println!("  | Hamulec error                  : {:<25} |", bool_str(inputs.brake_err));
    println!("  | Predkosc value                 : {:<18.2} km/h |", inputs.speed_value);
    println!("  | Predkosc error                 : {:<25} |", bool_str(inputs.speed_err));
    println!("  | Bateria OK                     : {:<25} |", bool_str(inputs.battery_ok));
    println!("  | Bateria error                  : {:<25} |", bool_str(inputs.battery_err));
    println!("  +--------------------[ WYJSCIA ]-----------------------------+");
    println!("  | Moc silnika                    : {:>3}/{:<21} |", outputs.motor_power, controller.max_motor_power());
    println!("  | LED Awarii                     : {:<25} |", led_str);
    println!("  +--------------------[ STAN ]--------------------------------+");
    println!("  | Stan automatu                  : {:<25} |", state_label);
    println!("  | Licznik baterii                : {:<19} tikow |", controller.battery_timer_ticks());
    println!("  | Licznik wspomagania pasywnego  : {:<19} tikow |", controller.passive_timer_ticks());
    print_virtual_time_row("Czas symulacji", total_ticks);
    print_virtual_time_row("Czas w aktualnym stanie", total_ticks - state_since_ticks);
    println!("  +------------------------------------------------------------+");
}

fn simulate(
    controller: &mut BikeEngineController,
    logger: &mut dyn Logger,
    inputs: &SensorInputs,
    ticks: u32,
    total_ticks: &mut u64,
    state_since_ticks: &mut u64,
    prev_state: &mut SystemState,
) -> SystemOutputs {
    let mut last_outputs = SystemOutputs { motor_power: 0, status_led: false };
    for _ in 0..ticks {
        last_outputs = controller.update(inputs, logger);
        *total_ticks = total_ticks.saturating_add(1);
        let cur = controller.current_state();
        if cur != *prev_state {
            *state_since_ticks = *total_ticks;
            *prev_state = cur;
        }
    }
    last_outputs
}

fn main() {
    let mut controller = BikeEngineController::new();
    let mut logger = ConsoleLogger;

    let mut total_ticks: u64 = 0;
    let mut state_since_ticks: u64 = 0;
    let mut prev_state = controller.current_state();

    println!("--- ROZPOCZECIE SYMULACJI SYSTEMU KONTROLI SILNIKA ELEKTRYCZNEGO DO ROWERU ---");
    println!("    (1 tik = {} ms czasu symulacji)", TICK_RATE_MS);

    println!("\nKrok 1: System gotowy. Brak ruchu. [100 ms]");
    let inputs = SensorInputs::default();
    let outputs = simulate(&mut controller, &mut logger, &inputs, 10,
                           &mut total_ticks, &mut state_since_ticks, &mut prev_state);
    print_status(1, &inputs, &outputs, &controller, total_ticks, state_since_ticks);

    println!("\nKrok 2: Rowerzysta naciska na pedaly. Predkosc rosnie. [5 s]");
    let inputs = SensorInputs { pas_value: 50, speed_value: 15.0, ..Default::default() };
    let outputs = simulate(&mut controller, &mut logger, &inputs, 500,
                           &mut total_ticks, &mut state_since_ticks, &mut prev_state);
    print_status(2, &inputs, &outputs, &controller, total_ticks, state_since_ticks);

    println!("\nKrok 3: Rower rozpedza sie powyzej limitu 25 km/h. [5 s]");
    let inputs = SensorInputs { pas_value: 50, speed_value: 28.0, ..Default::default() };
    let outputs = simulate(&mut controller, &mut logger, &inputs, 500,
                           &mut total_ticks, &mut state_since_ticks, &mut prev_state);
    print_status(3, &inputs, &outputs, &controller, total_ticks, state_since_ticks);

    println!("\nKrok 3b: Utrzymanie predkosci > 25 km/h przez kolejne 20 sekund. [20 s]");
    let outputs = simulate(&mut controller, &mut logger, &inputs, 2000,
                           &mut total_ticks, &mut state_since_ticks, &mut prev_state);
    print_status(30, &inputs, &outputs, &controller, total_ticks, state_since_ticks);

    println!("\nKrok 4: Nagle, pelne wcisniecie hamulca! [200 ms]");
    let inputs = SensorInputs { pas_value: 80, brake_value: 100, speed_value: 20.0, ..Default::default() };
    let outputs = simulate(&mut controller, &mut logger, &inputs, 20,
                           &mut total_ticks, &mut state_since_ticks, &mut prev_state);
    print_status(4, &inputs, &outputs, &controller, total_ticks, state_since_ticks);

    println!("\nKrok 5: Rowerzysta zwalnia hamulec, brak pedalowania. [1 s]");
    let inputs = SensorInputs { brake_value: 0, pas_value: 0, speed_value: 0.0, ..Default::default() };
    let outputs = simulate(&mut controller, &mut logger, &inputs, 100,
                           &mut total_ticks, &mut state_since_ticks, &mut prev_state);
    print_status(5, &inputs, &outputs, &controller, total_ticks, state_since_ticks);

    println!("\nKrok 6: Uszkodzenie czujnika PAS! [100 ms]");
    let inputs = SensorInputs { pas_err: true, ..Default::default() };
    let outputs = simulate(&mut controller, &mut logger, &inputs, 10,
                           &mut total_ticks, &mut state_since_ticks, &mut prev_state);
    print_status(6, &inputs, &outputs, &controller, total_ticks, state_since_ticks);

    println!("\nKrok 7: Proba jazdy pomimo uszkodzenia (system zatrzasniety). [30 s]");
    let inputs = SensorInputs { pas_value: 100, pas_err: false, ..Default::default() };
    let outputs = simulate(&mut controller, &mut logger, &inputs, 3_000,
                           &mut total_ticks, &mut state_since_ticks, &mut prev_state);
    print_status(7, &inputs, &outputs, &controller, total_ticks, state_since_ticks);
}
