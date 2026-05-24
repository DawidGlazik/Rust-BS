#![deny(unsafe_code)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

pub const SPEED_LIMIT: f32 = 25.0;
pub const TICK_RATE_MS: u32 = 10;
const BATTERY_DEGRADATION_TICKS: u32 = 100_000 / TICK_RATE_MS;
const PASSIVE_DECREASE_TICKS: u32 = 5_000 / TICK_RATE_MS;
const MAX_POWER_LIMIT: u8 = 100;

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
        } else if valid_inputs.brake_value == 100 && self.state != SystemState::EmergencyBraking {
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
        if clean.pas_value > 100 {
            logger.log_event("Ostrzezenie: Przekroczono zakres PAS_Value, obcieto do 100");
            clean.pas_value = 100;
        }
        if clean.brake_value > 100 {
            logger.log_event("Ostrzezenie: Przekroczono zakres Brake_Value, obcieto do 100");
            clean.brake_value = 100;
        }
        if clean.speed_value < 0.0 {
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
                if inputs.pas_value > 0 && inputs.brake_value < 100 && inputs.speed_value < SPEED_LIMIT {
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
                let pas_factor = inputs.pas_value as f32 / 100.0;
                let brake_factor = (100.0 - inputs.brake_value as f32) / 100.0;
                self.current_motor_power = (p_max * pas_factor * brake_factor) as u8;
            }
            SystemState::AssistPassive => {
                self.passive_timer_ticks = self.passive_timer_ticks.saturating_add(1);
                if self.passive_timer_ticks >= PASSIVE_DECREASE_TICKS {
                    self.current_motor_power = self.current_motor_power.saturating_sub(1);
                    self.passive_timer_ticks = 0;
                }
                let p_max = self.max_motor_power as f32;
                let brake_factor = (100.0 - inputs.brake_value as f32) / 100.0;
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

fn main() {
    let mut controller = BikeEngineController::new();
    let mut logger = ConsoleLogger;

    println!("--- ROZPOCZECIE SYMULACJI SYSTEMU KONTROLI SILNIKA ELEKTRYCZNEGO DO ROWERU ---");

    println!("\nKrok 1: System gotowy. Brak ruchu.");
    let inputs = SensorInputs::default();
    let outputs = controller.update(&inputs, &mut logger);
    println!("Moc silnika: {}, LED Awarii: {}", outputs.motor_power, outputs.status_led);

    println!("\nKrok 2: Rowerzysta naciska na pedaly. Predkosc rosnie.");
    let inputs = SensorInputs { pas_value: 50, speed_value: 15.0, ..Default::default() };
    let outputs = controller.update(&inputs, &mut logger);
    println!("Moc silnika: {}, LED Awarii: {}", outputs.motor_power, outputs.status_led);

    println!("\nKrok 3: Rower rozpedza sie powyzej limitu 25 km/h.");
    let inputs = SensorInputs { pas_value: 50, speed_value: 28.0, ..Default::default() };
    let outputs = controller.update(&inputs, &mut logger);
    println!("Moc silnika: {}, LED Awarii: {}", outputs.motor_power, outputs.status_led);

    println!("\nKrok 4: Nagle, pelne wcisniecie hamulca!");
    let inputs = SensorInputs { pas_value: 80, brake_value: 100, speed_value: 20.0, ..Default::default() };
    let outputs = controller.update(&inputs, &mut logger);
    println!("Moc silnika: {}, LED Awarii: {}", outputs.motor_power, outputs.status_led);

    println!("\nKrok 5: Rowerzysta zwalnia hamulec, brak pedalowania.");
    let inputs = SensorInputs { brake_value: 0, pas_value: 0, speed_value: 0.0, ..Default::default() };
    let outputs = controller.update(&inputs, &mut logger);
    println!("Moc silnika: {}, LED Awarii: {}", outputs.motor_power, outputs.status_led);

    println!("\nKrok 6: Uszkodzenie czujnika PAS!");
    let inputs = SensorInputs { pas_err: true, ..Default::default() };
    let outputs = controller.update(&inputs, &mut logger);
    println!("Moc silnika: {}, LED Awarii: {}", outputs.motor_power, outputs.status_led);
    
    println!("\nKrok 7: Proba jazdy pomimo uszkodzenia (system zatrzasniety).");
    let inputs = SensorInputs { pas_value: 100, pas_err: false, ..Default::default() };
    let outputs = controller.update(&inputs, &mut logger);
    println!("Moc silnika: {}, LED Awarii: {}", outputs.motor_power, outputs.status_led);
}