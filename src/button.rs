//#![allow(warnings)]

use rp_pico::{
    Pins, 
    hal::{self, Timer}};
use rp_pico::hal::gpio::{Pin, DynPinId, PullUp, FunctionSioInput};

use embedded_hal::digital::InputPin;

pub const CFG_PRESS_DEBOUNCE_MILLIS: u32 = 50;

pub const ACTIONS_LEN: usize = 4;

pub type Actions = [u8; ACTIONS_LEN];

pub struct Button {
    pin: Pin<DynPinId, FunctionSioInput, PullUp>,
    // name: char,
    //mode: ButtonMode,
    is_pressed: bool,
    last_debounce: u64,
    debounce_time: u64,
}

pub struct ButtonManager {
    buttons: [Button; 8],
    timer: Timer,
}

impl ButtonManager {
    pub fn new(
        pins: [Pin<DynPinId, FunctionSioInput, PullUp>; 8],
        timer: Timer,
        debounce_time: u64,
    ) -> Self {
        let buttons = pins.map(|pin| Button {
            pin,
            is_pressed: false,
            debounce_time,
            last_debounce: 0,
        });

        Self { buttons, timer }
    }

    pub fn update(&mut self) -> u8 {
        let current_time = self.timer.get_counter().ticks();
        let mut button_state = 0;

        for (i, button) in self.buttons.iter_mut().enumerate() {
            if button.pin.is_low().unwrap() != button.is_pressed {
                button.last_debounce = current_time;
            }

            if (current_time - button.last_debounce) >= button.debounce_time {
                if button.pin.is_low().unwrap() {
                    button_state |= 1 << i;
                }
                button.is_pressed = button.pin.is_low().unwrap();
            }
        }

        button_state
    }
}

pub fn setup_button_manager(
    pins: [Pin<DynPinId, FunctionSioInput, PullUp>; 8],
    timer: Timer,
) -> ButtonManager {
    let debounce_time: u64 = (crate::button::CFG_PRESS_DEBOUNCE_MILLIS*1000).into();
    ButtonManager::new(pins, timer, debounce_time)
}

// fn init_buttons(pins: Pins) -> Vec<Buttons> {
//     let buttons = [Button; 32];
//     // Our button inputs
//     let mut button_A_pin = pins.gpio15.into_pull_up_input();
//     let button_A = Button::new(button_A_pin, 'A');
//     buttons.push(button_A);
// }



// fn button_handler() {
//     let mut gpio = hal::GPIO::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
//     let button_pin = gpio.pin(15).into_pull_up_input();

//     // In the main loop:
//     if button_pin.is_low().unwrap() {
//         report.buttons |= 1; // Set the first button bit
//     }
// }