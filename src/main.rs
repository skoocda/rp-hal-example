// # Pico Game Controller
//
// Sends HID Joystick packets based on GPIO input.
//
// This will blink an LED attached to GP25, which is the pin the Pico uses for
// the on-board LED, based on a button hooked up to GP15. The
// button should cause the line to be grounded, as the input pin is pulled high
// internally by this example. When the button is pressed, the LED will blink
// faster, and the Pico will send the button press to the host
#![no_std]
#![no_main]

// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::digital::{InputPin, OutputPin};
// Embedded HAL 1.0.0 doesn't have an ADC trait, so use the one from 0.2
use embedded_hal_0_2::adc::OneShot;
use embedded_hal_nb::serial::Write as UartWrite;

use rp_pico::hal::{
    self,
    Timer,
    pac::{self, interrupt},
    //prelude::*,
    adc::{Adc, AdcPin},
    gpio::{self, Pin, DynPinId, PullUp, FunctionSioInput, 
        bank0::{Gpio0, Gpio1, Gpio26, Gpio27}
    },
    uart::{DataBits, StopBits, UartConfig}
};
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
//use usbd_serial::SerialPort;

// We also need this for the 'Delay' object to work.
use rp2040_hal::Clock;

// Time handling traits
use fugit::RateExtU32;

// USB Human Interface Device (HID) Class support
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::hid_class::HIDClass;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

// The writeln! trait.
use core::fmt::Write;

// Some short-cuts to useful types
use core::cell::RefCell;
use critical_section::Mutex;
use heapless::spsc::Queue;

mod button;
mod thumbstick;
mod vector;
mod gyro;
mod imu;
mod hid;
// use crate::button::ButtonManager;

// Used to demonstrate writing formatted strings
// use core::fmt::Write;
// use heapless::String;

/// Alias the type for our UART pins to make things clearer.
type UartPins = (
    gpio::Pin<Gpio0, gpio::FunctionUart, gpio::PullNone>,
    gpio::Pin<Gpio1, gpio::FunctionUart, gpio::PullNone>,
);
/// Alias the type for our ADC pins to make things clearer.

type AdcPins = (
    AdcPin<Pin<Gpio26, gpio::FunctionSio<gpio::SioInput>, gpio::PullNone>>,
    AdcPin<Pin<Gpio27, gpio::FunctionSio<gpio::SioInput>, gpio::PullNone>>,
);

type ButtonPins = [Pin<DynPinId, FunctionSioInput, PullUp>; 8];

/// Alias the type for our UART to make things clearer.
type Uart = hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>;

/// This describes the queue we use for outbound UART data
struct UartQueue {
    mutex_cell_queue: Mutex<RefCell<Queue<u8, 64>>>,
    interrupt: pac::Interrupt,
}

/// This is our outbound UART queue. We write to it from the main thread, and
/// read from it in the UART IRQ.
static UART_TX_QUEUE: UartQueue = UartQueue {
    mutex_cell_queue: Mutex::new(RefCell::new(Queue::new())),
    interrupt: hal::pac::Interrupt::UART0_IRQ,
};


/// This how we transfer the UART into the Interrupt Handler
static GLOBAL_UART: Mutex<RefCell<Option<Uart>>> = Mutex::new(RefCell::new(None));


#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = JOYSTICK) = {
        (collection = APPLICATION, usage = POINTER) = {
            (usage = X,) = {
                #[item_settings data,variable,absolute] x=input;
            };
            (usage = Y,) = {
                #[item_settings data,variable,absolute] y=input;
            };
            // (usage = 0x33,) = {
            //     #[item_settings data,variable,absolute] rx=input;
            // };
            // (usage = 0x34,) = {
            //     #[item_settings data,variable,absolute] ry=input;
            // };
        };
        (usage_page = BUTTON, usage_min = BUTTON_1, usage_max = BUTTON_8) = {
            #[packed_bits 8] #[item_settings data,variable,absolute] buttons=input;
        }
    }
)]
#[allow(dead_code)]
pub struct JoystickReport {
    pub x: i8,
    pub y: i8,
    pub buttons: u8,
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then runs an infinite loop
/// sampling button states and sending reports to the host
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();    

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();


    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    
    // The delay object lets us wait for specified amounts of milliseconds
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();


    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.reconfigure(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.reconfigure(),
    );
    // Make a UART on the given pins
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    // Tell the UART to raise its interrupt line on the NVIC when the TX FIFO
    // has space in it.
    uart.enable_tx_interrupt();

    // Now we give away the entire UART peripheral, via the variable
    // `GLOBAL_UART`. We can no longer access the UART from this main thread.
    critical_section::with(|cs| {
        GLOBAL_UART.borrow(cs).replace(Some(uart));
    });

    let mut button_pins = [
        pins.gpio14.into_pull_up_input().into_dyn_pin(),  // A
        pins.gpio15.into_pull_up_input().into_dyn_pin(),  // B
        
        pins.gpio16.into_pull_up_input().into_dyn_pin(),  // X
        pins.gpio17.into_pull_up_input().into_dyn_pin(),  // Y

        pins.gpio6.into_pull_up_input().into_dyn_pin(),  // Up
        pins.gpio7.into_pull_up_input().into_dyn_pin(),  // Down
        pins.gpio8.into_pull_up_input().into_dyn_pin(),  // Left
        pins.gpio9.into_pull_up_input().into_dyn_pin(),  // Right
        // Need to revise the Descriptor macro before expanding this to multi-byte
        // pins.gpio8.into_pull_up_input().into_dyn_pin(),  // Start
        // pins.gpio9.into_pull_up_input().into_dyn_pin(),  // Select
        // pins.gpio10.into_pull_up_input().into_dyn_pin(), // L Shoulder
        // pins.gpio11.into_pull_up_input().into_dyn_pin(), // R Shoulder
        // pins.gpio12.into_pull_up_input().into_dyn_pin(), // L Stick Click
        // pins.gpio13.into_pull_up_input().into_dyn_pin(), // R Stick Click
    ];

    // Enable adc
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    // Configure the pins as an ADC input
    let adc_pin_0 = AdcPin::new(pins.gpio26.into_floating_input()).unwrap();
    let adc_pin_1 = AdcPin::new(pins.gpio27.into_floating_input()).unwrap();
    adc.free_running(&adc_pin_0);
    adc.free_running(&adc_pin_1);

    let mut adc_pins: AdcPins = (adc_pin_0, adc_pin_1);

    // let mut dpad_pins = [
    //     pins.gpio16.into_pull_up_input().into_dyn_pin(),  // Left X
    //     pins.gpio17.into_pull_up_input().into_dyn_pin(),  // Left Y
    //     //pins.gpio18.into_pull_up_input().into_dyn_pin(),  // Right X
    //     //pins.gpio19.into_pull_up_input().into_dyn_pin(),  // Right Y
    // ];
    
    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }
        // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB HID Class Device driver, providing Mouse Reports
    let usb_hid = HIDClass::new(bus_ref, JoystickReport::desc(), 60);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        USB_HID = Some(usb_hid);
    }
    
    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27da))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Twitchy Joystick")
            .serial_number("TEST")])
        .unwrap()
        .device_class(0)
        .build();
    
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    }

    writeln!(
        &UART_TX_QUEUE,
        "Booting up the Suri Alpakka system! \
        Please enjoy this smol poem as you wait: \n
        
She is harnessed for a long journey; on her back she carries an entire store of wool.
She walks without rest, and sees with eyes full of strangeness. 
The wool merchant has forgotten to come to get her, and she is ready.
In this world, nothing comes better equipped than the alpaca; one is more burdened with rags than the next. 
Her sky-high softness is such that if a newborn is placed on her back, he will not feel a bone of the animal.
The weather is very hot. Today, large scissors that will cut and cut represent mercy for the alpaca.
When something is lost in the park, to whom do we look but this ever-prepared beast which seems to secretly carry all things?
And when children think about the objects they have lost— dolls, teddy bears, flying rats, trees with seven voices (they can be hidden in only one place)— they remember the alpaca, their infinitely prepared companion.
But look at those eyes, those astonished eyes without knowledge; they only ask why she has been harnessed for such a long trip and why no one comes to relieve her.
The high plateau is to blame for this tragedy— the mother alpaca incessantly stares at it. 
The mountain was also casting off burdens, and so its summit became clear, and filled the eyes of the mother alpaca.
She was taken down from the plateau and situated near a nonsensical horizon, and when she turns her neck, she continues looking for the older alpaca, for the one who sheds a pack on high, and returns to the sun's radiance.
What have you and I done to our Andean cordillera? I ask the alpaca.
        "
    )
    .unwrap();

    // let mut button_manager = setup_buttons::<Pin<DynPinId, FunctionSioInput, PullUp>>(pins, timer);

    loop {
        led_pin.set_high().unwrap();
        let state = get_joystick_report(&mut button_pins, &mut adc, &mut adc_pins);
        push_gamepad_report(state).ok().unwrap_or(0);
        let time = timer.get_counter().ticks();

        for (i, button) in button_pins.iter_mut().enumerate() {
            if button.is_low().unwrap() {
                writeln!( &UART_TX_QUEUE,"{}: {}", time, i).unwrap();
            }
        }
        delay.delay_ms(5);

        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

fn get_joystick_report(
    button_pins: &mut ButtonPins, 
    adc: &mut Adc, 
    adc_pins: &mut AdcPins,
    )-> JoystickReport {
    let buttons = get_button_state(button_pins);
    let (x, y,) = get_analog_state(adc, adc_pins);
    JoystickReport { buttons, x, y}
}

fn get_button_state(button_pins: &mut ButtonPins) 
    -> u8 {
    let mut button_bits = 0;
    for (idx, pin) in button_pins[..8].iter_mut().enumerate() {
        if pin.is_low().unwrap() {
            button_bits |= 1 << idx;
        }
    }
    button_bits
}

fn get_analog_state(adc: &mut Adc, adc_pins: &mut AdcPins) 
    -> (i8, i8) {
    let (p0, p1) = adc_pins;

    let p0_state: u16 = adc.read(p0).unwrap_or(0);
    let p1_state: u16 = adc.read(p1).unwrap_or(0);

    let p0_converted = convert_adc_to_i8(p0_state);
    let p1_converted = convert_adc_to_i8(p1_state);

    (p0_converted, p1_converted)
}

fn get_dpad_state(pins: &mut [Pin<DynPinId, FunctionSioInput, PullUp>; 4]) -> (i8, i8,) {
    // We're using digital switches in a D-PAD style configuration
    //    10
    //  8    9
    //    11
    // These are mapped to the limits of an axis
    let x = if pins[0].is_low().unwrap() {
        -127 // left
    } else if pins[1].is_low().unwrap() {
        127 // right
    } else {
        0 // center
    };

    let y = if pins[2].is_low().unwrap() {
        -127 // up
    } else if pins[3].is_low().unwrap() {
        127 // down
    } else {
        0 // center
    };

    (x, y,)
}

/// Submit a new joystick input report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_gamepad_report(report: JoystickReport) -> Result<usize, usb_device::UsbError> {
    critical_section::with(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        USB_HID.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);
}

impl UartQueue {
    /// Try and get some data out of the UART Queue. Returns None if queue empty.
    fn read_byte(&self) -> Option<u8> {
        critical_section::with(|cs| {
            let cell_queue = self.mutex_cell_queue.borrow(cs);
            let mut queue = cell_queue.borrow_mut();
            queue.dequeue()
        })
    }

    /// Peek at the next byte in the queue without removing it.
    fn peek_byte(&self) -> Option<u8> {
        critical_section::with(|cs| {
            let cell_queue = self.mutex_cell_queue.borrow(cs);
            let queue = cell_queue.borrow_mut();
            queue.peek().cloned()
        })
    }

    /// Write some data to the queue, spinning until it all fits.
    fn write_bytes_blocking(&self, data: &[u8]) {
        // Go through all the bytes we need to write.
        for byte in data.iter() {
            // Keep trying until there is space in the queue. But release the
            // mutex between each attempt, otherwise the IRQ will never run
            // and we will never have space!
            let mut written = false;
            while !written {
                // Grab the mutex, by turning interrupts off. NOTE: This
                // doesn't work if you are using Core 1 as we only turn
                // interrupts off on one core.
                critical_section::with(|cs| {
                    // Grab the mutex contents.
                    let cell_queue = self.mutex_cell_queue.borrow(cs);
                    // Grab mutable access to the queue. This can't fail
                    // because there are no interrupts running.
                    let mut queue = cell_queue.borrow_mut();
                    // Try and put the byte in the queue.
                    if queue.enqueue(*byte).is_ok() {
                        // It worked! We must have had space.
                        if !pac::NVIC::is_enabled(self.interrupt) {
                            unsafe {
                                // Now enable the UART interrupt in the *Nested
                                // Vectored Interrupt Controller*, which is part
                                // of the Cortex-M0+ core. If the FIFO has space,
                                // the interrupt will run as soon as we're out of
                                // the closure.
                                pac::NVIC::unmask(self.interrupt);
                                // We also have to kick the IRQ in case the FIFO
                                // was already below the threshold level.
                                pac::NVIC::pend(self.interrupt);
                            }
                        }
                        written = true;
                    }
                });
            }
        }
    }
}

impl core::fmt::Write for &UartQueue {
    /// This function allows us to `writeln!` on our global static UART queue.
    /// Note we have an impl for &UartQueue, because our global static queue
    /// is not mutable and `core::fmt::Write` takes mutable references.
    fn write_str(&mut self, data: &str) -> core::fmt::Result {
        self.write_bytes_blocking(data.as_bytes());
        Ok(())
    }
}
#[interrupt]
fn UART0_IRQ() {
    // This variable is special. It gets mangled by the `#[interrupt]` macro
    // into something that we can access without the `unsafe` keyword. It can
    // do this because this function cannot be called re-entrantly. We know
    // this because the function's 'real' name is unknown, and hence it cannot
    // be called from the main thread. We also know that the NVIC will not
    // re-entrantly call an interrupt.
    static mut UART: Option<hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>> =
        None;

    // This is one-time lazy initialisation. We steal the variable given to us
    // via `GLOBAL_UART`.
    if UART.is_none() {
        critical_section::with(|cs| {
            *UART = GLOBAL_UART.borrow(cs).take();
        });
    }

    // Check if we have a UART to work with
    if let Some(uart) = UART {
        // Check if we have data to transmit
        while let Some(byte) = UART_TX_QUEUE.peek_byte() {
            if uart.write(byte).is_ok() {
                // The UART took it, so pop it off the queue.
                let _ = UART_TX_QUEUE.read_byte();
            } else {
                break;
            }
        }

        if UART_TX_QUEUE.peek_byte().is_none() {
            pac::NVIC::mask(UART_TX_QUEUE.interrupt);
        }
    }

    // Set an event to ensure the main thread always wakes up, even if it's in
    // the process of going to sleep.
    cortex_m::asm::sev();
}


fn convert_adc_to_i8(adc_value: u16) -> i8 {
    // Shift the 12-bit value to the right by 4 bits to get the 8 most significant bits
    let msb_8 = (adc_value >> 4) as u8;
    
    // Convert the u8 to i8, treating the most significant bit as the sign bit
    msb_8 as i8
}

// End of file
