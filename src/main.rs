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

// The macro for marking our interrupt functions
use rp_pico::hal::pac::interrupt;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB Communications Class Device support
//use usbd_serial::SerialPort;

// USB Human Interface Device (HID) Class support
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::MouseReport;
//use usbd_hid::descriptor::MouseReport;
use usbd_hid::hid_class::HIDClass;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

mod button;

// Used to demonstrate writing formatted strings
// use core::fmt::Write;
// use heapless::String;


/// GamePadReport describes a report and its companion descriptor than can be used
/// to send HID game pad inputs and button presses to a host.
// #[gen_hid_descriptor(
//     (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = GAMEPAD) = {

//         (usage_page = BUTTON, usage_min = 0x1, usage_max = 0x1F) = {
//             #[packed_bits 32] #[item_settings data,variable,absolute] buttons=input;
//         };

//         (usage_page = GENERIC_DESKTOP,) = {
//             #[packed_bits 64]
//             (usage = X,) = {
//                 #[item_settings data,variable,absolute,null] x=input;
//             };
//             (usage = Y,) = {
//                 #[item_settings data,variable,relative] y=input;
//             };
//             (usage = Z,) = {
//                 #[item_settings data,variable,relative] wheel=input;
//             };
//             (usage = rX,) = {
//                 #[item_settings data,variable,absolute,null] x=input;
//             };
//             (usage = RY,) = {
//                 #[item_settings data,variable,relative] y=input;
//             };
//             (usage = Z,) = {
//                 #[item_settings data,variable,relative] wheel=input;
//             };
//         };


//     }
// )]
// pub struct hid_gamepad_custom_report_t {
//     buttons: u32,
//     lx: i16,
//     ly: i16,
//     rx: i16,
//     ry: i16,
//     lz: i16,
//     rz: i16,
// }

// pub struct xinput_report {
//     report_id: u8,
//     report_size: u8 ,
//     buttons_0: u8,
//     buttons_1: u8,
//     lz: u8,
//     rz: u8,
//     lx: i16,
//     ly: i16,
//     rx: i16,
//     ry: i16,
//     reserved: [u8; 6],
// }

/// JoystickReport describes a report and its companion descriptor that can be
/// used to send joystick movements and button presses to a host.
#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = JOYSTICK) = {
        (collection = APPLICATION, usage = POINTER) = {
            (usage = X,) = {
                #[item_settings data,variable,absolute] x=input;
            };
            (usage = Y,) = {
                #[item_settings data,variable,absolute] y=input;
            };
            (usage = 0x33,) = {
                #[item_settings data,variable,absolute] rx=input;
            };
            (usage = 0x34,) = {
                #[item_settings data,variable,absolute] ry=input;
            };
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
    pub rx: i8,
    pub ry: i8,
    pub buttons: u8,
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

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

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();

    // Our button input
    let mut button_pin = pins.gpio15.into_pull_up_input();

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
    let usb_hid = HIDClass::new(bus_ref, MouseReport::desc(), 60);
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
    };

    let core = pac::CorePeripherals::take().unwrap();    
    
    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());



    // Blink the LED at 1 Hz
    // If button is pressed, blink at 10 Hz 
    loop {
        if button_pin.is_high().unwrap() {
            let on = JoystickReport {
                x: 0,
                y: 0,
                rx: 0,
                ry: 0,
                buttons: 0,
            };
            push_joystick_movement(on).ok().unwrap_or(0);
            led_pin.set_high().unwrap();
            delay.delay_ms(500);
            led_pin.set_low().unwrap();
            delay.delay_ms(500);
        } else {
            let off = JoystickReport {
                x: 0,
                y: 0,
                rx: 0,
                ry: 0,
                buttons: 1,
            };
            push_joystick_movement(off).ok().unwrap_or(0);
            led_pin.set_high().unwrap();
            delay.delay_ms(50);
            led_pin.set_low().unwrap();
            delay.delay_ms(50);
        }
    }
}


/// Submit a new joystick input report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_joystick_movement(report: JoystickReport) -> Result<usize, usb_device::UsbError> {
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


// End of file
