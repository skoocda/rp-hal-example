/// Submit a new mouse movement report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_mouse_movement(report: MouseReport) -> Result<usize, usb_device::UsbError> {
    critical_section::with(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        USB_HID.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}

fn mouse_loop () {
    

    // Blink the LED at 1 Hz
    // If button is pressed, blink at 10 Hz 
    loop {
        if button_pin.is_high().unwrap() {
            let rep_up = MouseReport {
                x: 0,
                y: 4,
                buttons: 0,
                wheel: 0,
                pan: 0,
            };
            push_mouse_movement(rep_up).ok().unwrap_or(0);
            led_pin.set_high().unwrap();
            delay.delay_ms(500);

            let rep_down = MouseReport {
                x: 0,
                y: -4,
                buttons: 0,
                wheel: 0,
                pan: 0,
            };
            push_mouse_movement(rep_down).ok().unwrap_or(0);
            led_pin.set_low().unwrap();
            delay.delay_ms(500);
        } else {
            let click = MouseReport {
                x: 0,
                y: 0,
                buttons: 0x1,
                wheel: 0,
                pan: 0,
            };
            push_mouse_movement(click).ok().unwrap_or(0);
            led_pin.set_high().unwrap();
            delay.delay_ms(50);

            led_pin.set_low().unwrap();
            delay.delay_ms(50);
        }
    }
}