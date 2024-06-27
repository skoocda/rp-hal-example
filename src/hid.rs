
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
/// 
/// 
// XInput report structure
// #[gen_hid_descriptor(
//     (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = JOYSTICK) = {
//         (usage = X, logical_minimum = 0, logical_maximum = 255, report_count = 1, report_size = 8, input = Data) = x,
//         (usage = Y, logical_minimum = 0, logical_maximum = 255, report_count = 1, report_size = 8, input = Data) = y,
//         (usage_page = BUTTON, usage_minimum = 1, usage_maximum = 14, logical_minimum = 0, logical_maximum = 1, report_count = 14, report_size = 1, input = Data) = buttons,
//         (usage_page = GENERIC_DESKTOP, usage = Z, logical_minimum = 0, logical_maximum = 255, report_count = 1, report_size = 8, input = Data) = left_trigger,
//         (usage = RZ, logical_minimum = 0, logical_maximum = 255, report_count = 1, report_size = 8, input = Data) = right_trigger,
//     }
// )]
// struct GamepadReport {
//     x: u8,
//     y: u8,
//     buttons: u16,
//     left_trigger: u8,
//     right_trigger: u8,
// }

pub fn mouse_move(x: f64, y: f64) {
    todo!()
}