#![allow(warnings)]
struct Thumbstick {
    mode:               ThumbstickMode,
    distance_mode:      ThumbstickDistance,
    deadzone_override:  bool,
    deadzone:           f32,
    antideadzone:       f32,
    overlap:            f32
}

struct ThumbstickPosition {
    x:      f32,
    y:      f32,
    angle:  f32,
    radius: f32,
}

enum ThumbstickMode {
    Off,
    Dir4,
    Alpha,
}
enum ThumbstickDistance {
    Axial,
    Radial,
}

enum Dir4 {
    None,
    Left,
    Right,
    Up,
    Down,
}

enum Dir8 {
    Center,
    Left,
    Right,
    Up,
    Down,
    UpLeft,
    UpRight,
    DownLeft,
    DownRight
}

impl Thumbstick {
    fn new() -> Self {
        Thumbstick {
            mode: ThumbstickMode::Dir4,
            distance_mode: ThumbstickDistance::Axial,
            deadzone_override: false,
            deadzone: 0f32,
            antideadzone: 0f32,
            overlap: 0f32,
        }
    }
    fn report(&self) {
        let x: f32 = todo!();
        let y: f32 = todo!();
    }
    fn report_axial(&self, pos: ThumbstickPosition) {todo!()}
    //fn report_radial(&self, pos: ThumbstickPosition) {todo!()}
    //fn report_alpha(&self, pos: ThumbstickPosition) {todo!()}
    // fn report_glyph(&self, input: Glyph) {todo!()}
    //fn report_wheel(&self, dir: Dir8) {todo!()}
}