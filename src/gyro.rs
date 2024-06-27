use crate::button::{Button, Actions};
use embedded_hal::digital::InputPin;
use crate::vector::{Vector, Vector4};
use crate::imu::{imu_read_gyro, };
use crate::hid::mouse_move;
use libm::{modf, pow};

pub const CFG_GYRO_SENSITIVITY: f64 = 0.00283203125; // pow(2.0, -9.0) * 1.45;
pub const CFG_GYRO_SENSITIVITY_X: f64 = CFG_GYRO_SENSITIVITY;
pub const CFG_GYRO_SENSITIVITY_Y: f64 = CFG_GYRO_SENSITIVITY;
pub const CFG_GYRO_SENSITIVITY_Z: f64 = CFG_GYRO_SENSITIVITY;

struct Gyro {
    //mode: GyroMode,
    engage: u8,
    //engage_button: Button,
    limits: GyroLimits,
    pressed: GyroPressed,
    actions: GyroActions,
    world: World,
    sens_mult: f64,
    sub: SubPixels,
}

struct World {
    world_init: u8,
    world_top: Vector,
    world_fw: Vector,
    world_right: Vector,
    accel_smooth: Vector,
}

struct GyroLimits {
    absolute_x_min: f64,
    absolute_x_max: f64,
    absolute_y_min: f64,
    absolute_y_max: f64,
    absolute_z_min: f64,
    absolute_z_max: f64,
}

struct GyroPressed {
    x_pos: bool,
    x_neg: bool,
    y_pos: bool,
    y_neg: bool,
    z_pos: bool,
    z_neg: bool,
}

struct GyroActions {
    x_pos: Actions,
    x_neg: Actions,
    y_pos: Actions,
    y_neg: Actions,
    z_pos: Actions,
    z_neg: Actions,
}

struct SubPixels {
    x: f64,
    y: f64,
    z: f64,
}

impl Gyro {
    fn new() -> Self {
        let world = World {
            world_init: 0,
            world_top:      Vector::new(),
            world_fw:       Vector::new(),
            world_right:    Vector::new(),
            accel_smooth:   Vector::new(),
        };
        let limits = GyroLimits {
            absolute_x_min:  0.0,
            absolute_x_max:  0.0,
            absolute_y_min:  0.0,
            absolute_y_max:  0.0,
            absolute_z_min:  0.0,
            absolute_z_max:  0.0,
        };
        let pressed = GyroPressed {
            x_pos: false,
            x_neg: false,
            y_pos: false,
            y_neg: false,
            z_pos: false,
            z_neg: false,
        };
        let actions = GyroActions {
            x_pos: [0, 0, 0, 0],
            x_neg: [0, 0, 0, 0],
            y_pos: [0, 0, 0, 0],
            y_neg: [0, 0, 0, 0],
            z_pos: [0, 0, 0, 0],
            z_neg: [0, 0, 0, 0],
        };
        let sub = SubPixels {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        Gyro {
            engage: 0,
            // engage_button,
            limits,
            pressed,
            actions,
            world,
            sens_mult: 0f64,
            sub,
        }
    }
    fn report(&mut self) {
        Self::report_incremental(self)
    }
    fn report_incremental(&mut self) {
         // Read gyro values.
        let imu_gyro: Vector = imu_read_gyro();
        let mut x: f64 = imu_gyro.x * CFG_GYRO_SENSITIVITY_X * self.sens_mult;
        let mut y: f64 = imu_gyro.y * CFG_GYRO_SENSITIVITY_Y * self.sens_mult;
        let mut z: f64 = imu_gyro.z * CFG_GYRO_SENSITIVITY_Z * self.sens_mult;
        // Additional processing.
        let t: f64 = 1.0;
        let k: f64 = 0.5;
        if      x > 0.0 && x <  t { x =  hssnf(t, k,  x) }
        else if x < 0.0 && x > -t { x = -hssnf(t, k, -x) };
        if      y > 0.0 && y <  t { y =  hssnf(t, k,  y) }
        else if y < 0.0 && y > -t { y = -hssnf(t, k, -y) };
        if      z > 0.0 && z <  t { z =  hssnf(t, k,  z) }
        else if z < 0.0 && z > -t { z = -hssnf(t, k, -z) };
        // Reintroduce subpixel leftovers.
        x += self.sub.x;
        y += self.sub.y;
        z += self.sub.z;
        // Round down and save leftovers.
        (self.sub.x, x) = modf(x);
        (self.sub.y, y) = modf(y);
        (self.sub.z, z) = modf(z);
        // Report.
        if x >= 0.0 {gyro_incremental_output( x, &self.actions.x_pos)}
        else        {gyro_incremental_output(-x, &self.actions.x_neg)};
        if y >= 0.0 {gyro_incremental_output( y, &self.actions.y_pos)}
        else        {gyro_incremental_output(-y, &self.actions.y_neg)};
        if z >= 0.0 {gyro_incremental_output( z, &self.actions.z_pos)}
        else        {gyro_incremental_output(-z, &self.actions.z_neg)};
    }
    // fn report_absolute() {todo!()}
    fn reset(&mut self) {
        self.pressed.x_pos = false;
        self.pressed.x_neg = false;
        self.pressed.y_pos = false;
        self.pressed.y_neg = false;
        self.pressed.z_pos = false;
        self.pressed.z_neg = false;
        self.world.world_init = 0;

    }
}

fn hssnf(t: f64, k: f64, x: f64) -> f64 {
    let a: f64 = x - (x * k);
    let b: f64 = 1.0 - (x * k * (1.0/t));
    return a / b;
}

fn gyro_incremental_output(value: f64, actions: &Actions) {
    let MOUSE_INDEX = 162u8;
    let MOUSE_X         =  MOUSE_INDEX + 7;
    let MOUSE_Y         =  MOUSE_INDEX + 8;
    let MOUSE_X_NEG     =  MOUSE_INDEX + 9;
    let MOUSE_Y_NEG     =  MOUSE_INDEX + 10;
    
    for action in actions.iter()
    {
        if      *action == MOUSE_X     {mouse_move(value, 0.0)}
        else if *action == MOUSE_Y     {mouse_move(0.0, value)}
        else if *action == MOUSE_X_NEG {mouse_move(-value, 0.0)}
        else if *action == MOUSE_Y_NEG {mouse_move(0.0, -value)};
    }
}