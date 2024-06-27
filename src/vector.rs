use libm::{fabs, sqrt, pow, sin, cos};


#[derive(Debug, Clone)]
pub struct Vector {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Clone)]
pub struct Vector4 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub r: f64,
}

impl Vector {
    pub fn new() -> Self {
        Vector {
            x: 0f64,
            y: 0f64,
            z: 0f64,
        }
    }

    pub fn normalize(v: Vector) -> Vector {
        let mut mag = (v.x*v.x) + (v.y*v.y) + (v.z*v.z);
        if fabs(mag - 1.0f64) > 0.0001f64 {
            mag = sqrt(mag);
            return Vector {
                x: v.x/mag,
                y: v.y/mag,
                z: v.z/mag,
            }
        }
        return v
    }

    pub fn add(a: Vector, b: Vector) -> Vector {
        Vector {
            x: a.x + b.x,
            y: a.y + b.y,
            z: a.z + b.z,
        }
    }

    pub fn sub(a: Vector, b: Vector) -> Vector {
        Vector {
            x: a.x - b.x,
            y: a.y - b.y,
            z: a.z - b.z,
        }
    }

    pub fn invert(v: Vector) -> Vector {
        Vector {
            x: -v.x,
            y: -v.y,
            z: -v.z,
        }
    }

    pub fn cross_product(a: Vector, b: Vector) -> Vector {
        Vector {
            x: (a.y * b.z) - (a.z * b.y),
            y: (a.z * b.x) - (a.x * b.z),
            z: (a.x * b.y) - (a.y * b.x),
        }
    }

    pub fn smooth(a: Vector, b: Vector, weight: f64) -> Vector {
        Vector {
            x: (a.x*weight + b.x) / (weight + 1f64),
            y: (a.y*weight + b.y) / (weight + 1f64),
            z: (a.z*weight + b.z) / (weight + 1f64),
        }
    }

    pub fn length(v: Vector) -> f64 {
        sqrt(
            pow(fabs(v.x), 2f64) +
            pow(fabs(v.y), 2f64) +
            pow(fabs(v.z), 2f64)
        )
    }
}

impl Vector4 {

    pub fn quaternion(v: Vector, r: f64) -> Vector4 {
        let v_norm = Vector::normalize(v);
        let theta: f64 = r / 2f64;
        Vector4 {
            x: v_norm.x * sin(theta),
            y: v_norm.y * sin(theta),
            z: v_norm.z * sin(theta),
            r: cos(theta),
        }
    }

    pub fn qmultiply(q1: Vector4, q2: Vector4) -> Vector4 {
        Vector4 {         
            x: q1.r * q2.x + q1.x * q2.r + q1.y * q2.z - q1.z * q2.y,
            y: q1.r * q2.y + q1.y * q2.r + q1.z * q2.x - q1.x * q2.z,
            z: q1.r * q2.z + q1.z * q2.r + q1.x * q2.y - q1.y * q2.x,
            r: q1.r * q2.r - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z, 
        }
    }

    pub fn qconjugate(q: Vector4) -> Vector4 {
        Vector4 {
            x: -q.x,
            y: -q.y,
            z: -q.z,
            r: q.r
        }
    }

    pub fn qvector(q: Vector4) -> Vector {
        Vector::normalize(Vector{
            x: q.x,
            y: q.y,
            z: q.z
        })
    }

    pub fn qrotate(q: Vector4, v: Vector) -> Vector {
        let u = Vector4{
            x: v.x,
            y: v.y,
            z: v.z,
            r: 0f64
        };
        Self::qvector(Self::qmultiply(Self::qmultiply(q.clone(),u), Self::qconjugate(q)))
    }
}