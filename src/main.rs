// Physics sim implemented in Rust
// Each `step` is a DAY (3600 seconds * 24 hours)

use std::ops::Add;

const GRAVITY: f64 = 6.67428e-11;
const DAY: f64 = 24.0 * 3600.0;
const AU: f64 = 149.6e9; // Astronomical Unit in meters, roughly distance earth -> sun
const HALF: f64 = AU * 10.0;

type Vel = (f64, f64);
type Pos = (f64, f64);
type Mass = f64;
type Accel = (f64, f64);
type Distance = (f64, f64);

#[derive(Debug, Clone, PartialEq)]
struct Force {
    x: f64,
    y: f64,
}

impl Add for Force {
    type Output = Force;
    fn add(self, other: Force) -> Force {
        Force {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
struct Body {
    pos: Pos,     // x y non negative coordinates
    vel: Vel,     // velocity in km/s
    mass: Mass,   // mass in kg
    name: String
}

impl Body {
    fn new(p: Pos, v: Vel, m: Mass, n: String, r: f64) -> Body {
        Body {
            pos: p,
            vel: v,
            mass: m,
            name: n
        }
    }

    // Get distance returns a vector of the Distance
    // between two points in meters.
    fn get_distance(&self, other: Pos) -> Distance {
        let dx = other.0 - self.pos.0;
        let dy = other.1 - self.pos.1;
        return (dx, dy);
    }

    // Get the force of gravity between two objects.
    //     F = mass_a * mass_b  / distance^2
    // Distance is calculated as
    //     sqrt((x_a - x_b)^2(y_a - y_b)^2)
    //
    // The Force returned is measured in newtons
    fn get_force(&self, other: Mass, dist: Distance) -> Force {
        let distance = (dist.0.powf(2.0) + dist.1.powf(2.0)).sqrt();
        if distance == 0.0 {
            return Force { x: 0.0, y: 0.0 };
        }
        let force = GRAVITY * self.mass * other / distance.powf(2.0);
        // atan2 is the tangent of angle theta
        let theta = dist.1.atan2(dist.0);
        // calculate the direction of force by multipling the
        // cos and sin with the total force.
        return Force {
            x: theta.cos() * force,
            y: theta.sin() * force,
        };
    }

    // apply force to velocity, and velocity to position
    // acceleration = force / mass
    fn accelerate(&self, force: Force) -> Accel {
        return (force.x / self.mass * DAY, force.y / self.mass * DAY);
    }

    fn move_body(&mut self, acc: Accel) {
        self.vel.0 += acc.0;
        self.vel.1 += acc.1;
        self.pos.0 += self.vel.0 * DAY;
        self.pos.1 += self.vel.1 * DAY;
    }
}

trait ToVec: Iterator {
    fn to_vec(self) -> Vec<Self::Item> where Self: Sized {
        self.collect::<Vec<_>>()
    }
}

impl<T> ToVec for T where T: Iterator {}

fn main() {
    let mut solar_system = big_bang();
    let mut args_iter = std::env::args();
    let _name = args_iter.next();
    let iters = args_iter.next().and_then(|n|n.parse().ok()).expect("Provide a number of iterations");

    for _ in 0..iters {
        // Calculate
        let sums = solar_system.iter().map(|body| {
            let forces_for_body = solar_system.iter().map(|body2| {
                let distance = body.get_distance(body2.pos);
                body.get_force(body2.mass, distance)
            }).to_vec();

            forces_for_body.iter()
                .fold(Force { x: 0.0, y: 0.0 }, |sum, value| sum + value.clone())
        }).to_vec();

        // Apply
        for (body, force) in solar_system.iter_mut().zip(sums) {
            let accel = body.accelerate(force);
            body.move_body(accel);
            // println!(
            //     "{:?}: \n  Pos: {:?}\n  Vel: {:?}",
            //     body.name, body.pos, body.vel
            // );
        }
    }
    println!("Result is {:#?}", solar_system);
}

fn big_bang() -> Vec<Body> {
    let mut solar_system = Vec::new();
    let sun = Body::new(
        (HALF, HALF),
        (0.0, 0.0),
        1.98892 * 10.0_f64.powf(30.0),
        "Sun".to_string(),
        15.0,
    );
    let mars = Body::new(
        (HALF + AU * 1.524, HALF),
        (0.0, -24.077 * 1000.0),
        6.38 * 10.0_f64.powf(23.0),
        "Mars".to_string(),
        10.0,
    );
    let earth = Body::new(
        (HALF - AU, HALF),
        (0.0, 29.78 * 1000.0),
        5.972 * 10.0_f64.powf(24.0),
        "Earth".to_string(),
        9.0,
    );
    let venus = Body::new(
        (HALF + 108.2e9, HALF),
        (0.0, -35.02 * 1000.0),
        4.8685 * 10.0_f64.powf(24.0),
        "Venus".to_string(),
        8.0,
    );
    let mercury = Body::new(
        (HALF + AU * 0.39, HALF),
        (0.0, -48.0 * 1000.0),
        3.3010 * 10.0_f64.powf(23.0),
        "Venus".to_string(),
        6.0,
    );
    let jupiter = Body::new(
        (HALF, HALF + 5.2 * AU),
        (13.1 * 1000.0, 0.0),
        1898.0 * 10.0_f64.powf(24.0),
        "Jupiter".to_string(),
        14.0,
    );
    let saturn = Body::new(
        (HALF, HALF + 9.58 * AU),
        (9.7 * 1000.0, 0.0),
        568.0 * 10.0_f64.powf(24.0),
        "Saturn".to_string(),
        14.0,
    );
    let uranus = Body::new(
        (HALF, HALF + 19.22 * AU),
        (6.8 * 1000.0, 0.0),
        86.8 * 10.0_f64.powf(24.0),
        "Uranus".to_string(),
        14.0,
    );
    let neptune = Body::new(
        (HALF, HALF + 30.1 * AU),
        (5.4 * 1000.0, 0.0),
        102.0 * 10.0_f64.powf(24.0),
        "Neptune".to_string(),
        14.0,
    );
    solar_system.extend(
        [
            sun, mercury, venus, earth, mars, jupiter, saturn, uranus, neptune
        ].iter()
            .cloned(),
    );
    return solar_system;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn new_planet() -> Body {
        return Body::new(
            (5.0, 5.0),
            (0.0, 20.0),
            500.0,
            "Simple Planet".to_string(),
            1.0,
        );
    }

    fn new_earth() -> Body {
        return Body::new(
            (HALF - AU, HALF),
            (0.0, 35.02 * 1000.0),
            5.972 * 10.0_f64.powf(24.0),
            "Earth".to_string(),
            0.9,
        );
    }

    #[test]
    fn test_get_distance() {
        let other_point: Pos = (10.0, 10.0);
        let planet = new_planet();
        let distance = planet.get_distance(other_point);
        assert_eq!(distance, (5.0, 5.0));
    }

    #[test]
    fn test_get_force() {
        let planet = new_earth();
        let distance: Distance = planet.get_distance((HALF, HALF));
        let force: Force = planet.get_force(
            1.98892 * 10.0_f64.powf(30.0), // the sun
            distance,
        );
        assert_eq!(
            force,
            Force {
                x: 35422429872810204000000.0,
                y: 0.0,
            }
        );
    }

    #[test]
    fn test_accelerate() {
        let planet = new_earth();
        let force = Force {
            x: 35422429872810204000000.0,
            y: 0.0,
        };
        assert_eq!((512.4745380125254, 0.0), planet.accelerate(force));
    }
}

#[test]
fn test_add_force() {
    assert_eq!(
        Force { x: 1.0, y: 0.0 } + Force { x: 2.0, y: 3.0 },
        Force { x: 3.0, y: 3.0 }
    );
}
