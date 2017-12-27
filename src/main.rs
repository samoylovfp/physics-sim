extern crate find_folder;
extern crate piston_window;

use std::collections::HashMap;
use std::ops::Add;

use piston_window::*;

const GRAVITY: f64 = 6.67428e-11;
const DAY: f64 = 24.0*3600.0;
const AU: f64 = 149.6e9; // Astronomical Unit in meters, roughly distance earth -> sun
const SCALE: f64 = 100.0 / AU;
const DIMENSION: u32 = 500;
const HALF: f64 = 149.6e9 * 2.0 + 149.6e9 / 2.0;


type Vel = (f64, f64);
type Pos = (f64, f64);
type Mass = f64;
type Accel = (f64, f64);
type Color = [f32; 4];
type Distance = (f64, f64);

#[derive(Debug, Clone, PartialEq)]
struct Force {x: f64, y: f64}
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
    pos: Pos,
    vel: Vel,
    mass: Mass,
    color: Color, // TODO: color
    name: String,
    scale_pos: Pos,
}

impl Body {
    fn new(p: Pos, v: Vel, m: Mass, c: Color, n: String) -> Body {
        Body {
            pos: p,
            vel: v,
            mass: m,
            color: c,
            name: n,
            scale_pos: (p.0 * SCALE, p.1 * SCALE),
        }
    }

    fn draw_planet(&self,c: Context, g: &mut G2d)  {
        let radius: f64 = 7.0;
        Rectangle::new(self.color) // red color
            .draw(
                [self.scale_pos.0, self.scale_pos.1,
                 radius, radius],
                &c.draw_state, c.transform, g
            );
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
            return Force{x: 0.0, y: 0.0};
        }
        let force = GRAVITY * self.mass * other / distance.powf(2.0);
        // atan2 is the tangent of angle theta
        let theta = dist.1.atan2(dist.0);
        // calculate the direction of force by multipling the
        // cos and sin with the total force.
        return Force{x: theta.cos() * force, y: theta.sin() * force};
    }

    // apply force to velocity, and velocity to position
    // acceleration = force / mass
    fn accelerate(&self, force: Force) -> Accel {
        return (force.x / self.mass * DAY, force.y / self.mass * DAY);
    }

    fn move_body(&mut self, acc: Accel) {
        self.vel.0 += acc.0;
        self.vel.1 += acc.1;
        self.pos.0 += self.vel.0 * DAY; // velocity is km/s
        self.pos.1 += self.vel.1 * DAY;
        self.scale_pos.0 = self.pos.0 * SCALE;
        self.scale_pos.1 = self.pos.1 * SCALE;
    }
}



fn main() {
    let mut window: PistonWindow = WindowSettings::new(
        "piston: draw_state", Size{width: DIMENSION, height: DIMENSION},
        ).exit_on_esc(true).samples(4).build().unwrap();
    window.set_lazy(true);

    let mut solar_system = big_bang();

    while let Some(e) = window.next() {
        window.draw_2d(&e, |c, g| {
            clear([0.8, 0.8, 0.8, 1.0], g);
            g.clear_stencil(0);

            let mut sums = Vec::new();
            for body in solar_system.iter() {
                let mut forces = Vec::new();
                for i in 0..solar_system.len() {
                    let distance = body.get_distance(solar_system[i].pos);
                    let force = body.get_force(solar_system[i].mass, distance);
                    forces.push(force);
                }
                let sum = forces.iter().fold(
                    Force{x: 0.0, y: 0.0},
                    (|sum,value| sum + value.clone())
                );
                sums.insert(0, sum);
            }
            for body in solar_system.iter_mut() {
                let force = sums.pop();
                let accel = body.accelerate(force.unwrap());
                body.move_body(accel);
                body.draw_planet(c, g);
                println!(
                    "{:?}: \n  Pos: {:?}\n  Vel: {:?}",
                    body.name, body.pos, body.vel
                );
            }
        });
    }
}


fn big_bang() -> Vec<Body> {
    let mut solar_system = Vec::new();
    let mut sun = Body::new(
        (HALF, HALF),
        (0.0, 0.0),
        1.98892 * 10.0_f64.powf(30.0),
        [255.0, 255.0, 0.0, 1.0],
        "Sun".to_string(),
    );
    let mut earth = Body::new(
        (HALF-AU, HALF),
        (0.0, 35.02 * 1000.0),
        5.972 * 10.0_f64.powf(24.0),
        [0.0, 0.0, 225.0, 1.0],
        "Earth".to_string(),
    );
    let mut venus = Body::new(
        (HALF+149.6e9, HALF),
        (0.0, 29.783 * 1000.0),
        4.8685 * 10.0_f64.powf(24.0),
        [0.1, 0.0, 0.0, 1.0],
        "Venus".to_string(),
    );
    solar_system.extend([sun, earth, venus].iter().cloned());
    return solar_system;
}


fn local_bodies() -> Vec<Body> {
    let mut solar_system = Vec::new();
    let mut moon = Body::new(
        (AU-384000000.0, AU),
        (0.0, 3683.0),
        7.34767309 * 10.0_f64.powf(22.0),
        [255.0, 255.0, 0.0, 1.0],
        "Moon".to_string(),
    );
    let mut earth = Body::new(
        (AU, AU),
        (0.0, 0.0),
        5.972 * 10.0_f64.powf(24.0),
        [0.0, 0.0, 225.0, 1.0],
        "Earth".to_string(),
    );
    solar_system.extend([earth, moon].iter().cloned());
    return solar_system;
}

#[test]
fn test_new_body() {
    let mut earth = Body::new(
        (HALF-AU, HALF),
        (0.0, 35.02 * 1000.0),
        5.972 * 10.0_f64.powf(24.0),
        [0.0; 4],
        "Earth".to_string(),
    );
    assert_eq!(earth.name, "Earth".to_string());
}



#[cfg(test)]
mod tests {
    use super::*;

    fn new_planet() -> Body {
        return Body::new(
            (5.0, 5.0),
            (0.0, 20.0),
            500.0,
            [0.0; 4],
            "Simple Planet".to_string(),
        );
    }

    fn new_earth() -> Body {
        return Body::new(
            (HALF-AU, HALF),
            (0.0, 35.02 * 1000.0),
            5.972 * 10.0_f64.powf(24.0),
            [0.0; 4],
            "Earth".to_string(),
        );
    }

    #[test]
    fn test_new_body() {
        let earth = new_earth();
        assert_eq!(earth.name, "Earth".to_string());
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
        assert_eq!(force, Force{x: 35422429872810204000000.0, y: 0.0});
    }

    #[test]
    fn test_accelerate() {
        let planet = new_planet();
        let force = Force{x: 5.0, y: 3.0};
        assert_eq!((0.01, 0.006), planet.accelerate(force));
    }
}

#[test]
fn test_add_force() {
    assert_eq!(Force { x: 1.0, y: 0.0 } + Force { x: 2.0, y: 3.0 },
               Force { x: 3.0, y: 3.0 });

}
