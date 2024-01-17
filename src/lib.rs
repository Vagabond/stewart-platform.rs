
use nalgebra::Vector3;
use std::f32::consts::PI;
use nalgebra::Quaternion;

struct Stewart {
    b : [Vector3<f32>; 6], // base joints in base frame
    p : [Vector3<f32>; 6], // platform joints in platform frame
    q: [Vector3<f32>; 6], // vector from base origin to Pk
    l: [Vector3<f32>; 6], // leg vector from b to p
    t0: Vector3<f32>, // Initial offset
    min_leg_length: u32,
    max_leg_length: u32,
}

impl Default for Stewart {
    fn default() -> Stewart {
        Stewart{
            min_leg_length: 0,
            max_leg_length: 0,
            b: [Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0)],
            p: [Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0)],
            q: [Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0)],
            l: [Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0)],
            t0: Vector3::new(0.0, 0.0, 0.0),
        }
    }
}

impl Stewart {
    pub fn new(base_radius: u32, platform_radius: u32, shaft_distance: f32, anchor_distance: f32, leg_length: u32, max_leg_length: u32) -> Self {
        let mut stewart = Stewart{min_leg_length: leg_length, max_leg_length: max_leg_length, ..Default::default()};
        for i in 0..6 {
            let pm = match i % 2 {
                0 => 1.0,
                1 => -1.0,
                _ => 1.0
            };
            println!("pm {}", pm);
            let phi_cut = (1 + i - i % 2) as f32 * PI / 3.0;
            let phi_b = (i + (i % 2)) as f32 * PI / 3.0 + pm * shaft_distance as f32 / 2.0;
            let phi_p = phi_cut - pm * anchor_distance as f32 / 2.0;
            stewart.b[i as usize][0] = phi_b.cos() * base_radius as f32;
            stewart.b[i as usize][1] = phi_b.sin() * base_radius as f32;
            stewart.p[i as usize][0] = phi_p.cos() * platform_radius as f32;
            stewart.p[i as usize][1] = phi_p.sin() * platform_radius as f32;
        }
        // compute the xy distance between both ends of the leg
        let leganchordistance = (((stewart.p[0][0] - stewart.b[0][0]).powf(2.0)) + (stewart.p[0][1] - stewart.b[0][1]).powf(2.0)).abs().sqrt();
        println!("leg anchor to anchor XY distance {}", leganchordistance);
        // compute the height of the right triangle 
        // a is xy distance between the anchors
        // c is the length of the leg
        // solve for b
        // b = sqrt(c^2 - a^2)
        stewart.t0[2] = ((leg_length * leg_length) as f32 - (leganchordistance * leganchordistance)).abs().sqrt();
        println!("t0 is {}", stewart.t0[2]);
        //stewart.t0[2] = 259.0;
        //println!("t0 is {}", stewart.t0[2]);
        for i in 0..6 {
            println!("b[{}][0] is {}", i, stewart.b[i][0]);
            println!("b[{}][1] is {}", i, stewart.b[i][1]);
            println!("");
            println!("p[{}][0] is {}", i, stewart.p[i][0]);
            println!("p[{}][1] is {}", i, stewart.p[i][1]);
            println!("");
        }

        stewart.update(Vector3::new(0.0, 0.0, 0.0), Quaternion::new(1.0, 0.0, 0.0, 0.0));
        stewart
    }
    pub fn update(&mut self, translation: Vector3<f32>, orientation: Quaternion<f32>) {
        // for each leg calculate q and l
        // q is the vector between the center of the base and the joint on the platform
        // l is the vector between the two leg anchors
        for i in 0..6 {
            // rotate the platform anchor point by the rotation vector
            let o = rotate_vector(orientation, self.p[i]);
            // add the translation and the rotation together
            self.q[i][0] = translation[0] + o[0];
            self.q[i][1] = translation[1] + o[1];
            self.q[i][2] = translation[2] + o[2] + self.t0[2];
            
            // subtract the base anchor from the q vector
            // l should now be between b and p points
            self.l[i][0] = self.q[i][0] - self.b[i][0];
            self.l[i][1] = self.q[i][1] - self.b[i][1];
            self.l[i][2] = self.q[i][2] - self.b[i][2];
        }
    }

    pub fn leg_length(&mut self, leg: usize) -> f32 {
        (self.l[leg][0].powf(2.0) + self.l[leg][1].powf(2.0) + self.l[leg][2].powf(2.0)).sqrt()
    }
}

// Rotates a vector according to the current quaternion, assumes |q|=1
// https://raw.org/proof/vector-rotation-using-quaternions/
// https://github.com/rawify/Quaternion.js/blob/master/quaternion.js#L1004

fn rotate_vector(o: Quaternion<f32>, v: Vector3<f32>) -> Vector3<f32> {
    let qx = o[0];
    let qy = o[1];
    let qz = o[2];
    let qw = o[3];

    let vx = v[0];
    let vy = v[1];
    let vz = v[2];

    // t = 2q x v
    let tx = 2.0 * (qy * vz - qz * vy);
    let ty = 2.0 * (qz * vx - qx * vz);
    let tz = 2.0 * (qx * vy - qy * vx);

    // v + w t + q x t
    return Vector3::new(vx + qw * tx + qy * tz - qz * ty,
                        vy + qw * ty + qz * tx - qx * tz,
                        vz + qw * tz + qx * ty - qy * tx)
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let mut s = Stewart::new(206, 206, 0.05314632, 0.05314632, 320, 520);
        println!("leg lengths {} {} {} {} {} {}", s.leg_length(0), s.leg_length(1), s.leg_length(2), s.leg_length(3), s.leg_length(4), s.leg_length(5));
        s.update(Vector3::new(0.0, 0.0, 0.0), Quaternion::new(1.0, 0.0, 0.0, 0.0).normalize());
        println!("leg lengths {} {} {} {} {} {}", s.leg_length(0), s.leg_length(1), s.leg_length(2), s.leg_length(3), s.leg_length(4), s.leg_length(5));
        s.update(Vector3::new(0.0, 0.0, 200.0), Quaternion::new(1.0, 0.0, 0.0, 0.0).normalize());
        println!("leg lengths {} {} {} {} {} {}", s.leg_length(0), s.leg_length(1), s.leg_length(2), s.leg_length(3), s.leg_length(4), s.leg_length(5));
        s.update(Vector3::new(0.0, 0.0, 0.0), Quaternion::new(1.0, 0.0, 0.0, 0.0).normalize());
        println!("leg lengths {} {} {} {} {} {}", s.leg_length(0), s.leg_length(1), s.leg_length(2), s.leg_length(3), s.leg_length(4), s.leg_length(5));
        s.update(Vector3::new(0.0, 200.0, 0.0), Quaternion::new(1.0, 10.0, 0.0, 0.0).normalize());
        println!("leg lengths {} {} {} {} {} {}", s.leg_length(0), s.leg_length(1), s.leg_length(2), s.leg_length(3), s.leg_length(4), s.leg_length(5));
        s.update(Vector3::new(0.0, 0.0, 0.0), Quaternion::new(1.0, 0.0, 0.0, 0.0).normalize());
        println!("leg lengths {} {} {} {} {} {}", s.leg_length(0), s.leg_length(1), s.leg_length(2), s.leg_length(3), s.leg_length(4), s.leg_length(5));





        assert!(false)
    }
}
