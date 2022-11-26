#![allow(dead_code)]
/// A 2d Vector of f64
#[derive(Debug, Clone)]
pub struct Vec2{
    pub x: f64,
    pub y: f64
}
impl Vec2{
    /// constructor
    pub fn new(x: f64, y: f64) -> Self{Vec2{x, y}}
    /// returns the length of the vector
    pub fn length(&self) -> f64 {(self.x*self.x + self.y*self.y).sqrt()}
    /// returns the squared length of the vector
    pub fn squared_length(&self) -> f64{self.x*self.x + self.y*self.y}
    /// returns arctan(y/x)
    pub fn atan2(&self) -> f64{self.y.atan2(self.x)}
    /// returns a new vector with same direction but length 1
    pub fn normalized(&self) -> Vec2{self.mult(1.0/self.length())}
    /// mutates the vector by multiplying it by a scalar and returns a self reference
    pub fn mut_mult(mut self, scale: f64) -> Self{ self.x *= scale; self.y *= scale; return self;}
    /// mutates the vector by dividing it by a scalar and returns a self reference
    pub fn mut_div(mut self, scale: f64) -> Self{ self.x /= scale; self.y /= scale; return self;}
    /// mutates the vector by adding another vector to it, and returns a self reference
    pub fn mut_add(mut self, other: &Vec2) -> Self{ self.x += other.x; self.y += other.y; return self;}
    /// mutates the vector by subtracting another vector to it, and returns a self reference
    pub fn mut_sub(mut self, other: &Vec2) -> Self{ self.x -= other.x; self.y -= other.y; return self;}
    /// mutates the vector by rotating it 90 degrees ccw, and returns a self reference
    pub fn mut_rotate_90_ccw(mut self) -> Self{let new_x = self.y*-1.0; self.y = self.x; self.x = new_x; return self;}
    /// multiplies self by a scalar and returns the result
    pub fn mult(&self, scale: f64) -> Vec2{ Vec2{x: self.x*scale, y: self.y*scale}}
    /// divides self by a scalar and returns the result
    pub fn div(&self, scale: f64) -> Vec2{ Vec2{x: self.x/scale, y: self.y/scale}}
    /// adds a vector to self, and returns the result
    pub fn add(&self, other: &Vec2) -> Vec2{ Vec2{x: self.x + other.x, y: self.y + other.y}}
    /// subtracts a vector from self and returns the result
    pub fn sub(&self, other: &Vec2) -> Vec2{ Vec2{x: self.x - other.x, y: self.y - other.y}}
    /// returns the vector from self to another vector
    pub fn get_step_to(&self, other: &Vec2) -> Vec2{ other.sub(self)}
    /// returns self rotated by 90 degrees ccw
    pub fn rotate_90_ccw(&self) -> Vec2{ Vec2{x: -1.0*self.y, y: self.x}}
    /// returns self rotated by 90 degrees cw
    pub fn rotate_90_cw(&self) -> Vec2{ Vec2{x: self.y, y: -1.0*self.x}}
    /// returns the distance between a vector and self
    pub fn distance_from(&self, other: &Vec2) -> f64 {self.sub(other).length()}
    /// returns the squared distance between a vector and self
    pub fn squared_distance_from(&self, other: &Vec2) -> f64 {self.sub(other).squared_length()}
    /// returns the dot product of self and another vector
    pub fn dot(&self, other: &Vec2) -> f64{self.x*other.x + self.y*other.y}
    /// returns the cross product of self and another vector
    pub fn cross(&self, other: &Vec2) -> f64{self.x*other.y - other.x*self.y}
    /// returns whether or not the vector is within the range (0, 0) -> (1, 1)
    pub fn is_in_region(&self) -> bool {self.x <= 1.0 && self.x >= 0.0 && self.y <= 1.0 && self.y >= 0.0}
    /// returns the distance between two vectors with an origin adjacent by 1 step
    /// Both vectors, self and other are assumed to be between (0, 0), and (1, 1)
    /// They both represent positions in a 1 by 1 box, and the boxes are assumed to be adjacent
    /// direction is used to represent the position of other relative to self 0:left, 1:right, 2:above, 3:below
    /// Example:
    /// Vec2{x:1, y:1}.adjacent_region_distance(Vec2{x:0, y:1}, 1) = 0.0
    /// *--------*--------*
    /// |       S|O       |
    /// |        |        |
    /// |        |        |
    /// *--------*--------*
    /// Vec2{x:1, y:1}.adjacent_region_distance(Vec2{x:0, y:1}, 0) = 2.0
    /// *--------*--------*
    /// |O       |       S|
    /// |        |        |
    /// |        |        |
    /// *--------*--------*
    pub fn adjacent_region_distance(&self, other: &Vec2, direction: usize) -> f64{
        match direction{
            0 => {return self.add(&Vec2{x: 1.0, y: 0.0}).distance_from(other);}//left
            1 => {return self.add(&Vec2{x: -1.0, y: 0.0}).distance_from(other);}//right
            2 => {return self.add(&Vec2{x: 0.0, y: -1.0}).distance_from(other);}//up
            3 => {return self.add(&Vec2{x: 0.0, y: 1.0}).distance_from(other);}//down
            _ => {return 0.0;}
        }
    }
    /// returns whether self is circumscribed by the circle represented by the three other points
    pub fn is_circumscribed(&self, a: &Vec2, b: &Vec2, c: &Vec2) -> bool{
        return determinant(
            ((a.x -self.x, a.y - self.y, (a.x*a.x - self.x*self.x) + (a.y*a.y - self.y*self.y)), 
            (b.x -self.x, b.y - self.y, (b.x*b.x - self.x*self.x) + (b.y*b.y - self.y*self.y)), 
            (c.x -self.x, c.y - self.y, (c.x*c.x - self.x*self.x) + (c.y*c.y - self.y*self.y)))
        ) > 0.0;
    }
    /// returns whether three vectors are in ccw order
    pub fn is_ccw(a: &Vec2, b: &Vec2, c: &Vec2) -> bool{
        determinant((
            (a.x, a.y, 1.0), 
            (b.x, b.y, 1.0), 
            (c.x, c.y, 1.0)
        )) > 0.0
    }
    /// returns a vector using an angle from the x axis
    pub fn from_angle(a: f64) -> Vec2 {Vec2{x: a.cos(), y: a.sin()}}
    /// returns whether or not a point is circumscribed by the circle represented by the three other points
    pub fn point_is_circumscibed(point: &Vec2, a: &Vec2, b: &Vec2, c: &Vec2) -> bool{
        return determinant(
            ((a.x -point.x, a.y - point.y, (a.x*a.x - point.x*point.x) + (a.y*a.y - point.y*point.y)), 
            (b.x -point.x, b.y - point.y, (b.x*b.x - point.x*point.x) + (b.y*b.y - point.y*point.y)), 
            (c.x -point.x, c.y - point.y, (c.x*c.x - point.x*point.x) + (c.y*c.y - point.y*point.y)))
        ) > 0.0;
    }
}


/// returns the determinant of the input matrix, in the form:
///        col1 col2 col3
/// row1 ((f64, f64, f64), 
/// row2  (f64, f64, f64), 
/// row3  (f64, f64, f64))
pub fn determinant(matrix: ((f64, f64, f64), (f64, f64, f64), (f64, f64, f64))) -> f64{
    matrix.0.0*matrix.1.1*matrix.2.2 + matrix.0.1*matrix.1.2*matrix.2.0 + matrix.0.2*matrix.1.0*matrix.2.1
    -matrix.0.0*matrix.1.2*matrix.2.1 - matrix.0.1*matrix.1.0*matrix.2.2 - matrix.0.2*matrix.1.1*matrix.2.0
}


/// A 2d coordinate struct of i32, used for the chunk hashmap
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Coordinate{
    pub x: i32,
    pub y: i32
}
impl Coordinate{
    /// returns the coordinate directly left of this one
    pub fn left     (&self) -> Coordinate {
        Coordinate { x: self.x-1, y: self.y }
    }
    /// returns the coordinate directly right of this one
    pub fn right    (&self) -> Coordinate {
        Coordinate { x: self.x+1, y: self.y }
    }
    /// returns the coordinate directly above this one
    pub fn up       (&self) -> Coordinate {
        Coordinate { x: self.x, y: self.y+1 }
    }
    /// returns the coordinate directly beneath this one
    pub fn down     (&self) -> Coordinate {
        Coordinate { x: self.x, y: self.y-1 }
    }
}