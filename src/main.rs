#![allow(unused_imports)]
#![allow(non_snake_case)]
mod triangulation;
mod point;
use macroquad::{prelude::*, color};
use std::{thread, time};
use triangulation::ChunkedTriangulator;
use point::{Coordinate, Vec2};

const SCALE: f64 = 150.0;
const ORIGIN: Vec2 = Vec2{x: 150.0, y: 10.0};
const SIZE: Vec2 = Vec2{x: 1500.0, y: 1000.0};
#[macroquad::main("Triangulation")]
async fn main() {
    let mut generator = ChunkedTriangulator::new();
    let mut pointer: Coordinate = Coordinate{x: 0, y: 0};
    request_new_screen_size(SIZE.x as f32, SIZE.y as f32);
    loop{
        clear_background(BLACK);
        for (coord, chunk) in &generator.chunks{
            //triangles:
            for i in 0..chunk.triangles.len(){
                line(&chunk.points[chunk.triangles[i].points[0]], &chunk.points[chunk.triangles[i].points[1]], GREEN, coord);
                line(&chunk.points[chunk.triangles[i].points[1]], &chunk.points[chunk.triangles[i].points[2]], GREEN, coord);
                line(&chunk.points[chunk.triangles[i].points[2]], &chunk.points[chunk.triangles[i].points[0]], GREEN, coord);
            }
            //hull:
            for i in 0..chunk.hull.len(){
                line(&chunk.points[chunk.hull[i]], &chunk.points[chunk.hull[(i+1)%chunk.hull.len()]], RED, coord);
            }
            //points
            for i in 0..chunk.points.len(){
                circle(chunk.points[i].x, chunk.points[i].y, BLUE, coord);
            }
        }
        if is_key_released(KeyCode::Space) {
            let first = time::SystemTime::now();
            let num = generator.generate_chunk(pointer.clone());
            println!("Generated {} points in {} milliseconds", num, time::SystemTime::now().duration_since(first).unwrap().as_millis());
        }
        if is_key_released(KeyCode::Right) {pointer = pointer.right();}
        if is_key_released(KeyCode::Left) {pointer = pointer.left();}
        if is_key_released(KeyCode::Up) {pointer = pointer.up();}
        if is_key_released(KeyCode::Down) {pointer = pointer.down();}
        next_frame().await;
    }
}
fn circle(x: f64, y: f64, c: macroquad::color::Color, offset: &Coordinate) {
    draw_circle(((x+(offset.x as f64))*SCALE + ORIGIN.x) as f32, (SIZE.y - ((y+(offset.y as f64))*SCALE + ORIGIN.y)) as f32, 1.0, c);
} 
fn line(a: &Vec2, b: &Vec2, c: macroquad::color::Color, offset: &Coordinate){
    draw_line(
        ((a.x+(offset.x as f64))*SCALE + ORIGIN.x) as f32, 
        (SIZE.y - ((a.y+(offset.y as f64))*SCALE + ORIGIN.y)) as f32, 
        ((b.x+(offset.x as f64))*SCALE + ORIGIN.x) as f32, 
        (SIZE.y - ((b.y+(offset.y as f64))*SCALE + ORIGIN.y)) as f32, 
        1.0, 
        c
    );
}