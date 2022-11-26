#![allow(dead_code)]
use std::f64::consts::PI;
use std::ops::Rem;
use std::thread::{current, Thread};
use std::{fs::File};
use std::io::Write; 
use delaunator::triangulate;
use rand::rngs::ThreadRng;
use rand::seq::SliceRandom;
use rand::{thread_rng, Rng};
use kiddo::KdTree;
use kiddo::distance::squared_euclidean;
use anyhow::{Result, Ok};
use rand_distr::Distribution;
const PROB: usize = 10;
const RANGE: f64 = 0.25;
//assorted Functions
pub struct Graph {
    pub points: Vec<Point>,
    pub triangles: Vec<Triangle>,
    hull_points: Vec<usize>,
    pub edges: Vec<Edge>,
    pub hull_edges: Vec<usize>,
    pub shapes: Vec<Shape>
}
impl Graph{
    pub fn delaunay_from_points(points: Vec<Point>) -> Graph{
        println!("Generating stuff");
        let input: Vec<delaunator::Point> = (0..(points.len())).map(|n| delaunator::Point { x: points[n].x, y: points[n].y }).collect::<Vec<delaunator::Point>>();
        let result = triangulate(&input);
        //Get our Data Arrays
        let mut edges: Vec<Edge> = vec![];
        let mut hull_edges: Vec<usize> = vec![];
        let mut edge_test: Vec<Vec<i32>> = vec![vec![-1; points.len()]; points.len()];
        let hull_points: Vec<usize> = result.hull.clone();
        let mut index: usize = 0;
        let mut triangles: Vec<Triangle> = result.triangles.clone().chunks(3).into_iter().map(|p| {
            if edge_test[p[0]][p[1]] == -1 {
                edge_test[p[0]][p[1]] = edges.len() as i32;
                edge_test[p[1]][p[0]] = edges.len() as i32;
                edges.push(Edge{points: vec![p[0], p[1]], hull: false, enable: true, triangles: vec![]});
            }
            if edge_test[p[1]][p[2]] == -1 {
                edge_test[p[1]][p[2]] = edges.len() as i32;
                edge_test[p[2]][p[1]] = edges.len() as i32;
                edges.push(Edge{points: vec![p[1], p[2]], hull: false, enable: true, triangles: vec![]});
            }
            if edge_test[p[0]][p[2]] == -1 {
                edge_test[p[0]][p[2]] = edges.len() as i32;
                edge_test[p[2]][p[0]] = edges.len() as i32;
                edges.push(Edge{points: vec![p[0], p[2]], hull: false, enable: true, triangles: vec![]});
            }
            edges[edge_test[p[0]][p[1]] as usize].triangles.push((index, Graph::is_obtuse(&[p[0], p[2], p[1]], &points), edge_test[p[1]][p[2]] as usize, edge_test[p[2]][p[0]] as usize));
            edges[edge_test[p[1]][p[2]] as usize].triangles.push((index, Graph::is_obtuse(&[p[2], p[0], p[1]], &points), edge_test[p[2]][p[0]] as usize, edge_test[p[0]][p[1]] as usize));
            edges[edge_test[p[2]][p[0]] as usize].triangles.push((index, Graph::is_obtuse(&[p[0], p[1], p[2]], &points), edge_test[p[1]][p[2]] as usize, edge_test[p[1]][p[0]] as usize));
            index += 1;
            Triangle{
                points: vec![p[0], p[1], p[2]], 
                circumcenter: Graph::get_circumcenter(&points[p[0]], &points[p[1]], &points[p[2]]),
                enable: true,
                shape: 0
            }
        }).collect();
        for i in 0..edges.len(){
            if edges[i].triangles.len() == 1 {edges[i].hull = true; hull_edges.push(i);}
        }
        //edit our triangles
        Graph::transform_triangles(&mut triangles, &mut edges, &mut hull_edges);
        //write triangles
        Graph::write_triangles(&triangles, &points);
        //create Shapes:
        let mut shapes: Vec<Shape> = vec![];
        let mut cur_index: usize = 0;
        for i in 0..triangles.len(){
            if triangles[i].enable{
                shapes.push(Shape{
                    triangles: vec![i], 
                    hull_edges: vec![
                        edge_test[triangles[i].points[0]][triangles[i].points[1]] as usize,
                        edge_test[triangles[i].points[1]][triangles[i].points[2]] as usize,
                        edge_test[triangles[i].points[2]][triangles[i].points[0]] as usize
                    ],
                    enable: true,
                    index: cur_index
                });
                cur_index+=1;
                triangles[i].shape = cur_index-1;
                edges[shapes[shapes.len()-1].hull_edges[0]].enable = true;
                edges[shapes[shapes.len()-1].hull_edges[0]].hull = true;
                edges[shapes[shapes.len()-1].hull_edges[1]].enable = true;
                edges[shapes[shapes.len()-1].hull_edges[1]].hull = true;
                edges[shapes[shapes.len()-1].hull_edges[2]].enable = true;
                edges[shapes[shapes.len()-1].hull_edges[2]].hull = true;
            }
        }
        //write shapes
        Graph::write_shapes(&shapes, &triangles, &points);
        return Graph { points: points, triangles: triangles, hull_points: hull_points, edges: edges, hull_edges: hull_edges, shapes: shapes };
    }
    pub fn delaunay_from_random(num: usize, xmin: f64, ymin: f64, xmax: f64, ymax: f64) -> Graph{
        let mut rng = rand::thread_rng();
        let mut points: Vec<Point> = vec![];
        for _i in 0..num
        {
            points.push(Point {x: rng.gen_range(xmin..xmax), y: rng.gen_range(ymin..ymax)});
        }
        return Graph::delaunay_from_points(points);
    }
    pub fn delaunay_from_random_area(xmin: f64, ymin: f64, xmax: f64, ymax: f64, min_dist: f64, max_tries: usize) -> Graph{
        let mut rng = thread_rng();
        let mut tree = KdTree::new();
        let mut created: Vec<([f64; 2], usize)> = vec![];
        created.push(([rng.gen_range(xmin..xmax), rng.gen_range(ymin..ymax)], 0));
        tree.add(&created[0].0, created[0].1).expect("weird");
        let mut found: bool;
        let min: f64 = min_dist*min_dist;
        let mut cur_min: f64;
        loop{
            found = false;
            cur_min = min * rng.gen_range(0.8..1.5);
            for i in 0..max_tries{
                let cur: ([f64; 2], usize) = ([rng.gen_range(xmin..xmax), rng.gen_range(ymin..ymax)], i);
                let nearest = tree.nearest_one(&cur.0, &squared_euclidean).unwrap().0;
                if nearest > cur_min {
                    created.push(cur);
                    found = true;
                    break;
                }
            }
            if !found {break;}
            tree.add(&created[created.len()-1].0, created[created.len()-1].1).expect("Wu");
        }
        let chosen: Vec<Point> = created.into_iter().map(|p| Point {x: p.0[0], y: p.0[1]}).collect();
        return Graph::delaunay_from_points(chosen);
    }
    pub fn delaunay_from_random_quad(num: usize, width: f64, height: f64) -> Graph{
        let mut generator = PointGenerator::new();
        return Graph::delaunay_from_points(generator.get_points(num, width, height));
    }
    pub fn delaunay_from_preset() -> Graph{
        let points: Vec<Point> = vec![
            Point {x: 318.0, y: 219.0},
            Point {x: 751.0, y: 219.0},
            Point {x: 853.0, y: 34.0},
            Point {x: 1460.0, y: 90.0},
            Point {x: 1756.0, y: 177.0},
            Point {x: 1600.0, y: 503.0},
            Point {x: 1376.0, y: 542.0},
            Point {x: 1191.0, y: 497.0},
            Point {x: 1804.0, y: 809.0},
            Point {x: 1273.0, y: 901.0},
            Point {x: 844.0, y: 844.0},
            Point {x: 933.0, y: 471.0},
            Point {x: 523.0, y: 494.0},
            Point {x: 323.0, y: 465.0},
            Point {x: 56.0, y: 608.0},
            Point {x: 206.0, y: 803.0},
            Point {x: 405.0, y: 942.0},
            Point {x: 527.0, y: 845.0}];
        return Graph::delaunay_from_points(points);
    }
    pub fn is_obtuse(order: &[usize], points: &Vec<Point>) -> bool{
        let s1 = points[order[0]].squared_distance_from(&points[order[1]]);
        let s2 = points[order[1]].squared_distance_from(&points[order[2]]);
        let l1 = points[order[0]].squared_distance_from(&points[order[2]]);
        return (s1+s2) < (l1);
    }
    pub fn transform_triangles(triangles: &mut Vec<Triangle>, edges: &mut Vec<Edge>, hull_edges: &mut Vec<usize>){
        Graph::cull_obtuse_hull_triangles(triangles, edges, hull_edges);
        //Graph::cull_lone_hull_triangles(triangles, edges, hull_edges);
    }
    pub fn cull_obtuse_hull_triangles(triangles: &mut Vec<Triangle>, edges: &mut Vec<Edge>, hull_edges: &mut Vec<usize>){
        let mut i: usize = 0;
        while i < hull_edges.len(){
            if !edges[edges[hull_edges[i]].triangles[0].2].hull && !edges[edges[hull_edges[i]].triangles[0].3].hull && edges[hull_edges[i]].triangles[0].1{
                
                triangles[edges[hull_edges[i]].triangles[0].0].enable = false;
                edges[hull_edges[i]].enable = false;
                edges[hull_edges[i]].hull = false;
                let index: usize = hull_edges[i];
                hull_edges[i] = edges[index].triangles[0].2;
                hull_edges.push(edges[index].triangles[0].3);
                edges[hull_edges[i]].hull = true; edges[hull_edges[hull_edges.len()-1]].hull = true;
                if edges[hull_edges[i]].triangles[0].0 == edges[index].triangles[0].0 {
                    edges[hull_edges[i]].triangles.remove(0);
                }else{
                    edges[hull_edges[i]].triangles.remove(1);
                }
                if edges[hull_edges[hull_edges.len()-1]].triangles[0].0 == edges[index].triangles[0].0 {
                    edges[hull_edges[hull_edges.len()-1]].triangles.remove(0);
                }else{
                    edges[hull_edges[hull_edges.len()-1]].triangles.remove(1);
                }
            }else{i+=1;}
        }
    }
    pub fn cull_lone_hull_triangles(triangles: &mut Vec<Triangle>, edges: &mut Vec<Edge>, hull_edges: &mut Vec<usize>){
        let mut i: usize = 0;
        while i < hull_edges.len(){
            let edge_a: usize = hull_edges[i];
            if edges[edges[edge_a].triangles[0].2].hull{
                let edge_b: usize = edges[hull_edges[i]].triangles[0].2;
                triangles[edges[edge_a].triangles[0].0].enable = false;
                edges[edge_a].enable = false; edges[edge_a].hull = false;
                edges[edge_b].enable = false; edges[edge_b].hull = false;
                let edge_c: usize = edges[edge_a].triangles[0].3;
                let cur_triangle: usize = edges[edge_a].triangles[0].0;
                edges[edge_c].triangles.retain(|t| t.0 != cur_triangle);
                edges[edge_c].hull = true;
                hull_edges[i] = edge_c;
                hull_edges.retain(|e| *e != edge_b);
            }else if edges[edges[hull_edges[i]].triangles[0].3].hull {
                let edge_b: usize = edges[hull_edges[i]].triangles[0].3;
                triangles[edges[edge_a].triangles[0].0].enable = false;
                edges[edge_a].enable = false; edges[edge_a].hull = false;
                edges[edge_b].enable = false; edges[edge_b].hull = false;
                let edge_c: usize = edges[edge_a].triangles[0].2;
                let cur_triangle: usize = edges[edge_a].triangles[0].0;
                edges[edge_c].triangles.retain(|t| t.0 != cur_triangle);
                edges[edge_c].hull = true;
                hull_edges[i] = edge_c;
                hull_edges.retain(|e| *e != edge_b);
            }else{i+=1;}
        }
    }
    pub fn write_triangles(triangles: &Vec<Triangle>, nodes: &Vec<Point>){
        let mut file = File::create("D:\\Programming folders\\CurrentProjects\\Procedural Fun\\FunMap\\src\\Triangles.txt").unwrap();
        for i in 0..triangles.len(){
            if triangles[i].enable{
                writeln!(&mut file, "{}", nodes[triangles[i].points[0]].x).unwrap();
                writeln!(&mut file, "{}", nodes[triangles[i].points[0]].y).unwrap();
                writeln!(&mut file, "{}", nodes[triangles[i].points[1]].x).unwrap();
                writeln!(&mut file, "{}", nodes[triangles[i].points[1]].y).unwrap();
                writeln!(&mut file, "{}", nodes[triangles[i].points[2]].x).unwrap();
                writeln!(&mut file, "{}", nodes[triangles[i].points[2]].y).unwrap();
            }
        }
    }
    pub fn write_shapes(shapes: &Vec<Shape>, triangles: &Vec<Triangle>, nodes: &Vec<Point>){
        let mut file = File::create("D:\\Programming folders\\CurrentProjects\\Procedural Fun\\FunMap\\src\\Shapes.txt").unwrap();
        for i in 0..shapes.len(){
            writeln!(&mut file, "Shape:").unwrap();
            for j in shapes[i].triangles.as_slice(){
                writeln!(&mut file, "{}", nodes[triangles[*j].points[0]].x).unwrap();
                writeln!(&mut file, "{}", nodes[triangles[*j].points[0]].y).unwrap();
                writeln!(&mut file, "{}", nodes[triangles[*j].points[1]].x).unwrap();
                writeln!(&mut file, "{}", nodes[triangles[*j].points[1]].y).unwrap();
                writeln!(&mut file, "{}", nodes[triangles[*j].points[2]].x).unwrap();
                writeln!(&mut file, "{}", nodes[triangles[*j].points[2]].y).unwrap();
            }
        }
    }
    pub fn get_circumcenter(a: &Point, b: &Point, c: &Point) -> Point {
        let mid_ab: Point = Point{x: (a.x+b.x)/2.0, y: (a.y+b.y)/2.0};
        let mid_bc: Point = Point{x: (b.x+c.x)/2.0, y: (b.y+c.y)/2.0};
        let slope_ab = -(a.x-b.x)/(a.y-b.y);
        let slope_bc = -(b.x-c.x)/(b.y-c.y);
        let x = (mid_bc.y - mid_ab.y - slope_bc*mid_bc.x + slope_ab*mid_ab.x)/(slope_ab - slope_bc);
        let y = slope_ab*(x-mid_ab.x) + mid_ab.y;
        return Point{x, y};
    }
    pub fn get_min_angle(points: &Vec<usize>, nodes: &Vec<Point>) -> (usize, f64){
        let ab: Point = Point{x: nodes[points[1]].x - nodes[points[0]].x, y: nodes[points[1]].y - nodes[points[0]].y};
        let ac: Point = Point{x: nodes[points[2]].x - nodes[points[0]].x, y: nodes[points[2]].y - nodes[points[0]].y};
        let a: f64 = ((ab.x*ac.x + ab.y*ac.y) / (ab.length() * ac.length())).acos();
        let ba: Point = Point{x: nodes[points[0]].x - nodes[points[1]].x, y: nodes[points[0]].y - nodes[points[1]].y};
        let bc: Point = Point{x: nodes[points[2]].x - nodes[points[1]].x, y: nodes[points[2]].y - nodes[points[1]].y};
        let b: f64 = ((ba.x*bc.x + ba.y*bc.y) / (ba.length() * bc.length())).acos();
        let ca: Point = Point{x: nodes[points[0]].x - nodes[points[2]].x, y: nodes[points[0]].y - nodes[points[2]].y};
        let cb: Point = Point{x: nodes[points[1]].x - nodes[points[2]].x, y: nodes[points[1]].y - nodes[points[2]].y};
        let c: f64 = ((ca.x*cb.x + ca.y*cb.y) / (ca.length() * cb.length())).acos();
        if a < b && a < c {return (0, a);}
        else if b < c {return (1, b);}
        return (2, c);
    }
    pub fn transform_shapes(&mut self){
        let mut merged: Vec<usize> = (0..self.shapes.len()).into_iter().collect();
        while merged.len() > 0 {
            let m = self.merge_shape();
            merged.retain(|p| *p!=m.0 && *p!=m.1);
        }
    }
    pub fn transform_single_triangle(&mut self) -> (usize, usize, usize){
        
        for i in 0..self.hull_edges.len(){
            if !self.edges[self.edges[self.hull_edges[i]].triangles[0].2].hull && !self.edges[self.edges[self.hull_edges[i]].triangles[0].3].hull && self.edges[self.hull_edges[i]].triangles[0].1{
                self.triangles[self.edges[self.hull_edges[i]].triangles[0].0].enable = false;
                self.edges[self.hull_edges[i]].enable = false;
                self.edges[self.hull_edges[i]].hull = false;
                let index: usize = self.hull_edges[i];
                self.hull_edges.push(self.edges[index].triangles[0].2);
                self.hull_edges.push(self.edges[index].triangles[0].3);
                self.hull_edges.remove(i);
                println!("{} {}", self.edges[self.hull_edges[self.hull_edges.len()-1]].triangles.len(), self.edges[self.hull_edges[self.hull_edges.len()-2]].triangles.len());
                println!("{} {} {} {}", self.hull_edges[self.hull_edges.len()-1], self.hull_edges[self.hull_edges.len()-2], self.edges[index].triangles[0].2, self.edges[index].triangles[0].3);
                self.edges[self.hull_edges[self.hull_edges.len()-2]].hull = true; self.edges[self.hull_edges[self.hull_edges.len()-1]].hull = true;
                println!("{:?}", self.edges[self.hull_edges[self.hull_edges.len()-2]].triangles);
                if self.edges[self.hull_edges[self.hull_edges.len()-2]].triangles[0].0 == self.edges[index].triangles[0].0 {
                    self.edges[self.hull_edges[self.hull_edges.len()-2]].triangles.remove(0);
                }else{
                    self.edges[self.hull_edges[self.hull_edges.len()-2]].triangles.remove(1);
                }
                println!("{:?}", self.edges[self.hull_edges[self.hull_edges.len()-2]].triangles);
                println!("{:?}", self.edges[self.hull_edges[self.hull_edges.len()-1]].triangles);
                if self.edges[self.hull_edges[self.hull_edges.len()-1]].triangles[0].0 == self.edges[index].triangles[0].0 {
                    self.edges[self.hull_edges[self.hull_edges.len()-1]].triangles.remove(0);
                }else{
                    self.edges[self.hull_edges[self.hull_edges.len()-1]].triangles.remove(1);
                }
                println!("{:?}", self.edges[self.hull_edges[self.hull_edges.len()-1]].triangles);
                return (index, self.edges[index].triangles[0].2, self.edges[index].triangles[0].3);
            }
        }
        return (0, 0, 0);
    }
    pub fn merge_shape(&mut self) -> (usize, usize){
        let mut min: (f64, usize, usize, Option<usize>, Option<usize>) = (100000., 0, 0, Some(0), Some(0));
        let mut mindex: usize = 0;
        for i in 0..self.shapes.len(){
            if self.shapes[i].enable && self.shapes[i].triangles.len() == 1{
                let x = self.shapes[i].get_min_angle(&self.edges, &self.points);
                if x.0 < min.0 {
                    min = x; 
                    mindex = i;
                }
            }
        }
        let other: usize;
        if (min.3).is_some() && (min.4).is_some() {
            let triangle_a = if self.edges[min.1].triangles[0].0 == (min.3).unwrap() {self.edges[min.1].triangles[1].0} else {self.edges[min.1].triangles[0].0};
            let triangle_b = if self.edges[min.2].triangles[0].0 == (min.4).unwrap() {self.edges[min.2].triangles[1].0} else {self.edges[min.2].triangles[0].0};
            let dist_a: f64 = self.triangles[triangle_a].circumcenter.squared_distance_from(&self.triangles[(min.3).unwrap()].circumcenter);
            let dist_b: f64 = self.triangles[triangle_b].circumcenter.squared_distance_from(&self.triangles[(min.4).unwrap()].circumcenter);
            if dist_a < dist_b {
                let index: usize = self.triangles[(min.3).unwrap()].shape;
                let merged: Shape = self.shapes[index].clone();
                self.shapes[mindex].merge_shape(merged, &self.edges, &mut self.triangles).expect("Failed to Merge");
                self.shapes[index].enable = false;
                other = index;
            }else{
                let index: usize = self.triangles[(min.4).unwrap()].shape;
                let merged: Shape = self.shapes[index].clone();
                self.shapes[mindex].merge_shape(merged, &self.edges, &mut self.triangles).expect("Failed to Merge");
                self.shapes[index].enable = false;
                other = index;
            }
        }else if (min.3).is_some() {
            let index: usize = self.triangles[(min.3).unwrap()].shape;
            let merged: Shape = self.shapes[index].clone();
            self.shapes[mindex].merge_shape(merged, &self.edges, &mut self.triangles).expect("Failed to Merge");
            self.shapes[index].enable = false;
            other = index;
        }else {
            let index: usize = self.triangles[(min.4).unwrap()].shape;
            let merged: Shape = self.shapes[index].clone();
            self.shapes[mindex].merge_shape(merged, &self.edges, &mut self.triangles).expect("Failed to Merge");
            self.shapes[index].enable = false;
            other = index;
        }
        return (mindex, other);
    }
}
/*
    Structs:
*/

//Point Struct
#[derive(Debug, Clone)]
pub struct Point{
    pub x: f64,
    pub y: f64
}
impl Point{
    pub fn squared_distance_from(&self, other: &Self) -> f64 { 
        (self.x - other.x).powi(2) + (self.y - other.y).powi(2) 
    }
    pub fn length(&self) -> f64 {
        return (self.x*self.x + self.y*self.y).sqrt();
    }
    pub fn normalized(&self) -> Self{
        let len: f64 = self.length();
        return Point{x: self.x/len, y: self.y/len};
    }
    pub fn dot(&self, other: &Self) -> f64{
        return self.x*other.x + self.y*other.y;
    }
    pub fn atan2(&self) -> f64{
        return self.y.atan2(self.x);
    }
    pub fn from_to(a: &Self, b: &Self) -> Self{
        return Point{x: b.x-a.x, y: b.y-a.y};
    }
    pub fn rotated_90_ccw(&self) ->Self{
        return Point{x: -1.0*self.y, y: self.x};
    }
    pub fn multiply(&self, f: f64) ->Self{
        return Point{x: self.x*f, y: self.y*f};
    }
    pub fn add(&self, other: &Self) ->Self{
        return Point{x: self.x+other.x, y: self.y+other.y};
    }
    pub fn distance_from(&self, other: &Self) ->f64{
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
    pub fn det(&self, other: &Self) -> f64{
        return self.x*other.y - self.y*other.x;
    }
    pub fn from_angle(angle: f64) -> Point{
        return Point{x: angle.cos(), y: angle.sin()};
    }
}

//Triangle Struct 
#[derive(Debug, Clone)]
pub struct Triangle{
    pub points: Vec<usize>,
    circumcenter: Point,
    pub enable: bool,
    shape: usize
}

//Edge Struct
#[derive(Debug, Clone)]
pub struct Edge{
    pub points: Vec<usize>,
    hull: bool,
    pub triangles: Vec<(usize, bool, usize, usize)>, // triangle index, opp point obtuseness, triangles other edges index
    enable: bool
}

//Shape struct
#[derive(Debug, Clone)]
pub struct Shape{
    pub triangles: Vec<usize>,
    pub hull_edges: Vec<usize>,
    index: usize,
    pub enable: bool
}
impl Shape{ 
    //return the minimum angle, 
    //first edge of angle, 
    //second edge of angle, 
    //an option of the triangle connected to teh first edge not in the shape, 
    //an option of the triangle connected to the second edge not in the shape.
     pub fn get_min_angle(&self, edges: &Vec<Edge>, points: &Vec<Point>) ->(f64, usize, usize, Option<usize>, Option<usize>){
        let mut min: f64 = 10000.;
        let mut index_a: usize = 0;
        let mut index_b: usize = 0;
        for i in 0..self.hull_edges.len(){
            let a: usize = self.hull_edges[i];
            let b: usize = self.hull_edges[(i+1).rem_euclid(self.hull_edges.len()) as usize];
            if edges[a].triangles.len() > 1 || edges[b].triangles.len() > 1 { // we only want to consider angles that can be fixed with  merge
                let x1: f64; let y1: f64; let x2: f64; let y2: f64;
                if edges[a].points[0] == edges[b].points[0]{
                    x1 = points[edges[a].points[1]].x - points[edges[a].points[0]].x;
                    y1 = points[edges[a].points[1]].y - points[edges[a].points[0]].y;
                    x2 = points[edges[b].points[1]].x - points[edges[b].points[0]].x;
                    y2 = points[edges[b].points[1]].y - points[edges[b].points[0]].y;
                }else if edges[a].points[1] == edges[b].points[1]{
                    x1 = points[edges[a].points[0]].x - points[edges[a].points[1]].x;
                    y1 = points[edges[a].points[0]].y - points[edges[a].points[1]].y;
                    x2 = points[edges[b].points[0]].x - points[edges[b].points[1]].x;
                    y2 = points[edges[b].points[0]].y - points[edges[b].points[1]].y;
                }else if edges[a].points[0] == edges[b].points[1]{
                    x1 = points[edges[a].points[1]].x - points[edges[a].points[0]].x;
                    y1 = points[edges[a].points[1]].y - points[edges[a].points[0]].y;
                    x2 = points[edges[b].points[0]].x - points[edges[b].points[1]].x;
                    y2 = points[edges[b].points[0]].y - points[edges[b].points[1]].y;
                }else {
                    x1 = points[edges[a].points[0]].x - points[edges[a].points[1]].x;
                    y1 = points[edges[a].points[0]].y - points[edges[a].points[1]].y;
                    x2 = points[edges[b].points[1]].x - points[edges[b].points[0]].x;
                    y2 = points[edges[b].points[1]].y - points[edges[b].points[0]].y;
                }
                let angle: f64 = ((x1*x2+y1*y2) / ((x1*x1+y1*y1).sqrt()*(x2*x2+y2*y2).sqrt())).acos();
                if angle < min { 
                    min = angle;
                    index_a = a;
                    index_b = b;
                };
            }
        }
        let triangle_a: Option<usize> = if edges[index_a].triangles.len() > 1 {
            if self.triangles.contains(&edges[index_a].triangles[0].0) {Some(edges[index_a].triangles[1].0)}
            else {Some(edges[index_a].triangles[0].0)}
        }else {None};
        let triangle_b: Option<usize> = if edges[index_b].triangles.len() > 1 {
            if self.triangles.contains(&edges[index_b].triangles[0].0) {Some(edges[index_b].triangles[1].0)}
            else {Some(edges[index_b].triangles[0].0)}
        }else {None};
        return (min, index_a, index_b, triangle_a, triangle_b);
     }
     pub fn merge_shape(&mut self, other: Self, edges: &Vec<Edge>, triangles: &mut Vec<Triangle>) -> Result<usize>{
        let mut hull_segments: Vec<Vec<usize>> = vec![vec![]];
        let mut cur_segment: usize = 0;
        for i in 0..self.hull_edges.len(){
            if !other.hull_edges.contains(&self.hull_edges[i]) {hull_segments[cur_segment].push(self.hull_edges[i]);}
            else if hull_segments[cur_segment].len() > 0 {hull_segments.push(vec![]); cur_segment += 1;}
        }
        if hull_segments[hull_segments.len()-1].len() == 0 {hull_segments.pop(); cur_segment -= 1;}
        hull_segments.push(vec![]);
        cur_segment += 1;
        for i in 0..other.hull_edges.len(){
            if !self.hull_edges.contains(&other.hull_edges[i]) {hull_segments[cur_segment].push(other.hull_edges[i]);}
            else if hull_segments[cur_segment].len() > 0 {hull_segments.push(vec![]); cur_segment += 1;}
        }
        if hull_segments[hull_segments.len()-1].len() == 0 {hull_segments.pop();}
        //hullsegments now contains a bunch of lined up edges
        let mut merged: bool = true;
        while merged{
            merged = false;
            'outer: for i in 0..hull_segments.len() {
                for j in 0..hull_segments.len(){
                    if i!=j{
                        if edges[hull_segments[j][0]].points[0] == edges[*hull_segments[i].last().ok_or(std::fmt::Error)?].points[0] ||
                        edges[hull_segments[j][0]].points[0] == edges[*hull_segments[i].last().ok_or(std::fmt::Error)?].points[1] || 
                        edges[hull_segments[j][0]].points[1] == edges[*hull_segments[i].last().ok_or(std::fmt::Error)?].points[0] || 
                        edges[hull_segments[j][0]].points[1] == edges[*hull_segments[i].last().ok_or(std::fmt::Error)?].points[1] {
                            //attach j to the end of i
                            let segment = hull_segments[j].clone();
                            hull_segments[i].extend_from_slice(segment.into_iter().as_slice());
                            hull_segments.remove(j);
                            merged = true;
                            break 'outer;
                        }
                    }
                }
            }
        }
        //all tail to head connections were made if we have 2 vectors, reverse one and add it to the end of tyhe other
        if hull_segments.len() > 1{
            while hull_segments[1].len() > 0 {
                let temp = hull_segments[1].pop().ok_or(std::fmt::Error)?;
                hull_segments[0].push(temp);
            }
            hull_segments.remove(1);
        }
        self.hull_edges = hull_segments[0].clone();
        //now merge triangles, 
        for i in other.triangles.into_iter(){
            self.triangles.push(i);
            triangles[i].shape = self.index;
        }
        return Ok(self.hull_edges.len());
     }
}


//Point Generator Struct
pub struct PointGenerator{
    pub trees: Vec<QuadTree>,
    pub rng: ThreadRng
}
impl PointGenerator{
    pub fn new() -> Self{
        PointGenerator { trees: vec![QuadTree::new(true, (0, 0))], rng: rand::thread_rng()}
    }
    pub fn generate_point(&mut self) {
        let mut current_tree: usize = 0;
        let mut generating: bool = true;
        while generating {
            //println!("Beggining of Loop: {}", current_tree);
            let num: (usize, i32) = self.trees[current_tree].sample(&mut self.rng);
            //println!("number: {:?}", num);
            assert_ne!(num.1, -3, "Couldnt Sample");
            if num.1 == -1 {
                self.trees[current_tree].branches[num.0] = -2;
                self.trees[current_tree].total_prob -= PROB-1;
                generating = false;
            }else if num.1 == -2 {
                self.trees.push(QuadTree::new(false, (current_tree, num.0)));
                self.trees[current_tree].branches[num.0] = (self.trees.len() - 1) as i32;
                generating = false;
                current_tree = self.trees.len()-1;
                self.trees[current_tree].total_prob -= (PROB-1)*2;
                let rand_a = self.rng.gen_range(0..4);
                let rand_b = self.rng.gen_range(1..4);
                self.trees[current_tree].branches[rand_a] = -2;
                self.trees[current_tree].branches[(rand_a+rand_b).rem(4)] = -2;
                //println!("{} {} {} {} {} {} {}", (rand_a+rand_b).rem(4), rand_a, rand_b, self.trees[current_tree].branches[0], self.trees[current_tree].branches[1], self.trees[current_tree].branches[2], self.trees[current_tree].branches[3]);
            }else {
                current_tree = num.1 as usize;
            }
        }
    }
    pub fn generate_points(&mut self, num: usize){
        for _ in 0..num{
            self.generate_point();
        }
    }
    pub fn get_points(&mut self, num: usize, width: f64, height: f64) -> Vec<Point> {
        self.generate_points(num);
        let mut points: Vec<Point> = vec![];
        for i in 0..self.trees.len(){
            self.trees[i].shuffle_origins(&mut self.rng);
        }
        for i in 0..self.trees.len(){
            for j in 0..self.trees[i].branches.len(){
                if self.trees[i].branches[j] == -2 {
                    let mut point: Point = Point{x: self.rng.gen::<f64>()*RANGE, y: self.rng.gen::<f64>()*RANGE};
                    point.x += (1.0-RANGE)*0.5; point.y += (1.0-RANGE)*0.5;
                    let mut current_tree: (usize, usize) = (i, j);
                    loop{
                        point.x *= 0.5; point.y *= 0.5;
                        let origin = self.trees[current_tree.0].origins[current_tree.1].clone();
                        point.x += origin.x; point.y += origin.y;
                        if self.trees[current_tree.0].root {break;}
                        current_tree = self.trees[current_tree.0].parent;
                    }
                    point.x*=width; point.y*= height;
                    points.push(point);
                }
            }
        }
        return points;
    }
}
//Quad Tree Struct
pub struct QuadTree{
    pub branches: Vec<i32>, // -1: unchosen  -2: chosen but not subdivided   anything else: chosen and value is index to child tree
    pub total_prob: usize,
    pub origins: Vec<Point>,
    pub root: bool,
    pub parent: (usize, usize)
}
impl QuadTree{
    pub fn new(root: bool, parent: (usize, usize)) -> Self{
        QuadTree{branches: vec![-1, -1, -1, -1], total_prob: 4*PROB, origins: vec![Point{x: 0., y: 0.}, Point{x: 0.5, y: 0.}, Point{x: 0., y: 0.5}, Point{x: 0.5, y: 0.5}], root: root, parent: parent}
    }
    pub fn sample(&self, rng: &mut ThreadRng) -> (usize, i32) {
        let mut x: usize = rng.gen_range(0..self.total_prob);
        //println!("Num and Range: {} {}", x, self.total_prob);
        for i in 0..self.branches.len(){
            if self.branches[i] == -1 {
                if x < PROB {return (i, self.branches[i]);}
                x -= PROB;
            }else{
                if x < 1 {return (i, self.branches[i]);}
                x -= 1;
            }
        }
        return (0, -3);
    }
    pub fn shuffle_origins(&mut self, rng: &mut ThreadRng){
        self.origins.as_mut_slice().shuffle(rng);
    }
}




//Curve and Angle Struct
pub struct Hull{
    pub a: Point,
    pub b: Point,
    c: Point,
    const_len: f64,
    var_len: f64,
    pivot: Point,
    step: Point,
    step_angle: f64,
    theta: f64
}
impl Hull{
    pub fn new(a: Point, b:Point, c:Point) -> Self{
        let const_len: f64 = a.distance_from(&b);
        let b_to_c: Point = Point::from_to(&b, &c);
        let b_to_a: Point = Point::from_to(&b, &a);
        let theta: f64 = (b_to_a.atan2() - b_to_c.atan2()).rem_euclid(2.0*PI) - PI;
        let var_len: f64 = theta;
        let pivot: Point = Point::from_to(&a, &b).normalized();
        let step: Point = pivot.rotated_90_ccw();
        let step_angle: f64 = step.atan2();
        return Hull{a, b, c, const_len, var_len, pivot ,step ,step_angle, theta};
    }
    pub fn get_length(&self, r: f64) -> f64{
        return self.const_len + self.var_len*r;
    }
    pub fn get_point(&self, r: f64, p: f64) -> (Point, usize) {
        let total_length: f64 = self.const_len + self.var_len*r;
        let pos: f64 = p*total_length;
        let excess: f64 = pos - self.const_len;
        if excess > 0.0{
            let final_angle: f64 = self.step_angle - (excess / r);
            //println!("P: {}", excess/(r*self.theta));
            if excess/(r*self.theta) <= 0.5 {
                return (Point::from_angle(final_angle).multiply(r).add(&self.b), 1);
            }else{
                return (Point::from_angle(final_angle).multiply(r).add(&self.b), 2);
            }
        }else{
            //println!("A: {}", pos/self.a.distance_from(&self.b));
            return (self.pivot.multiply(pos).add(&(self.step.multiply(r))).add(&self.a), 1);
        }
    }
    pub fn recompute(&mut self){
        self.const_len= self.a.distance_from(&self.b);
        let b_to_c: Point = Point::from_to(&self.b, &self.c);
        let b_to_a: Point = Point::from_to(&self.b, &self.a);
        self.theta = (b_to_a.atan2() - b_to_c.atan2()).rem_euclid(2.0*PI) - PI;
        self.var_len = self.theta;
        self.pivot = Point::from_to(&self.a, &self.b).normalized();
        self.step = self.pivot.rotated_90_ccw();
        self.step_angle= self.step.atan2();
    }
    pub fn set_c(&mut self, point: Point){
        self.c = point;
        let b_to_c: Point = Point::from_to(&self.b, &self.c);
        let b_to_a: Point = Point::from_to(&self.b, &self.a);
        self.theta = (b_to_a.atan2() - b_to_c.atan2()).rem_euclid(2.0*PI) - PI;
        self.var_len = self.theta;
    }
    pub fn set_b_c(&mut self, point_b: Point, point_c: Point){
        self.b = point_b;
        self.c = point_c;
        self.recompute();
    }
}

pub struct ProceduralGraphBuilder{
    pub points: Vec<Point>,
    pub hull: Vec<Hull>, //clockwise array of segments on the hull
    rng: ThreadRng,
    minimum_distance: f64,
    distance_range: f64,
    const_len: f64,
    var_len: f64
}
impl ProceduralGraphBuilder{
    /* creates a new generator with the initial points of a triangle at (0,0)
     */
    pub fn new(min: f64, range: f64) -> Self{
        let x = ProceduralGraphBuilder { 
            points: vec![], 
            rng: rand::thread_rng(), 
            minimum_distance: min, 
            distance_range: range,
            hull: vec![
                Hull::new(Point{x: -1.0*min, y: 0.0*min}, Point{x: 0.0*min, y: 2.0*min}, Point{x: 1.0*min, y: 0.0*min}), 
                Hull::new(Point{x: 0.0*min, y: 2.0*min}, Point{x: 1.0*min, y: 0.0*min}, Point{x: -1.0*min, y: 0.0*min}), 
                Hull::new(Point{x: 1.0*min, y: 0.0*min}, Point{x: -1.0*min, y: 0.0*min}, Point{x: 0.0*min, y: 2.0*min})
                ],
            const_len: 0.0,
            var_len: 0.0
        };
        return x;
    }
    pub fn calculate_lengths(&mut self) {
        self.const_len = 0.0; self.var_len = 0.0;
        for i in 0..self.hull.len(){
            self.const_len += self.hull[i].const_len;
            self.var_len += self.hull[i].var_len;
        }
    }
    pub fn generate(&mut self){
        let points: Vec<(Point, usize)> = self.generate_points();
        //self.add_point(temp);
        //self.fix_hull();
    }
    pub fn generate_points(&mut self) -> Vec<(Point, usize)>{
        self.calculate_lengths();
        let mut ranges: Vec<(f64, f64)> = vec![(0.0, 1.0)];
        let mut total_range: f64 = 1.0;
        let min: f64 = self.minimum_distance / (self.const_len + self.var_len*self.minimum_distance);
        let range: f64 = self.distance_range / (self.const_len + self.var_len*self.minimum_distance);
        let mut positions: Vec<f64> = vec![];
        while ranges.len() > 0 {
            let new_position = self.rng.gen_range(0.0..total_range);
            let new_range = range + min;
            let mut cur_position = new_position;
            for i in 0..ranges.len(){
                if cur_position <= ranges[i].1 - ranges[i].0 {
                    cur_position += ranges[i].0;
                    positions.push(cur_position);
                    let cur_range = ranges.remove(i);
                    total_range -= cur_range.1 - cur_range.0;
                    if cur_position + new_range < cur_range.1 {
                        ranges.insert(i, (cur_position + new_range, cur_range.1));
                        total_range += ranges[i].1 - ranges[i].0;
                    }
                    if cur_position - new_range > cur_range.0 {
                        ranges.insert(i, (cur_range.0, cur_position - new_range));
                        total_range += ranges[i].1 - ranges[i].0;
                    }
                    break;
                }else{
                    cur_position -= ranges[i].1 - ranges[i].0;
                }
            }
        }
        println!("{:?}", positions);
        let random_rotation = self.rng.gen_range(0.0..1.0);
        let mut points: Vec<(Point, usize)> = vec![];
        for p in 0..positions.len(){
            let rand_dist = self.rng.gen_range(0.0..1.0)*self.distance_range + self.minimum_distance;
            let mut current_pos: f64 = ((positions[p] + random_rotation) % 1.0) * (self.const_len + self.var_len*rand_dist);
            for i in 0..self.hull.len(){
                current_pos -= self.hull[i].get_length(rand_dist);
                if current_pos <= 0.0 {
                    let p: f64 = 1.0 - current_pos.abs() / self.hull[i].get_length(rand_dist);
                    let point = self.hull[i].get_point(rand_dist, p);
                    points.push((point.0, (point.1 + i)%self.hull.len()));
                    break;
                }
            }
        }
        return points;
    }
    pub fn generate_point(&mut self) -> (Point, usize){
        //Generate 2 random numbers, one for the distance, and the other for the place on the hull
        self.calculate_lengths();
        let rand_dist: f64 = self.rng.gen_range(0.0..1.0) * self.distance_range + self.minimum_distance;
        let total_length: f64 = self.const_len + self.var_len*rand_dist;
        let rand_pos: f64 = self.rng.gen_range(0.0..1.0)*total_length;
        let mut current_pos: f64 = rand_pos;
        for i in 0..self.hull.len(){
            current_pos -= self.hull[i].get_length(rand_dist);
            if current_pos <= 0.0 {
                let p: f64 = 1.0 - current_pos.abs() / self.hull[i].get_length(rand_dist);
                let point = self.hull[i].get_point(rand_dist, p);
                return (point.0, (point.1 + i)%self.hull.len());
            }
        }
        println!("ERROR");
        return (Point{x: -1.0, y: -1.0}, 0);
    }
    pub fn add_point(&mut self, new: (Point, usize)){
        let back_2: usize = ((new.1 as i32) - 2).rem_euclid(self.hull.len() as i32) as usize;
        let back_1: usize = ((new.1 as i32) - 1).rem_euclid(self.hull.len() as i32) as usize;
        self.hull[back_2].set_c(new.0.clone());
        let temp = self.hull[new.1].a.clone();
        self.hull[back_1].set_b_c(new.0.clone(), temp);
        let new_hull: Hull = Hull::new(new.0.clone(), self.hull[new.1].a.clone(), self.hull[new.1].b.clone());
        self.hull.insert(new.1, new_hull);
    }
    pub fn fix_hull(&mut self) {
        let mut edited: bool = true;
        while edited{
            edited = false;
            for i in 0..self.hull.len(){
                if self.hull[i].theta < 0.0 || self.hull[i].theta > PI{
                    //that means its b is concave!!!
                    let back_1: usize = (i as i32-1).rem_euclid(self.hull.len() as i32) as usize;
                    let bad: usize = (i+1)%self.hull.len();
                    let new_forward: usize = (i+2)%self.hull.len();
                    self.points.push(self.hull[bad].a.clone());
                    let temp = self.hull[new_forward].a.clone();
                    self.hull[back_1].set_c(temp);
                    let temp_1 = self.hull[new_forward].a.clone();
                    let temp_2 = self.hull[new_forward].b.clone();
                    self.hull[i].set_b_c(temp_1, temp_2);
                    self.hull.remove(bad);
                    edited = true;
                    break;
                }
            }
        }
    }
}

//Chunked Point Generator
#[derive(Debug, Clone)]
pub struct Coordinate{
    x: i32,
    y: i32
}
pub struct ChunkedPointGenerator{
    chunks: Vec<PointChunk>,
    pointer: Coordinate
}
impl ChunkedPointGenerator{
    pub fn new() -> Self {ChunkedPointGenerator { chunks: vec![] , pointer: Coordinate { x: 0, y: 0 }}}
    pub fn generate_chunk(&mut self) { self.generate_chunk_at(self.pointer.clone());}
    pub fn generate_chunk_at(&mut self, coord: Coordinate){
        let new = PointChunk::new(coord);
        //get rid of edge cases
        self.chunks.push(new);
    }
    pub fn travel_north(&mut self){self.pointer.y += 1;}
    pub fn travel_south(&mut self){self.pointer.y -= 1;}
    pub fn travel_east(&mut self){self.pointer.x += 1;}
    pub fn travel_west(&mut self){self.pointer.x -= 1;}
}
pub struct PointChunk{
    index: Coordinate,
    points: Vec<Point>
}
impl PointChunk{
    pub fn new(coord: Coordinate) -> Self{
        let mut new = PointChunk{index: coord, points: vec![]};
        new.generate_points();
        return new;
    }
    pub fn generate_points(&mut self){
        //generate chunk points
    }
}