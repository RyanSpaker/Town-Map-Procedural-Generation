#![allow(dead_code)]
use crate::point::{Vec2, Coordinate};
use std::{collections::{HashMap, HashSet}, cmp::Ordering, f64::consts::PI};
use rand::{thread_rng, rngs::ThreadRng, Rng};


/// Global RNG variable, accessed using the global rng() function
static mut RNG: Option<ThreadRng> = None;
/// Accesses the global RNG variable of type 'ThreadRng'
/// If the varaible is defined it will unwrap and return it
/// Otherwise it will create it and then return it
unsafe fn rng() -> &'static mut ThreadRng {
    RNG.as_mut().unwrap_or_else(|| {
        RNG = Some(thread_rng());
        return RNG.as_mut().unwrap();
    })
}


/// A struct that handles generating and storing chunks of triangulated points
pub struct ChunkedTriangulator{
    /// a hashmap of chunks represented by their coordinate in the grid of chunks
    pub chunks: HashMap<Coordinate, TriangleChunk>, 
    /// a float representing a chunk's worldspace width
    pub chunk_width: f64, 
    /// a float representing the range parameter for point generation (in world scale)
    pub point_range: f64, 
    /// a float representing the width parameter for point generation (in world scale)
    pub point_width: f64 
}
impl ChunkedTriangulator{
    /// a constructor that sets chunk width to 1, and the point generation parameters to 0.005
    pub fn new() -> ChunkedTriangulator {
        ChunkedTriangulator { chunks: HashMap::new(), chunk_width: 1.0, point_range: 0.075, point_width: 0.05}
    }
    /// generates a chunk at a x and y position
    pub fn generate_chunk_at(&mut self, x: i32, y: i32){
        self.generate_chunk(Coordinate{x, y});
    }
    /// generates a chunk given a coordinate position. doesn't generate a chunk if one already exists at the coordinate
    pub fn generate_chunk(&mut self, coord: Coordinate) -> usize{
        if !self.chunks.contains_key(&coord) {
            let mut new_chunk = TriangleChunk::new();
            unsafe{new_chunk.generate(self.point_width/self.chunk_width, self.point_range/self.chunk_width, false);}
            self.chunks.insert(coord.clone(), new_chunk);
            return self.chunks[&coord].points.len();
        }
        return 0;
    }
    /// returns a reference to the chunk at an x and y position. errors if the chunk hasn't been generated yet
    pub fn get_chunk_at(&self, x: i32, y: i32) -> &TriangleChunk{
        self.get_chunk(&Coordinate{x, y})
    }
    /// returns a reference to the chunk at a coordinate. errors if the chunk hasn't been generated yet
    pub fn get_chunk(&self, coord: &Coordinate) -> &TriangleChunk{
        self.chunks.get(coord).unwrap()
    }
    /// returns a mut reference to the chunk at an x and y position. errors if the chunk hasn't been generated yet
    pub fn get_mut_chunk_at(&mut self, x: i32, y: i32) -> &mut TriangleChunk{
        self.get_mut_chunk(&Coordinate{x, y})
    }
    /// returns a mut reference to the chunk at a coordinate. errors if the chunk hasn't been generated yet
    pub fn get_mut_chunk(&mut self, coord: &Coordinate) -> &mut TriangleChunk{
        self.chunks.get_mut(coord).unwrap()
    }

}


/// a struct storing a single chunk, including its points and triangulation data
#[derive(Debug)]
pub struct TriangleChunk{
    /// a vector of vec2's representing each point in the chunk
    pub points: Vec<Vec2>, 
    /// a vector of triangle struct's representing the triangulation data
    pub triangles: Vec<Triangle>, 
    /// a vec containing a list of points connected in a line as th hull of the chunk
    pub hull: Vec<usize> 
}
impl TriangleChunk{
    /// a constructor that makes an empty chunk
    pub fn new() -> TriangleChunk{ TriangleChunk {points: vec![], triangles: vec![], hull: vec![]}}
    /// this function will generate all of the points and traingles for an empty chunk
    /// generates 3 starter points, and then repeatedly generates waves of points, until the chunk is filled
    /// triangulates the chunks at the same time
    pub unsafe fn generate(&mut self, min: f64, range: f64, quick: bool){
        if quick{
            let (p, t, h) = generate(min, range);
            self.points = p;
            self.triangles = t.into_iter().map(|v| {Triangle{points: v.0.to_vec(), adj:v.1.to_vec()}}).collect();
            self.hull = h;
        }
        else{
            let mut hull = Hull::new();
            self.generate_initial_values(min, range, &mut hull);
            while self.generate_single_round(min, range, &mut hull) {}
            self.hull = hull.generate_hull();
        }
    }
    /// this function will generate a single wave of points, and add it to the struct
    /// returns whether or not the function could generate any points within the chunk
    pub unsafe fn generate_single_round(&mut self, min: f64, range: f64, hull: &mut Hull) -> bool{
        // get a list of points that are projected away from the hull by a distance between min and min+range, at random but spaced points along the hull
        // doesn't include any points that were projected beyond the bounds of the chunk: (0, 0) -> (1, 1)
        let mut new = hull.get_points(min, range, &self.points);
        if new.len() == 0 {return false;}
        // expands the hull structs variables to be able to hold our new data
        hull.extend(new.len());
        let center: Vec2 = Vec2{x: 0.5, y: 0.5};
        // sorts the points based on their distance from the center 
        // ensures that when we add the points to our triangulation, a point's triangles cant encapsulate a point that hasn't been added yet
        new.sort_by(|a, b| {
            if a.squared_distance_from(&center) < b.squared_distance_from(&center){Ordering::Less}else{Ordering::Greater}
        });
        //adds each new point to the struct consecutively
        for point in new.into_iter(){
            self.add_point(point, hull);
        }
        return true;
    }
    /// adds a single point to the struct, and updates the triangulation to include the new point
    pub fn add_point(&mut self, point: Vec2, hull: &mut Hull){
        // places the point in our structs points vector
        self.points.push(point);
        let new_index: usize = self.points.len()-1;
        //find the start and end indices of hull edges that are visible to the point
        let (start, end) = hull.find_visible_hull_points(&self.points[new_index], &self.points);
        let first_tri: usize;
        let mut last_tri: usize;
        let mut altered_triangles: HashSet<usize> = HashSet::new();
        //add the first triangle our new point will create
        let new = Triangle{
            points: vec![start, new_index, hull.next[start]],
            adj: vec![None, None, Some((hull.tri[start].0, (hull.tri[start].1 + 2) % 3))]
        };
        self.triangles.push(new);
        altered_triangles.insert(self.triangles.len()-1);
        first_tri = self.triangles.len()-1;
        last_tri = self.triangles.len()-1;
        //altre triangle adjacencies
        self.triangles[hull.tri[start].0].adj[hull.tri[start].1] = Some((self.triangles.len()-1, 1));
        let mut current: usize = hull.next[start];
        //go through and add the rest of the triangles
        while hull.prev[current] != end {
            let new = Triangle{
                points: vec![current, new_index, hull.next[current]],
                adj: vec![Some((self.triangles.len()-1, 0)), None, Some((hull.tri[current].0, (hull.tri[current].1 + 2)%3))]
            };
            self.triangles.push(new);
            altered_triangles.insert(self.triangles.len()-1);
            last_tri = self.triangles.len()-1;
            self.triangles[hull.tri[current].0].adj[hull.tri[current].1] = Some((self.triangles.len()-1, 1));
            let index = self.triangles.len()-2;
            self.triangles[index].adj[1] = Some((self.triangles.len()-1, 2));
            current = hull.next[current];
        }
        // fix the hull
        let new_end = hull.next[end];

        hull.next[start] = new_index; hull.next[new_index] = new_end;
        hull.prev[new_end] = new_index; hull.prev[new_index] = start;

        hull.tri[start] = (first_tri, 0);
        hull.tri[new_index] = (last_tri, 1);

        hull.start = new_index;
        //fix the triangulation
        self.fix_triangulation(altered_triangles, hull);
        //given the new point, add it, update triangulation and hull data
    }
    /// updates the structs triangulation data
    /// altered is a list of triangles that need to be checked, and possibly altered
    pub fn fix_triangulation(&mut self, mut altered: HashSet<usize>, hull: &mut Hull){
        while !altered.is_empty(){
            let items = altered.clone();
            for triangle in items{
                if self.triangles[triangle].fix_adj(0, &self.points, &self.triangles) {
                    let adj = self.triangles[triangle].adj[0].unwrap();
                    //swap the two interchanged points
                    self.triangles[triangle].points[1] = self.triangles[adj.0].points[adj.1];
                    self.triangles[adj.0].points[(adj.1+2)%3] = self.triangles[triangle].points[2];
                    //swap the two interchanged adjacencies
                    self.triangles[triangle].adj[0] = self.triangles[adj.0].adj[(adj.1+2)%3];
                    self.triangles[adj.0].adj[(adj.1+1)%3] = self.triangles[triangle].adj[1];
                    //create the new interreferencial adjacencies
                    self.triangles[triangle].adj[1] = Some((adj.0, (adj.1+1)%3));
                    self.triangles[adj.0].adj[(adj.1+2)%3] = Some((triangle, 0));
                    //if necessary, alter other adjacent triangles adjacency to the new triangles index
                    if self.triangles[triangle].adj[0].is_some() {
                        let adj_index = (self.triangles[triangle].adj[0].unwrap().1+1)%3;
                        let adj_triangle = self.triangles[triangle].adj[0].unwrap().0;
                        self.triangles[adj_triangle].adj[adj_index] = Some((triangle, 2));
                    }
                    if self.triangles[adj.0].adj[(adj.1+1)%3].is_some() {
                        let adj_index = (self.triangles[adj.0].adj[(adj.1+1)%3].unwrap().1+1)%3;
                        let adj_triangle = self.triangles[adj.0].adj[(adj.1+1)%3].unwrap().0;
                        self.triangles[adj_triangle].adj[adj_index] = Some((adj.0, adj.1));
                    }
                    //check every possible hull segment, and alter their triangle pointer if necessary
                    if hull.next[self.triangles[triangle].points[2]] == self.triangles[triangle].points[0] {
                        hull.tri[self.triangles[triangle].points[2]] = (triangle, 2);
                    }
                    if hull.next[self.triangles[triangle].points[0]] == self.triangles[triangle].points[1] {
                        hull.tri[self.triangles[triangle].points[0]] = (triangle, 0);
                    }
                    if hull.next[self.triangles[adj.0].points[adj.1]] == self.triangles[adj.0].points[(adj.1+1)%3] {
                        hull.tri[self.triangles[adj.0].points[adj.1]] = (adj.0, adj.1);
                    }
                    if hull.next[self.triangles[adj.0].points[(adj.1+1)%3]] == self.triangles[adj.0].points[(adj.1+2)%3] {
                        hull.tri[self.triangles[adj.0].points[(adj.1+1)%3]] = (adj.0, (adj.1+1)%3);
                    }
                    altered.insert(adj.0);
                }
                else if self.triangles[triangle].fix_adj(1, &self.points, &self.triangles) {
                    let adj = self.triangles[triangle].adj[1].unwrap();
                    //swap the two interchanged points
                    self.triangles[triangle].points[2] = self.triangles[adj.0].points[adj.1];
                    self.triangles[adj.0].points[(adj.1+2)%3] = self.triangles[triangle].points[0];
                    //swap the two interchanged adjacencies
                    self.triangles[triangle].adj[1] = self.triangles[adj.0].adj[(adj.1+2)%3];
                    self.triangles[adj.0].adj[(adj.1+1)%3] = self.triangles[triangle].adj[2];
                    //create the new interreferencial adjacencies
                    self.triangles[triangle].adj[2] = Some((adj.0, (adj.1+1)%3));
                    self.triangles[adj.0].adj[(adj.1+2)%3] = Some((triangle, 1));
                    //if necessary, alter other adjacent triangles adjacency to the new triangles index
                    if self.triangles[triangle].adj[1].is_some() {
                        let adj_index = (self.triangles[triangle].adj[1].unwrap().1+1)%3;
                        let adj_triangle = self.triangles[triangle].adj[1].unwrap().0;
                        self.triangles[adj_triangle].adj[adj_index] = Some((triangle, 0));
                    }
                    if self.triangles[adj.0].adj[(adj.1+1)%3].is_some() {
                        let adj_index = (self.triangles[adj.0].adj[(adj.1+1)%3].unwrap().1+1)%3;
                        let adj_triangle = self.triangles[adj.0].adj[(adj.1+1)%3].unwrap().0;
                        self.triangles[adj_triangle].adj[adj_index] = Some((adj.0, adj.1));
                    }
                    //check every possible hull segment, and alter their triangle pointer if necessary
                    if hull.next[self.triangles[triangle].points[0]] == self.triangles[triangle].points[1] {
                        hull.tri[self.triangles[triangle].points[0]] = (triangle, 0);
                    }
                    if hull.next[self.triangles[triangle].points[1]] == self.triangles[triangle].points[2] {
                        hull.tri[self.triangles[triangle].points[1]] = (triangle, 1);
                    }
                    if hull.next[self.triangles[adj.0].points[adj.1]] == self.triangles[adj.0].points[(adj.1+1)%3] {
                        hull.tri[self.triangles[adj.0].points[adj.1]] = (adj.0, adj.1);
                    }
                    if hull.next[self.triangles[adj.0].points[(adj.1+1)%3]] == self.triangles[adj.0].points[(adj.1+2)%3] {
                        hull.tri[self.triangles[adj.0].points[(adj.1+1)%3]] = (adj.0, (adj.1+1)%3);
                    }
                    altered.insert(adj.0);
                }
                else if self.triangles[triangle].fix_adj(2, &self.points, &self.triangles) {
                    let adj = self.triangles[triangle].adj[2].unwrap();
                    //swap the two interchanged points
                    self.triangles[triangle].points[0] = self.triangles[adj.0].points[adj.1];
                    self.triangles[adj.0].points[(adj.1+2)%3] = self.triangles[triangle].points[1];
                    //swap the two interchanged adjacencies
                    self.triangles[triangle].adj[2] = self.triangles[adj.0].adj[(adj.1+2)%3];
                    self.triangles[adj.0].adj[(adj.1+1)%3] = self.triangles[triangle].adj[0];
                    //create the new interreferencial adjacencies
                    self.triangles[triangle].adj[0] = Some((adj.0, (adj.1+1)%3));
                    self.triangles[adj.0].adj[(adj.1+2)%3] = Some((triangle, 2));
                    //if necessary, alter other adjacent triangles adjacency to the new triangles index
                    if self.triangles[triangle].adj[2].is_some() {
                        let adj_index = (self.triangles[triangle].adj[2].unwrap().1+1)%3;
                        let adj_triangle = self.triangles[triangle].adj[2].unwrap().0;
                        self.triangles[adj_triangle].adj[adj_index] = Some((triangle, 1));
                    }
                    if self.triangles[adj.0].adj[(adj.1+1)%3].is_some() {
                        let adj_index = (self.triangles[adj.0].adj[(adj.1+1)%3].unwrap().1+1)%3;
                        let adj_triangle = self.triangles[adj.0].adj[(adj.1+1)%3].unwrap().0;
                        self.triangles[adj_triangle].adj[adj_index] = Some((adj.0, adj.1));
                    }
                    //check every possible hull segment, and alter their triangle pointer if necessary
                    if hull.next[self.triangles[triangle].points[1]] == self.triangles[triangle].points[2] {
                        hull.tri[self.triangles[triangle].points[1]] = (triangle, 1);
                    }
                    if hull.next[self.triangles[triangle].points[2]] == self.triangles[triangle].points[0] {
                        hull.tri[self.triangles[triangle].points[2]] = (triangle, 2);
                    }
                    if hull.next[self.triangles[adj.0].points[adj.1]] == self.triangles[adj.0].points[(adj.1+1)%3] {
                        hull.tri[self.triangles[adj.0].points[adj.1]] = (adj.0, adj.1);
                    }
                    if hull.next[self.triangles[adj.0].points[(adj.1+1)%3]] == self.triangles[adj.0].points[(adj.1+2)%3] {
                        hull.tri[self.triangles[adj.0].points[(adj.1+1)%3]] = (adj.0, (adj.1+1)%3);
                    }
                    altered.insert(adj.0);
                }
                else {
                    altered.remove(&triangle);
                }
            }
        }
    
    }
    /// finds 3 starting points, and initializes the struct variables
    /// the 3 starting points are the vertices of an equilateral triangle of side length min+range centered at (0.5, 0.50)
    pub fn generate_initial_values(&mut self, min: f64, range: f64, hull: &mut Hull) {
        let starter_distance = (min+range) / 3_f64.sqrt();
        let center: Vec2 = Vec2{x: 0.5, y: 0.5};
        unsafe{ 
            let rand_start_angle = rng().gen::<f64>() * PI * 2.0;         
            self.points.push(Vec2::from_angle(rand_start_angle).mut_mult(starter_distance).mut_add(&center));
            self.points.push(Vec2::from_angle(rand_start_angle + 2.0*PI/3.0).mut_mult(starter_distance).mut_add(&center));
            self.points.push(Vec2::from_angle(rand_start_angle + 4.0*PI/3.0).mut_mult(starter_distance).mut_add(&center));
        }
        self.triangles.push(Triangle { points: vec![0, 1, 2], adj: vec![None, None, None] });
        hull.initialize();
    }
}


///The entire generation alg in one function with no structs
pub unsafe fn generate(min: f64, range: f64) -> (Vec<Vec2>, Vec<([usize; 3], [Option<(usize, usize)>;3])>, Vec<usize>){
    /*Define initial values*/
    let rand_start_angle = rng().gen::<f64>() * PI * 2.0;
    let starter_distance = (min+range) / 3_f64.sqrt();
    let center = Vec2{x:0.5, y:0.5};
    let mut points = vec![
        Vec2::from_angle(rand_start_angle).mut_mult(starter_distance).mut_add(&center), 
        Vec2::from_angle(rand_start_angle + 2.0*PI/3.0).mut_mult(starter_distance).mut_add(&center), 
        Vec2::from_angle(rand_start_angle + 4.0*PI/3.0).mut_mult(starter_distance).mut_add(&center)
    ];
    let mut triangles: Vec<([usize; 3], [Option<(usize, usize)>;3])> = vec![([0, 1, 2], [None, None, None])];
    let mut next = vec![1, 2, 0];
    let mut prev = vec![2, 0, 1];
    let mut tri = vec![(0, 0), (0, 1), (0, 2)];
    let mut hull_start = 0;
    let mut total_hull_length: f64 = 3.0*(min+range);
    let mut hull_lengths: Vec<f64> = vec![min+range, min+range, min+range];
    /*Generate*/
    loop{
        let mut new_points: Vec<Vec2> = vec![];
        {//Generates new points
            //subdivide our length into as many min+range segments as possible
            let num_points: usize = (total_hull_length / (min+range)).floor() as usize;
            //the base distance between each new point
            let base_distance: f64 = total_hull_length/(num_points as f64);
            //a random shift for all points along the hull: keeps the very first point the lowest one, and the last point doesnt wrap arround the hull
            let global_shift: f64 = rng().gen::<f64>()*(base_distance - range);

            let mut position: f64 = hull_lengths[0];
            let mut point: usize = hull_start;

            for i in 0..num_points{
                let point_position = (i as f64)*base_distance + global_shift + rng().gen::<f64>()*range;
                while position < point_position {
                    point = next[point];
                    position += hull_lengths[point];
                }
                let mut step_along = points[next[point]].get_step_to(&points[point]).normalized();
                let step_off = step_along.rotate_90_ccw().mut_mult(rng().gen::<f64>()*range + min);
                step_along = step_along.mut_mult(position - point_position);//step from the second point in our current edge to our projection spot
                new_points.push(points[next[point]].add(&step_along).add(&step_off));
                if !new_points.last().unwrap().is_in_region() {new_points.pop();}
            }
        }
        if new_points.len() == 0 {break;}
        let mut altered_triangles: HashSet<usize> = HashSet::new();
        {//Adds new points
            next.resize(points.len()+new_points.len(), 0);
            prev.resize(points.len()+new_points.len(), 0);
            tri.resize(points.len()+new_points.len(), (0, 0));
            hull_lengths.resize(points.len()+new_points.len(), 0.0);

            new_points.sort_by(|a, b| {
                if a.squared_distance_from(&center) < b.squared_distance_from(&center){Ordering::Less}else{Ordering::Greater}
            });
            for p in new_points.into_iter(){
                points.push(p);
                let point = points.len()-1;

                let start: usize;
                let end: usize;
                {//Find first and last hull edge visible to the point
                    let mut forward_point: usize = hull_start;
                    let mut backward_point: usize = hull_start;
                    match Vec2::is_ccw(&points[point], &points[hull_start], &points[next[hull_start]]){
                        true => {
                            start = loop{
                                if !Vec2::is_ccw(&points[point], &points[forward_point], &points[next[forward_point]]){
                                    break forward_point;
                                }
                                forward_point = next[forward_point];
                            };
                            end = loop{
                                if !Vec2::is_ccw(&points[point], &points[backward_point], &points[next[backward_point]]){
                                    break backward_point;
                                }
                                backward_point = prev[backward_point];
                            };
                        },
                        false => {
                            start = loop{
                                if Vec2::is_ccw(&points[point], &points[backward_point], &points[next[backward_point]]){
                                    break next[backward_point];
                                }
                                backward_point = prev[backward_point];
                            };
                            end = loop{
                                if Vec2::is_ccw(&points[point], &points[forward_point], &points[next[forward_point]]){
                                    break prev[forward_point];
                                }
                                forward_point = next[forward_point];
                            };
                        }
                    };
                }
                
                let first_tri: usize;
                let mut last_tri: usize;
                let mut sub_length: f64 = 0.0;
                {//Add triangles
                    //add first triangle
                    triangles.push((
                        [start, point, next[start]], 
                        [None, None, Some((tri[start].0, (tri[start].1 + 2) % 3))]
                    ));
                    altered_triangles.insert(triangles.len()-1);
                    first_tri = triangles.len()-1;
                    last_tri = triangles.len()-1;
                    //alter triangle adjacencies
                    triangles[tri[start].0].1[tri[start].1] = Some((first_tri, 1));
                    let mut current: usize = next[start];
                    sub_length += hull_lengths[start];
                    //go through and add the rest of the triangles
                    while prev[current] != end {
                        sub_length += hull_lengths[current];
                        triangles.push((
                            [current, point, next[current]],
                            [Some((triangles.len()-1, 0)), None, Some((tri[current].0, (tri[current].1 + 2)%3))]
                        ));
                        altered_triangles.insert(triangles.len()-1);
                        last_tri = triangles.len()-1;
                        triangles[tri[current].0].1[tri[current].1] = Some((triangles.len()-1, 1));
                        let index = triangles.len()-2;
                        triangles[index].1[1] = Some((triangles.len()-1, 2));
                        current = next[current];
                    }
                }
                
                {// fix the hull
                    let new_end = next[end];
                    next[start] = point; next[point] = new_end;
                    prev[new_end] = point; prev[point] = start;
                    tri[start] = (first_tri, 0);
                    tri[point] = (last_tri, 1);
                    hull_start = point;
                    hull_lengths[start] = points[start].distance_from(&points[point]);
                    hull_lengths[point] = points[point].distance_from(&points[new_end]);
                    total_hull_length -= sub_length - hull_lengths[start] - hull_lengths[point];
                }
            }
        }
        {//Fixes triangulation
            while !altered_triangles.is_empty(){
                let items = altered_triangles.clone();
                'outer: for triangle in items{
                    for (i, adj) in triangles[triangle].1.iter().enumerate(){
                        if adj.map_or_else(|| true, |adj| {
                            !Vec2::point_is_circumscibed(
                                &points[triangles[adj.0].0[adj.1]], 
                                &points[triangles[triangle].0[0]], 
                                &points[triangles[triangle].0[1]], 
                                &points[triangles[triangle].0[2]]
                            )
                        }) {continue;}
                        let adj = adj.unwrap();
                        //swap the two interchanged points
                        triangles[triangle].0[(1+i)%3] = triangles[adj.0].0[adj.1];
                        triangles[adj.0].0[(adj.1+2)%3] = triangles[triangle].0[(2+i)%3];
                        //swap the two interchanged adjacencies
                        triangles[triangle].1[i%3] = triangles[adj.0].1[(adj.1+2)%3];
                        triangles[adj.0].1[(adj.1+1)%3] = triangles[triangle].1[(1+i)%3];
                        //create the new interreferencial adjacencies
                        triangles[triangle].1[(1+i)%3] = Some((adj.0, (adj.1+1)%3));
                        triangles[adj.0].1[(adj.1+2)%3] = Some((triangle, i));
                        //if necessary, alter other adjacent triangles adjacency to the new triangles index
                        if triangles[triangle].1[i].is_some() {
                            let adj_index = (triangles[triangle].1[i].unwrap().1+1)%3;
                            let adj_triangle = triangles[triangle].1[i].unwrap().0;
                            triangles[adj_triangle].1[adj_index] = Some((triangle, (2+i)%3));
                        }
                        if triangles[adj.0].1[(adj.1+1)%3].is_some() {
                            let adj_index = (triangles[adj.0].1[(adj.1+1)%3].unwrap().1+1)%3;
                            let adj_triangle = triangles[adj.0].1[(adj.1+1)%3].unwrap().0;
                            triangles[adj_triangle].1[adj_index] = Some((adj.0, adj.1));
                        }
                        //check every possible hull segment, and alter their triangle pointer if necessary
                        if next[triangles[triangle].0[(2+i)%3]] == triangles[triangle].0[i] {
                            tri[triangles[triangle].0[(2+i)%3]] = (triangle, (2+i)%3);
                        }
                        if next[triangles[triangle].0[i]] == triangles[triangle].0[(1+i)%3] {
                            tri[triangles[triangle].0[i]] = (triangle, i);
                        }
                        if next[triangles[adj.0].0[adj.1]] == triangles[adj.0].0[(adj.1+1)%3] {
                            tri[triangles[adj.0].0[adj.1]] = (adj.0, adj.1);
                        }
                        if next[triangles[adj.0].0[(adj.1+1)%3]] == triangles[adj.0].0[(adj.1+2)%3] {
                            tri[triangles[adj.0].0[(adj.1+1)%3]] = (adj.0, (adj.1+1)%3);
                        }
                        altered_triangles.insert(adj.0);
                        continue 'outer;
                    }
                    altered_triangles.remove(&triangle);
                }
            }    
        }
    }
    let mut hull: Vec<usize> = vec![];
    let mut index = hull_start;
    while index != hull_start {
        hull.push(index);
        index = next[index];
    }
    return (points, triangles, hull);
}


/// a struct storing a single triangle
#[derive(Debug)]
pub struct Triangle{
    /// a list of 3 point indices in it's parent's point vector
    pub points: Vec<usize>,
    /// a list of 3 possible adjacencies a triangle can have. 
    /// first usize is the adjacent triangle's index into the parent triangle vector. 
    /// Second usize is the index of the adjacent triangle's point which isnt connected to this triangle: index is in the adjacent triangles point array, so either 0, 1, or 2
    pub adj: Vec<Option<(usize, usize)>>
}
impl Triangle{
    /// determines if a specific adjacency for a triangle needs to swap its shared edge in order to make a better triangulation
    pub fn fix_adj(&self, i: usize, points: &Vec<Vec2>, triangles: &Vec<Triangle>) -> bool{
        return self.adj[i].map_or_else(|| false, |adj| {
                return Vec2::point_is_circumscibed(
                    &points[triangles[adj.0].points[adj.1]], 
                    &points[self.points[0]], 
                    &points[self.points[1]], 
                    &points[self.points[2]]
                );
        });
    }
}


/// a struct which contains information about the convex hull of a set of points. The hull is sorted in CCW order
#[derive(Debug)]
pub struct Hull{
    /// Vector with length = num points. Each value represents the point of equal index.
    /// the value is the index of the next point in the hull.
    pub next: Vec<usize>,
    /// Vector with length = num points. Each value represents the point of equal index.
    /// the value is the index of the prev point in the hull.
    pub prev: Vec<usize>,
    /// Vector with length = num points. Each value represents the point of equal index.
    /// the value represents the triangle connected to the point, and the hull point.
    /// (index of triangle, index of current hull point in triangles points vector)
    pub tri: Vec<(usize, usize)>,
    /// the index of the first point in the hull. is arbitrary, could be any point in the hull
    pub start: usize,
    /// vector that contains a list of consecutive hull point indices that is calculated once all points are generated.
    pub hull: Vec<usize>
}
impl Hull{
    /// constructs an empty new hull
    pub fn new() -> Hull{ Hull{next: vec![], prev: vec![], tri: vec![], start: 0, hull: vec![]}}
    /// defines the starting values for the hull 
    pub fn initialize(&mut self) {
        self.next = vec![1, 2, 0];
        self.prev = vec![2, 0, 1];
        self.tri = vec![(0, 0), (0, 1), (0, 2)];
        self.start = 0;
    }
    /// extends the hull's vectors by enough slots to fit the newly generated points
    pub fn extend(&mut self, num: usize){
        for _ in 0..num{
            self.next.push(0); self.prev.push(0);
            self.tri.push((0, 0));
        }
    }
    /// generates a list of new points given a min distance and range, as well as the current points
    pub unsafe fn get_points(&self, min: f64, range: f64, points: &Vec<Vec2>) -> Vec<Vec2> {
        //calculate the lengths of the hull segments, and the total hull length
        let mut total_length: f64 = 0.0;
        let mut lengths: Vec<f64> = vec![];
        let mut cur_point: usize = self.start;
        loop{
            lengths.push(points[cur_point].distance_from(&points[self.next[cur_point]]));
            total_length += lengths[lengths.len()-1];
            cur_point = self.next[cur_point];
            if cur_point == self.start {break;}
        }
        //subdivide our length into as many min+range segments as possible
        let num_points: f64 = (total_length / (min+range)).floor();
        //a random shift for all points along the hull: keeps the very first point the lowest one, and the last point doesnt wrap arround the hull
        let global_shift: f64 = rng().gen::<f64>()*(total_length/num_points - range);
        //the base distance between each new point
        let base_distance: f64 = total_length/num_points;
        
        //variables to track values as we generate our points
        let mut cur_rand_shift: f64;//a small random shift along the hull for each point
        let mut cur_point_pos: f64;
        let mut cur_pos: f64 = lengths[0];
        let mut cur_point: usize = self.start;
        let mut cur_index: usize = 0;
        let mut cur_rand_dist: f64;//the random distance away from th hull to project, between min and min+range
        let mut new_points: Vec<Vec2> = vec![];
        for i in 0..(num_points as usize){
            //get point specific random values
            cur_rand_shift = rng().gen::<f64>()*range;
            cur_point_pos = (i as f64)*base_distance + global_shift + cur_rand_shift;
            cur_rand_dist = rng().gen::<f64>()*range + min;
            //keep going through hull segments until we encapsulate our current goal of cur_point_pos.
            while cur_pos < cur_point_pos {
                cur_index += 1;
                cur_point = self.next[cur_point];
                cur_pos += lengths[cur_index];
            }
            //vector math to get new point
            let mut step_along = points[self.next[cur_point]].get_step_to(&points[cur_point]).normalized();
            let step_off = step_along.rotate_90_ccw().mut_mult(cur_rand_dist);//projection away from hull
            step_along = step_along.mut_mult(cur_pos - cur_point_pos);//step from the second point in our current edge to our projection spot
            let new_point = points[self.next[cur_point]].add(&step_along).add(&step_off);
            if new_point.is_in_region() {new_points.push(new_point);}
        }
        return new_points;
    }
    /// given a point, finds the range of hull points visible to the point
    pub fn find_visible_hull_points(&self, point: &Vec2, points: &Vec<Vec2>) -> (usize, usize){
        let mut forward_point: usize = self.start;
        let mut backward_point: usize = self.start;
        match Vec2::is_ccw(point, &points[self.start], &points[self.next[self.start]]){
            true => {return (
                loop{
                    if !Vec2::is_ccw(point, &points[forward_point], &points[self.next[forward_point]]){
                        break forward_point;
                    }
                    forward_point = self.next[forward_point];
                },
                loop{
                    if !Vec2::is_ccw(point, &points[backward_point], &points[self.next[backward_point]]){
                        break backward_point;
                    }
                    backward_point = self.prev[backward_point];
                }
            );},
            false => {return (
                loop{
                    if Vec2::is_ccw(point, &points[backward_point], &points[self.next[backward_point]]){
                        break self.next[backward_point];
                    }
                    backward_point = self.prev[backward_point];
                },
                loop{
                    if Vec2::is_ccw(point, &points[forward_point], &points[self.next[forward_point]]){
                        break self.prev[forward_point];
                    }
                    forward_point = self.next[forward_point];
                }
            );}
        };
    }
    /// Generates the final hull vector
    pub fn generate_hull(&mut self) -> Vec<usize>{
        let mut cur: usize = self.start;
        let mut hull: Vec<usize> = vec![];
        loop{
            hull.push(cur);
            cur = self.next[cur];
            if cur == self.start {break;}
        }
        return hull;
    }
}