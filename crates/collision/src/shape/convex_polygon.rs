use crate::shape::{Shape, ShapeType, MassProperties};
use physics_math::{Transform, Vec2, Aabb};
use smallvec::SmallVec;

pub struct ConvexPolygon {
    pub vertices: SmallVec<[Vec2; 8]>,
    pub normals:  SmallVec<[Vec2; 8]>,
    pub centroid: Vec2,
}

impl ConvexPolygon {
    pub fn new(vertices: SmallVec<[Vec2; 8]>) -> Self {
        debug_assert!(vertices.len() >= 3, "ConvexPolygon needs at least 3 vertices");
        debug_assert!(vertices.len() <= 8, "ConvexPolygon supports a maximum of 8 vertices for cache efficiency");
        let area = 0.5 * vertices.windows(2).map(|w| w[0].cross(w[1])).sum::<f32>();
        let mut vertices = vertices;
        if area < 0.0 {
            vertices.reverse();
        }
        let normals = vertices.windows(2).map(|w| {
            let edge = w[1] - w[0];
            edge.perp().normalize()
        }).chain(std::iter::once({
            let edge = vertices[0] - vertices[vertices.len() - 1];
            edge.perp().normalize()
        })).collect();
        let mut centroid = Vec2::zero();
        for i in 0..vertices.len() {
            let cross = vertices[i].cross(vertices[(i + 1) % vertices.len()]);
            centroid += (vertices[i] + vertices[(i + 1) % vertices.len()]) * cross;
        }
        centroid = centroid / (6.0 * area);
        Self {
            vertices,
            normals,
            centroid,
        }
    }
}

impl Shape for ConvexPolygon {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Polygon
    }

    fn compute_aabb(&self, transform: &Transform) -> Aabb {
        // Transform all vertices to world space and take the min/max.
        //
        // Algorithm:
        //   1. Initialize: min = Vec2::splat(f32::MAX), max = Vec2::splat(f32::MIN)
        //   2. For each vertex v in self.vertices:
        //        world_v = transform.apply(v)
        //        min = min.min_comp(world_v)
        //        max = max.max_comp(world_v)
        //   3. Return Aabb::new(min, max)
        //
        // Cost: n transform applications (n mul_vec + add).
        // For n <= 8 this is always cheap.
        let mut min = Vec2::splat(f32::MAX);
        let mut max = Vec2::splat(f32::MIN);
        for &v in &self.vertices {
            let world_v = transform.apply(v);
            min = min.min_comp(world_v);
            max = max.max_comp(world_v);
        }
        Aabb::new(min, max)
    }

    fn compute_mass_properties(&self, density: f32) -> MassProperties {
        let mut total_area = 0.0;
        let mut centroid = Vec2::zero();
        let mut inertia_origin = 0.0;
        for i in 0..self.vertices.len() {
            let p1 = Vec2::zero();
            let p2 = self.vertices[i];
            let p3 = self.vertices[(i + 1) % self.vertices.len()];
            let triangle_area = 0.5 * p2.cross(p3);
            total_area += triangle_area.abs();
            let triangle_centroid = (p1 + p2 + p3) / 3.0;
            centroid += triangle_centroid * triangle_area;
            inertia_origin += (triangle_area * density / 18.0) * (
                p1.dot(p1) + p2.dot(p2) + p3.dot(p3) +
                p1.dot(p2) + p2.dot(p3) + p1.dot(p3)
            );
        }
        centroid *= 1.0 / total_area;
        let mass = density * total_area;
        let inertia_centroid = inertia_origin - mass * centroid.dot(centroid);
        MassProperties {
            mass,
            inv_mass: if mass > 0.0 { 1.0 / mass } else { 0.0 },
            inertia: inertia_centroid,
            inv_inertia: if inertia_centroid > 0.0 { 1.0 / inertia_centroid } else { 0.0 },
            local_centroid: centroid,
        }
    }

    fn support(&self, direction: Vec2) -> Vec2 {
        let mut best_dot = f32::NEG_INFINITY;
        let mut best_point = self.vertices[0];
        for &v in &self.vertices {
            let d = v.dot(direction);
            if d > best_dot {
                best_dot = d;
                best_point = v;
            }
        }
        best_point
    }

    fn as_any(&self) -> &dyn std::any::Any { self }

    fn local_centroid(&self) -> Vec2 {
        self.centroid
    }
}


