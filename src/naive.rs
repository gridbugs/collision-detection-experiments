use aabb::Aabb;
use cgmath::Vector2;

pub struct NaiveCollisionDetection<T> {
    items: Vec<(Aabb, T)>,
}

impl<T> NaiveCollisionDetection<T> {
    pub fn new(_size: Vector2<f32>) -> Self {
        Self { items: Vec::new() }
    }
    pub fn insert(&mut self, aabb: Aabb, t: T) {
        self.items.push((aabb, t));
    }
    pub fn for_each_intersection<F: FnMut(&Aabb, &T)>(&self, aabb: &Aabb, mut f: F) {
        for &(ref other_aabb, ref t) in self.items.iter() {
            if aabb.is_intersecting(other_aabb) {
                f(other_aabb, t);
            }
        }
    }
}
