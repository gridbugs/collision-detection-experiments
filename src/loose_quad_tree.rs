use aabb::*;
use cgmath::{vec2, Vector2};
use std::num::NonZeroUsize;

#[derive(Debug, Clone)]
pub struct LooseQuadTree<T> {
    seq: u64,
    nodes: Vec<Node<T>>,
    size: Vector2<f32>,
    next_free: usize,
}

#[derive(Debug, Clone)]
struct Node<T> {
    items: Vec<(Aabb, T)>,
    child_offset: Option<NonZeroUsize>,
    seq: u64,
}

impl<T> Default for Node<T> {
    fn default() -> Self {
        Self {
            items: Vec::new(),
            child_offset: None,
            seq: 0,
        }
    }
}
impl<T> Node<T> {
    fn reuse(&mut self, seq: u64) {
        self.items.clear();
        self.child_offset = None;
        self.seq = seq;
    }
}

impl<T> LooseQuadTree<T> {
    const TOP_LEFT: usize = 0;
    const TOP_RIGHT: usize = 1;
    const BOTTOM_LEFT: usize = 2;
    const BOTTOM_RIGHT: usize = 3;
    const NUM_CHILDREN: usize = 4;

    pub fn new(size: Vector2<f32>) -> Self {
        Self {
            seq: 1,
            nodes: vec![Default::default()],
            size,
            next_free: 1,
        }
    }

    pub fn clear(&mut self) {
        self.seq += 1;
        self.nodes[0].reuse(self.seq);
    }

    pub fn insert(&mut self, aabb: Aabb, t: T) {
        let mut centre = aabb.centre();
        let mut index = 0;
        let mut max_size = self.size / 2.;
        let nodes = &mut self.nodes;
        let next_free = &mut self.next_free;
        loop {
            let child_offset = {
                while nodes.len() <= index {
                    nodes.push(Default::default());
                }
                let mut node = &mut nodes[index];
                if node.seq != self.seq {
                    node.reuse(self.seq);
                }
                if aabb.size.x > max_size.x || aabb.size.y > max_size.y {
                    node.items.push((aabb, t));
                    break;
                }
                node.child_offset
                    .get_or_insert_with(|| {
                        let free = *next_free;
                        *next_free += Self::NUM_CHILDREN;
                        NonZeroUsize::new(free).expect("unexpected state")
                    })
                    .get() as usize
            };
            if centre.x < max_size.x {
                if centre.y < max_size.y {
                    index = child_offset + Self::TOP_LEFT;
                } else {
                    index = child_offset + Self::BOTTOM_LEFT;
                    centre.y = centre.y - max_size.y;
                }
            } else {
                if centre.y < max_size.y {
                    index = child_offset + Self::TOP_RIGHT;
                    centre.x = centre.x - max_size.x;
                } else {
                    index = child_offset + Self::BOTTOM_RIGHT;
                    centre = centre - max_size;
                }
            }
            max_size = max_size / 2.;
        }
    }

    fn for_each_intersection_rec<F: FnMut(&Aabb, &T)>(
        nodes: &[Node<T>],
        current_index: usize,
        current_node_aabb: &Aabb,
        aabb_to_test: &Aabb,
        f: &mut F,
    ) {
        if let Some(node) = nodes.get(current_index) {
            for &(ref aabb, ref t) in node.items.iter() {
                if aabb.is_intersecting(aabb_to_test) {
                    f(aabb, t);
                }
            }
            if let Some(child_offset) = node.child_offset {
                let child_offset = child_offset.get() as usize;
                let AabbSplitFour {
                    top_left,
                    top_right,
                    bottom_left,
                    bottom_right,
                } = current_node_aabb.split_four();
                if top_left.double_about_centre().is_intersecting(aabb_to_test) {
                    Self::for_each_intersection_rec(
                        nodes,
                        child_offset + Self::TOP_LEFT,
                        &top_left,
                        aabb_to_test,
                        f,
                    );
                }
                if top_right
                    .double_about_centre()
                    .is_intersecting(aabb_to_test)
                {
                    Self::for_each_intersection_rec(
                        nodes,
                        child_offset + Self::TOP_RIGHT,
                        &top_right,
                        aabb_to_test,
                        f,
                    );
                }
                if bottom_left
                    .double_about_centre()
                    .is_intersecting(aabb_to_test)
                {
                    Self::for_each_intersection_rec(
                        nodes,
                        child_offset + Self::BOTTOM_LEFT,
                        &bottom_left,
                        aabb_to_test,
                        f,
                    );
                }
                if bottom_right
                    .double_about_centre()
                    .is_intersecting(aabb_to_test)
                {
                    Self::for_each_intersection_rec(
                        nodes,
                        child_offset + Self::BOTTOM_RIGHT,
                        &bottom_right,
                        aabb_to_test,
                        f,
                    );
                }
            }
        }
    }

    pub fn for_each_intersection<F: FnMut(&Aabb, &T)>(&self, aabb: &Aabb, mut f: F) {
        let root_aabb = Aabb::new(vec2(0., 0.), self.size);
        Self::for_each_intersection_rec(&self.nodes, 0, &root_aabb, aabb, &mut f);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use naive::NaiveCollisionDetection;
    use rand::{Rng, SeedableRng, StdRng};

    #[test]
    fn basic_collision() {
        let mut q = LooseQuadTree::new(vec2(10., 10.));
        q.insert(Aabb::new(vec2(1., 4.), vec2(2., 2.)), 0);
        q.insert(Aabb::new(vec2(6., 4.), vec2(2., 2.)), 1);
        q.insert(Aabb::new(vec2(2., 5.), vec2(1., 1.)), 2);
        q.insert(Aabb::new(vec2(0., 0.), vec2(10., 10.)), 3);
        q.insert(Aabb::new(vec2(5., 0.), vec2(5., 5.)), 4);

        let mut collisions = Vec::new();
        q.for_each_intersection(&Aabb::new(vec2(2., 5.), vec2(2., 2.)), |_aabb, id| {
            collisions.push(*id);
        });

        collisions.sort();

        assert_eq!(collisions, vec![0, 2, 3]);
    }

    fn random_entities<R: Rng>(num: u16, rng: &mut R) -> Vec<(u16, Aabb)> {
        (0..num)
            .map(|id| {
                let top_left_coord =
                    vec2((rng.next_u32() % 90) as f32, (rng.next_u32() % 90) as f32);
                let size =
                    vec2((rng.next_u32() % 5) as f32, (rng.next_u32() % 5) as f32) + vec2(0.1, 0.1);
                let aabb = Aabb::new(top_left_coord, size);
                (id, aabb)
            })
            .collect::<Vec<_>>()
    }

    fn compare_to_naive_impl(
        entities: &[(u16, Aabb)],
        q: &LooseQuadTree<u16>,
        n: &NaiveCollisionDetection<u16>,
    ) {
        for (_id, aabb) in entities.iter() {
            let mut qc = Vec::new();
            let mut nc = Vec::new();

            q.for_each_intersection(&aabb, |_aabb, &id| qc.push(id));
            n.for_each_intersection(&aabb, |_aabb, &id| nc.push(id));

            qc.sort();
            nc.sort();

            assert_eq!(qc, nc);
        }
    }

    #[test]
    fn compare_to_naive() {
        let mut rng = StdRng::from_seed(&[0]);
        let size = vec2(100., 100.);
        let mut q = LooseQuadTree::new(size);
        let mut n = NaiveCollisionDetection::new(size);
        let entities = random_entities(1000, &mut rng);
        for (id, aabb) in entities.iter() {
            q.insert(*aabb, *id);
            n.insert(*aabb, *id);
        }
        compare_to_naive_impl(&entities, &q, &n);
    }

    #[test]
    fn clear_and_reuse() {
        let mut rng = StdRng::from_seed(&[0]);
        let size = vec2(100., 100.);
        let mut q = LooseQuadTree::new(size);
        let mut n = NaiveCollisionDetection::new(size);
        let entities = random_entities(1000, &mut rng);
        for (id, aabb) in entities.iter() {
            q.insert(*aabb, *id);
            n.insert(*aabb, *id);
        }
        compare_to_naive_impl(&entities, &q, &n);
        q.clear();
        n.clear();
        let entities = random_entities(1000, &mut rng);
        for (id, aabb) in entities.iter() {
            q.insert(*aabb, *id);
            n.insert(*aabb, *id);
        }
        compare_to_naive_impl(&entities, &q, &n);
    }
}
