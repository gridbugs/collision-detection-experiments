#![feature(test)]
#![feature(nonzero)]

extern crate cgmath;
extern crate fnv;
extern crate rand;
extern crate test;

mod aabb;
mod loose_quad_tree;
mod naive;

fn main() {}

#[cfg(test)]
mod bench {
    use rand::{Rng, SeedableRng, StdRng};
    use test::{self, Bencher};

    use aabb::*;
    use cgmath::{vec2, Vector2};
    use loose_quad_tree::*;
    use naive::*;

    macro_rules! collision_detection_bench {
        ($name:ident, $initial:expr, $size:expr) => {
            #[bench]
            fn $name(b: &mut Bencher) {
                let mut rng = StdRng::from_seed(&[0]);

                let entity_size = vec2(5., 5.);
                let world_size = vec2(1000.0, 1000.0);
                let coord_max: Vector2<u32> = (world_size - entity_size).cast().unwrap();
                let mut collision_detection = $initial(world_size);

                let entities = (0..$size)
                    .map(|id| {
                        let top_left_coord = vec2(
                            (rng.next_u32() % coord_max.x) as f32,
                            (rng.next_u32() % coord_max.y) as f32,
                        );
                        let aabb = Aabb::new(top_left_coord, entity_size);
                        (id, aabb)
                    })
                    .collect::<Vec<_>>();

                for (id, aabb) in entities.iter() {
                    collision_detection.insert(*aabb, *id);
                }

                b.iter(|| {
                    for &(id, aabb) in entities.iter() {
                        collision_detection.for_each_intersection(
                            &aabb,
                            |intersect_aabb, &intersect_id| {
                                if id != intersect_id {
                                    test::black_box(intersect_aabb);
                                }
                            },
                        );
                    }
                });
            }
        };
    }

    collision_detection_bench!(naive_32, NaiveCollisionDetection::new, 32);
    collision_detection_bench!(naive_64, NaiveCollisionDetection::new, 64);
    collision_detection_bench!(naive_128, NaiveCollisionDetection::new, 128);
    collision_detection_bench!(naive_256, NaiveCollisionDetection::new, 256);
    collision_detection_bench!(naive_512, NaiveCollisionDetection::new, 512);
    collision_detection_bench!(naive_1024, NaiveCollisionDetection::new, 1024);
    collision_detection_bench!(loose_quad_32, LooseQuadTree::new, 32);
    collision_detection_bench!(loose_quad_64, LooseQuadTree::new, 64);
    collision_detection_bench!(loose_quad_128, LooseQuadTree::new, 128);
    collision_detection_bench!(loose_quad_256, LooseQuadTree::new, 256);
    collision_detection_bench!(loose_quad_512, LooseQuadTree::new, 512);
    collision_detection_bench!(loose_quad_1024, LooseQuadTree::new, 1024);
    collision_detection_bench!(loose_quad_2048, LooseQuadTree::new, 2048);
    collision_detection_bench!(loose_quad_4096, LooseQuadTree::new, 4096);
    collision_detection_bench!(loose_quad_8192, LooseQuadTree::new, 8192);

}
