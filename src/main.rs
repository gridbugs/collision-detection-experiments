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
    use aabb::*;
    use cgmath::{vec2, Vector2};
    use loose_quad_tree::*;
    use naive::*;
    use test::{self, Bencher};

    macro_rules! collision_detection_bench {
        ($name:ident, $typ:ident, $size:expr) => {
            mod $name {
                use super::*;
                use rand::{Rng, SeedableRng, StdRng};

                fn init() -> (Vec<(u32, Aabb)>, $typ<u32>) {
                    let mut rng = StdRng::from_seed(&[0]);

                    let entity_size = vec2(5., 5.);
                    let world_size = vec2(1000.0, 1000.0);
                    let coord_max: Vector2<u32> = (world_size - entity_size).cast().unwrap();
                    let collision_detection = $typ::new(world_size);

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

                    (entities, collision_detection)
                }

                fn insert_all(entities: &[(u32, Aabb)], collision_detection: &mut $typ<u32>) {
                    for (id, aabb) in entities.iter() {
                        collision_detection.insert(*aabb, *id);
                    }
                }

                fn compare_all(entities: &[(u32, Aabb)], collision_detection: &$typ<u32>) {
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
                }

                #[bench]
                fn compare(b: &mut Bencher) {
                    let (entities, mut collision_detection) = init();
                    insert_all(&entities, &mut collision_detection);
                    b.iter(|| {
                        compare_all(&entities, &collision_detection);
                    });
                }

                #[bench]
                fn insert(b: &mut Bencher) {
                    let (entities, mut collision_detection) = init();
                    b.iter(|| {
                        insert_all(&entities, &mut collision_detection);
                        collision_detection.clear();
                    });
                }

                #[bench]
                fn insert_and_compare(b: &mut Bencher) {
                    let (entities, mut collision_detection) = init();
                    b.iter(|| {
                        insert_all(&entities, &mut collision_detection);
                        compare_all(&entities, &collision_detection);
                        collision_detection.clear();
                    });
                }

            }
        };
    }

    collision_detection_bench!(naive_32, NaiveCollisionDetection, 32);
    collision_detection_bench!(naive_64, NaiveCollisionDetection, 64);
    collision_detection_bench!(naive_128, NaiveCollisionDetection, 128);
    collision_detection_bench!(naive_256, NaiveCollisionDetection, 256);
    collision_detection_bench!(naive_512, NaiveCollisionDetection, 512);
    collision_detection_bench!(naive_1024, NaiveCollisionDetection, 1024);
    collision_detection_bench!(loose_quad_32, LooseQuadTree, 32);
    collision_detection_bench!(loose_quad_64, LooseQuadTree, 64);
    collision_detection_bench!(loose_quad_128, LooseQuadTree, 128);
    collision_detection_bench!(loose_quad_256, LooseQuadTree, 256);
    collision_detection_bench!(loose_quad_512, LooseQuadTree, 512);
    collision_detection_bench!(loose_quad_1024, LooseQuadTree, 1024);
    collision_detection_bench!(loose_quad_2048, LooseQuadTree, 2048);
    collision_detection_bench!(loose_quad_4096, LooseQuadTree, 4096);
    collision_detection_bench!(loose_quad_8192, LooseQuadTree, 8192);

}
