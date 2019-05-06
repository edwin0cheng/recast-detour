use rand::rngs::StdRng;
use rand::Rng;
use rand::SeedableRng;
use recast_detour_rs::{NavMeshData, NavObjFile, Point, RecastQuery};
use std::path::Path;

fn get_point(i: u16, verts: &[f32]) -> Point {
    let i = i as usize;
    Point::new((verts[i * 3 + 0], verts[i * 3 + 1], verts[i * 3 + 2]))
}

fn get_triangle_points(data: &NavMeshData, i_tris: usize) -> (Point, Point, Point) {
    let i0 = data.indices[i_tris * 3 + 0];
    let i1 = data.indices[i_tris * 3 + 1];
    let i2 = data.indices[i_tris * 3 + 2];

    (
        get_point(i0, &data.vertices),
        get_point(i1, &data.vertices),
        get_point(i2, &data.vertices),
    )
}

fn center_point(data: &NavMeshData, i_tris: usize) -> Point {
    let (p0, p1, p2) = get_triangle_points(data, i_tris);

    // Get the center point
    let x = (p0.x() + p1.x() + p2.x()) / 3.0;
    let y = (p0.y() + p1.y() + p2.y()) / 3.0;
    let z = (p0.z() + p1.z() + p2.z()) / 3.0;

    Point::new((x, y, z))
}

fn sign(p1: &Point, p2: &Point, p3: &Point) -> f32 {
    (p1.x() - p3.x()) * (p2.z() - p3.z()) - (p2.x() - p3.x()) * (p1.z() - p3.z())
}

fn point_in_triangle(pt: &Point, v1: &Point, v2: &Point, v3: &Point) -> bool {
    let d1 = sign(pt, v1, v2);
    let d2 = sign(pt, v2, v3);
    let d3 = sign(pt, v3, v1);

    let has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
    let has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);

    !(has_neg && has_pos)
}

fn find_triangle(data: &NavMeshData, p: &Point) -> Option<usize> {
    let n_tris = data.indices.len() / 3; // Numbe of triangles

    for i in 0..n_tris {
        let (p0, p1, p2) = get_triangle_points(data, i);

        if point_in_triangle(&p, &p0, &p1, &p2) {
            return Some(i);
        }
    }

    None
}

fn rand_pick(data: &NavMeshData, rng: &mut impl Rng) -> (usize, usize) {
    let n_tris = data.indices.len() / 3; // Numbe of triangles
    (rng.gen_range(0, n_tris), rng.gen_range(9, n_tris))
}

#[test]
fn test_integration() {
    let r = NavObjFile::open(Path::new("tests/data/simplenav_hold.obj")).unwrap();

    let mesh = r.data.clone();
    let q = RecastQuery::new_from_mesh(r.data).unwrap();

    let mut rng = StdRng::seed_from_u64(1000);
    let mut test_pairs = vec![];
    for _ in 0..50 {
        let pair = rand_pick(&mesh, &mut rng);
        test_pairs.push(pair);
    }

    let test_pairs = &test_pairs;

    // randomly pick a point in the mesh
    for &(i, j) in test_pairs {
        let p0 = center_point(&mesh, i);
        let p1 = center_point(&mesh, j);
        dbg!((i, j, p0, p1));

        let path = q
            .find_path(p0, p1, 0.5)
            .map_err(|err| {
                panic!("Failed to find_path, i: {:#?} j: {:#?} {:#?}", i, j, err);
            })
            .unwrap();

        for p in path {
            find_triangle(&mesh, &p).unwrap_or_else(|| {
                panic!("Point should be in triangle {:#?}", p);
            });
        }
    }
}

#[test]
fn test_unstable_findpoint() {
    for _ in 0..100 {
        let r = NavObjFile::open(Path::new("tests/data/simplenav_hold.obj")).unwrap();                        
        let q = RecastQuery::new_from_mesh(r.data).unwrap();

        q.find_poly(Point::new((-6.6666665, 0.08333433, -6.6666665)), 0.4)
            .unwrap();
    }
}
