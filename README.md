# recast-detour-rs

[Latest Version]: https://img.shields.io/crates/v/recast-detour-rs.svg
[crates.io]: https://crates.io/crates/recast-detour-rs

### This project do not and will not be a full function Recast & Detour library. 

This project do the followings things

* Use `cmake` crate to build the `Recast & Detour` c++ library.
* Add an very simple c-api for `Detour` Query.
* Manual binding for that c-api in rust crate (`recast-detour-sys`)
* An actual rust crate for provide a better usage (`recast-detour-rs`)
* An augmented obj file format reader for reading NavMesh from a file
* An demo implementation for *Unity NavMesh* Exporter


### Basic Usage

```rust

#[cfg(test)]
mod tests {
    use super::*;
    use insta::*;

    fn simple_mesh() -> NavMeshData {
        let vertices = vec![
            0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0, 0.0, 10.0, 0.0, 0.0, 10.0,
        ];

        let indices = vec![0, 1, 2, 0, 2, 3];

        NavMeshData {
            vertices,
            indices,
            walkable_height: 0.2,
            walkable_radius: 0.2,
            walkable_climb: 0.2,
            cell_size: 0.1,
            cell_height: 0.1,
        }
    }

    #[test]
    fn test_simple_path() {
        assert_eq!("0.0.1", version());
        let mesh = simple_mesh();

        let q = RecastQuery::new_from_mesh(mesh).unwrap();
        let p = q
            .find_path((0.2, 0.1, 0.4).into(), (0.8, 0.1, 0.5).into(), 0.2)
            .unwrap();

        assert_debug_snapshot_matches!(p, @r###"Point(
    [
        0.29999924,
        0.0,
        0.29999924
    ]
)"###);
    }
}

```

