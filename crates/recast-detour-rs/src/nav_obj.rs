use std::fs;
/// A Simplied but augmented obj file reader for  navmesh
use std::io;
use std::io::BufRead;
use std::path::Path;

#[derive(Debug)]
pub enum Error {
    FileOpen(io::Error),
    ReadLine(io::Error),
    LineError,
    ParseError,
}

macro_rules! impl_from {
    { $($ty:ty),+ } => {
        $(
            impl From<$ty> for Error {
                fn from(_: $ty) -> Error {
                    Error::ParseError
                }
            }
        )*
    }
}

impl_from!(std::num::ParseFloatError, std::num::ParseIntError);

type Result<T> = std::result::Result<T, Error>;

pub struct NavObjFile {
    pub data: crate::NavMeshData,
    pub name: String,
}

fn parse_data_line(obj_file: &mut NavObjFile, line: &String) -> Result<()> {
    let mut items = line.split_whitespace();
    let first_item = items.next().ok_or(Error::LineError)?.to_string();
    let second_item = items.next().ok_or(Error::LineError)?.to_string();

    match first_item.as_ref() {
        "walkable_height" => obj_file.data.walkable_height = second_item.parse::<f32>()?,
        "walkable_radius" => obj_file.data.walkable_radius = second_item.parse::<f32>()?,
        "walkable_climb" => obj_file.data.walkable_climb = second_item.parse::<f32>()?,
        "cell_size" => obj_file.data.cell_size = second_item.parse::<f32>()?,
        "cell_height" => obj_file.data.cell_height = second_item.parse::<f32>()?,
        "g" => obj_file.name = second_item,
        "v" => {
            let f1 = second_item.parse::<f32>()?;
            let f2 = items.next().ok_or(Error::LineError)?.parse::<f32>()?;
            let f3 = items.next().ok_or(Error::LineError)?.parse::<f32>()?;
            obj_file.data.vertices.push(f1);
            obj_file.data.vertices.push(f2);
            obj_file.data.vertices.push(f3);
        }
        "f" => {
            let i1 = second_item.parse::<u16>()?;
            let i2 = items.next().ok_or(Error::LineError)?.parse::<u16>()?;
            let i3 = items.next().ok_or(Error::LineError)?.parse::<u16>()?;
            obj_file.data.indices.push(i1 - 1);
            obj_file.data.indices.push(i2 - 1);
            obj_file.data.indices.push(i3 - 1);
        }

        _ => {}
    }

    Ok(())
}

impl NavObjFile {
    pub fn open(path: &Path) -> Result<NavObjFile> {
        let file = fs::File::open(path).map_err(Error::FileOpen)?;
        let file = io::BufReader::new(&file);

        NavObjFile::from_buf(file)
    }

    fn from_buf<R: BufRead>(file: R) -> Result<NavObjFile> {
        let mut nav_file = NavObjFile {
            data: crate::NavMeshData::default(),
            name: "".into(),
        };

        for line in file.lines() {
            let line = line.map_err(Error::ReadLine)?;
            let mut line = line.trim().to_string();

            let first = line.trim().split_whitespace().next();

            match first {
                Some("#") => {
                    line.remove(0);
                    let _ = parse_data_line(&mut nav_file, &line);
                }
                _ => {
                    let _ = parse_data_line(&mut nav_file, &line);
                },
            }
        }

        Ok(nav_file)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nav_obj_smoke_test() {
        let test_data = r#"
# walkable_height 2
# walkable_radius 0.5
# walkable_climb 0.75
# cell_size 0.1666667
# cell_height 0.1666667

g navmesh
v -6.833334 0.01006675 0
v -6.666668 0.01006675 -0.166666
v -7.666668 0.01006675 -0.3333321
v -16.83333 0.01006675 -0.3333321
f 1 2 3
f 1 3 4
f 1 4 5
f 6 7 8
f 9 10 11
"#;
        let r = NavObjFile::from_buf(test_data.as_bytes()).unwrap();

        assert_eq!(r.data.walkable_height, 2.0);
        assert_eq!(r.data.walkable_radius, 0.5);
        assert_eq!(r.data.walkable_climb, 0.75);
        assert_eq!(r.data.cell_size, 0.1666667);
        assert_eq!(r.data.cell_height, 0.1666667);
        assert_eq!(r.name, "navmesh");

        assert_eq!(r.data.vertices[0], -6.833334);
        assert_eq!(r.data.vertices[1], 0.01006675);
        assert_eq!(r.data.vertices[2], 0.0);
        assert_eq!(r.data.vertices[3], -6.666668);
        assert_eq!(r.data.vertices[4], 0.01006675);
        assert_eq!(r.data.vertices[5], -0.166666);

        assert_eq!(r.data.indices[0], 0);
        assert_eq!(r.data.indices[1], 1);
        assert_eq!(r.data.indices[2], 2);
        assert_eq!(r.data.indices[3], 0);
        assert_eq!(r.data.indices[4], 2);
        assert_eq!(r.data.indices[5], 3);
    }
}
