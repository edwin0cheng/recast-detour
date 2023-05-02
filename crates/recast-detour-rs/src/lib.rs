use recastnavigation_sys::*;
use std::collections::HashMap;
use std::ptr;

mod nav_obj;
mod util;

pub use nav_obj::NavObjFile;

const POLYFLAGS_WALK: u16 = 0x1;

#[derive(Debug)]
pub struct RecastQuery {
    mesh: ptr::NonNull<dtNavMesh>,
    q: ptr::NonNull<dtNavMeshQuery>,
}

impl Drop for RecastQuery {
    fn drop(&mut self) {
        unsafe {
            dtFreeNavMeshQuery(self.q.as_ptr());
            dtFreeNavMesh(self.mesh.as_ptr());
        }
    }
}

#[derive(Debug)]
pub enum Error {
    CreateQueryError(String),
    FindPointError(String),
    FindPathError(String),
    PartialResult,
}

type Result<T> = std::result::Result<T, Error>;

/// A Navgation Mesh Data
#[derive(Debug, Default, Clone)]
pub struct NavMeshData {
    /// Vertices in world unit, length = 3 * Number of Vertices
    pub vertices: Vec<f32>,
    /// Indices,  length = 3 * Number of Triangles
    pub indices: Vec<u16>,
    /// Walkable height in nav mesh in World Unit
    pub walkable_height: f32,
    /// Walkable Radius in nav mesh in World Unit
    pub walkable_radius: f32,
    /// Walkable climb height in World Unit
    pub walkable_climb: f32,

    /// Cell size in world unit
    pub cell_size: f32,
    /// Cell height in world unit
    pub cell_height: f32,
}

fn compute_bb(vertices: &[f32]) -> ([f32; 3], [f32; 3]) {
    let mut bmin = [std::f32::MAX; 3];
    let mut bmax = [std::f32::MIN; 3];
    debug_assert!(vertices.len() % 3 == 0);

    for i in (0..vertices.len()).step_by(3) {
        bmin[0] = vertices[i + 0].min(bmin[0]);
        bmin[1] = vertices[i + 1].min(bmin[1]);
        bmin[2] = vertices[i + 2].min(bmin[2]);

        bmax[0] = vertices[i + 0].max(bmax[0]);
        bmax[1] = vertices[i + 1].max(bmax[1]);
        bmax[2] = vertices[i + 2].max(bmax[2]);
    }

    (bmin, bmax)
}

#[inline]
fn world_unit_to_cell_unit(f: f32, bmin: f32, cs: f32) -> u16 {
    let f = ((f - bmin) / cs).max(0.0);
    f.round() as u16
}

#[derive(Debug, Copy, Clone, Default, PartialEq)]
pub struct Point([f32; 3]);

impl Point {
    pub fn new((x, y, z): (f32, f32, f32)) -> Point {
        Point([x, y, z])
    }

    pub fn x(&self) -> f32 {
        self.0[0]
    }
    pub fn y(&self) -> f32 {
        self.0[1]
    }
    pub fn z(&self) -> f32 {
        self.0[2]
    }
}

impl From<(f32, f32, f32)> for Point {
    fn from(f: (f32, f32, f32)) -> Point {
        Point::new(f)
    }
}

pub fn remove_dup(verts: &[u16], indices: &[u16]) -> (Vec<u16>, Vec<u16>) {
    let mut verts_map: HashMap<(u16, u16, u16), u16> = HashMap::new();
    let mut idx_map: HashMap<u16, u16> = HashMap::new();

    let n_verts = verts.len() / 3;
    let mut rv = Vec::new();
    let mut ri = Vec::new();

    for i in 0..n_verts {
        let p = (verts[i * 3 + 0], verts[i * 3 + 1], verts[i * 3 + 2]);
        let i = i as u16;
        let new_i = verts_map.entry(p).or_insert_with(|| {
            let idx = rv.len() / 3;

            rv.push(p.0);
            rv.push(p.1);
            rv.push(p.2);

            idx as u16
        });

        idx_map.insert(i, *new_i);
    }

    for idx in indices {
        ri.push(*idx_map.get(idx).unwrap());
    }

    (rv, ri)
}

impl RecastQuery {
    /// Create a query from NavMesh
    pub fn new_from_mesh(data: NavMeshData) -> Result<RecastQuery> {
        assert!(data.vertices.len() % 3 == 0);
        assert!(data.indices.len() % 3 == 0);

        let (bmin, bmax) = compute_bb(&data.vertices);

        let mut cu_verts = Vec::new();

        // World Unit to Cell Unit
        for i in (0..data.vertices.len()).step_by(3) {
            for j in 0..3 {
                cu_verts.push(world_unit_to_cell_unit(
                    data.vertices[i + j],
                    bmin[j],
                    data.cell_size,
                ));
            }
        }
        assert!(data.vertices.len() == cu_verts.len());

        let (cu_verts, indices) = remove_dup(&cu_verts, &data.indices);

        let vert_count = cu_verts.len() / 3;
        let triangles_count = data.indices.len() / 3;

        let mut polys = Vec::with_capacity(triangles_count * 6);
        for i in 0..triangles_count as usize {
            polys.push(indices[i * 3 + 0]);
            polys.push(indices[i * 3 + 1]);
            polys.push(indices[i * 3 + 2]);
            polys.push(0);
            polys.push(0);
            polys.push(0);
        }

        util::build_mesh_adjacency(&mut polys, vert_count, 3);

        let poly_flags = vec![POLYFLAGS_WALK; triangles_count];

        // TODO(edwin):
        // We assume all polygon are in the same area
        let poly_areas = vec![0; triangles_count];

        let mut params = dtNavMeshCreateParams {
            verts: cu_verts.as_ptr(),
            vertCount: vert_count as i32,
            polys: polys.as_ptr(),
            polyFlags: poly_flags.as_ptr(),
            polyAreas: poly_areas.as_ptr(),
            polyCount: triangles_count as i32,
            nvp: 3,
            detailMeshes: std::ptr::null(),
            detailVerts: std::ptr::null(),
            detailVertsCount: 0,
            detailTris: std::ptr::null(),
            detailTriCount: 0,
            offMeshConVerts: std::ptr::null(),
            offMeshConRad: std::ptr::null(),
            offMeshConFlags: std::ptr::null(),
            offMeshConAreas: std::ptr::null(),
            offMeshConDir: std::ptr::null(),
            offMeshConUserID: std::ptr::null(),
            offMeshConCount: 0,
            userId: 0,
            tileX: 0,
            tileY: 0,
            tileLayer: 0,
            bmin,
            bmax,
            walkableHeight: data.walkable_height,
            walkableRadius: data.walkable_radius,
            walkableClimb: data.walkable_climb,
            cs: data.cell_size,
            ch: data.cell_height,
            buildBvTree: true,
        };

        let mut nav_data = std::ptr::null_mut();
        let mut nav_data_size = 0;

        // SAFETY: dtCreateNavMeshData manages its own memory and only mutates `nav_data` and
        // `nav_data_size`. `params` is only read.
        if !unsafe { dtCreateNavMeshData(&mut params, &mut nav_data, &mut nav_data_size) } {
            return Err(Error::CreateQueryError(
                "dtCreateNavMeshData failed!".to_owned(),
            ));
        }

        if nav_data.is_null() {
            return Err(Error::CreateQueryError(
                "dtCreateNavMeshData returned null data".to_owned(),
            ));
        }
        if nav_data_size == 0 {
            return Err(Error::CreateQueryError(
                "dtCreateNavMeshData returned zero size data".to_owned(),
            ));
        }

        // SAFETY: dtAllocNavMesh either allocates its own memory or returns null, which is handled
        // by NonNull.
        let mut mesh = match ptr::NonNull::new(unsafe { dtAllocNavMesh() }) {
            Some(mesh) => mesh,
            None => {
                return Err(Error::CreateQueryError(
                    "Could not allocate dtNavMesh".to_owned(),
                ))
            }
        };
        // SAFETY: We can turn the mesh into a mutable borrow, since only this scope has the
        // pointer, so it is exclusive. `init1` takes ownership of nav_data and otherwise only
        // mutates `mesh`.
        let status = unsafe {
            mesh.as_mut().init1(
                nav_data,
                nav_data_size,
                dtTileFlags_DT_TILE_FREE_DATA as i32,
            )
        };
        if (status & DT_FAILURE) != 0 {
            // SAFETY: These pointers are only present here, and the memory was allocated by
            // Detour, so freeing is safe.
            unsafe {
                dtFree(nav_data as _);
                dtFreeNavMesh(mesh.as_ptr());
            }
            return Err(Error::CreateQueryError(
                "Could not init Detour NavMesh".to_owned(),
            ));
        }

        // SAFETY: dtAllocNavMeshQuery either allocates its own memory or returns null, which is
        // handled by NonNull.
        let mut q = match ptr::NonNull::new(unsafe { dtAllocNavMeshQuery() }) {
            Some(q) => q,
            None => {
                return Err(Error::CreateQueryError(
                    "Could not allocate dtNavMesh".to_owned(),
                ))
            }
        };
        // SAFETY: We can turn the mesh into a mutable borrow, since only this scope has the
        // pointer, so it is exclusive. `init` only mutates `q` and otherwise just reads from
        // `mesh`.
        let status = unsafe { q.as_mut().init(mesh.as_ptr(), 1000) };
        if (status & DT_FAILURE) != 0 {
            // SAFETY: These pointers are only present here, and the memory was allocated by
            // Detour, so freeing is safe.
            unsafe {
                dtFreeNavMeshQuery(q.as_ptr());
                dtFreeNavMesh(mesh.as_ptr());
            }
            return Err(Error::CreateQueryError(
                "Could not init Detour NavMesh".to_owned(),
            ));
        }

        Ok(RecastQuery { mesh, q })
    }

    pub fn find_path(&self, start: Point, end: Point, r: f32) -> Result<Vec<Point>> {
        let (start_p, start_poly) = self.find_poly(start, r)?;
        let (end_p, end_poly) = self.find_poly(end, r)?;

        if start_poly == end_poly {
            return Ok(vec![end_p]);
        }

        let filter = dtQueryFilter {
            m_areaCost: [1.0; 64],
            m_includeFlags: POLYFLAGS_WALK,
            m_excludeFlags: 0,
        };

        let mut path = vec![0; 100];
        let mut count = 0;

        let status = unsafe {
            (*self.q.as_ptr()).findPath(
                start_poly,
                end_poly,
                start_p.0.as_ptr(),
                end_p.0.as_ptr(),
                &filter,
                path.as_mut_ptr(),
                &mut count,
                100,
            )
        };
        path.truncate(count as usize);

        if status & DT_FAILURE != 0 {
            Err(Error::FindPathError("FAIL_TO_FIND_PATH".to_owned()))
        } else if status & DT_INVALID_PARAM != 0 {
            Err(Error::FindPathError("INVALID_PARAM".to_owned()))
        } else if status & DT_BUFFER_TOO_SMALL != 0 {
            Err(Error::FindPathError("BUFFER_TOO_SMALL".to_owned()))
        } else if status & DT_OUT_OF_NODES != 0 {
            Err(Error::FindPathError("OUT_OF_NODES".to_owned()))
        } else if status & DT_PARTIAL_RESULT != 0 {
            Err(Error::PartialResult)
        } else if path.len() == 0 {
            Err(Error::FindPathError("NoPath".to_owned()))
        } else {
            if path.len() == 1 {
                // Same Poly, so just return the next point
                Ok(vec![end_p])
            } else {
                // remap the poly and the points
                let mut res = vec![];
                let mut p = start_p;
                for &poly in path.iter() {
                    p = self.find_closest(p, poly)?;
                    res.push(p);
                }

                Ok(res)
            }
        }
    }

    fn find_closest(&self, pos: Point, target_poly: u32) -> Result<Point> {
        let mut closest = [0.0; 3];
        let status = unsafe {
            (*self.q.as_ptr()).closestPointOnPoly(
                target_poly,
                pos.0.as_ptr(),
                closest.as_mut_ptr(),
                std::ptr::null_mut(),
            )
        };

        if status & DT_INVALID_PARAM != 0 {
            return Err(Error::FindPointError(
                "Fail to find path: reason: [Invalid Param]".to_owned(),
            ));
        }
        if status & DT_FAILURE != 0 {
            return Err(Error::FindPointError(
                "Fail to find closest point: reason[unknown]".to_owned(),
            ));
        }

        Ok(Point(closest))
    }

    pub fn find_poly(&self, pos: Point, r: f32) -> Result<(Point, u32)> {
        let filter = dtQueryFilter {
            m_areaCost: [1.0; 64],
            m_includeFlags: POLYFLAGS_WALK,
            m_excludeFlags: 0,
        };
        let mut result_poly = 0;
        let mut result_pos = [0.0; 3];
        // SAFETY: Dereferencing is safe for Rust since Self only stores pointers.
        // `findNearestPoly` only mutates `result_poly` and `result_pos`.
        let status = unsafe {
            (*self.q.as_ptr()).findNearestPoly(
                pos.0.as_ptr(),
                [r, r, r].as_ptr(),
                &filter,
                &mut result_poly,
                result_pos.as_mut_ptr(),
            )
        };

        if status & DT_FAILURE != 0 {
            return Err(Error::FindPointError("FAIL_TO_FIND_PATH".to_owned()));
        }
        if status & DT_INVALID_PARAM != 0 {
            return Err(Error::FindPointError("INVALID_PARAM".to_owned()));
        }
        if status & DT_BUFFER_TOO_SMALL != 0 {
            return Err(Error::FindPointError("BUFFER_TOO_SMALL".to_owned()));
        }
        if status & DT_OUT_OF_NODES != 0 {
            return Err(Error::FindPointError("OUT_OF_NODES".to_owned()));
        }
        if status & DT_PARTIAL_RESULT != 0 {
            return Err(Error::FindPointError("PARTIAL_RESULT".to_owned()));
        }

        if result_poly == 0 {
            Err(Error::FindPointError("No poly found".to_owned()))
        } else {
            Ok((Point(result_pos), result_poly))
        }
    }
}

pub fn version() -> String {
    "0.0.1".to_owned()
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn smoke_test() {
        assert_eq!("0.0.1", version());
        let mesh = simple_mesh();

        let q = RecastQuery::new_from_mesh(mesh).unwrap();
        drop(q);
    }

    #[test]
    fn test_compute_bb() {
        let data = &[-1.0, 1.0, -1.0, 1.0, 2.0, 2.0, 2.0, -2.0, 1.0];

        let (bmin, bmax) = compute_bb(data);

        assert_eq!(bmin[0], -1.0);
        assert_eq!(bmin[1], -2.0);
        assert_eq!(bmin[2], -1.0);

        assert_eq!(bmax[0], 2.0);
        assert_eq!(bmax[1], 2.0);
        assert_eq!(bmax[2], 2.0);
    }

    #[test]
    fn test_simple_path() {
        assert_eq!("0.0.1", version());
        let mesh = simple_mesh();

        let q = RecastQuery::new_from_mesh(mesh).unwrap();
        let p = q
            .find_path((0.2, 0.1, 0.4).into(), (0.8, 0.1, 0.5).into(), 0.2)
            .unwrap();

        assert_eq!(
            p,
            [Point([0.2, 0.0, 0.4]), Point([0.29999924, 0.0, 0.29999924])]
        );
    }
}
