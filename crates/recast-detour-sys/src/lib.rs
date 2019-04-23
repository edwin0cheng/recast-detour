use std::borrow;
use std::fmt;
use std::os::raw::{c_char, c_void};

#[repr(C)]
pub struct RecastNavMeshData {
    pub verts: *const u16,
    pub vert_count: u32,
    pub indices: *const u16,
    pub triangles_count: u32,
    pub bmin: [f32; 3],
    pub bmax: [f32; 3],

    pub walkable_height: f32,
    pub walkable_radius: f32,
    pub walkable_climb: f32,
    pub cell_size: f32,
    pub cell_height: f32,
}

#[derive(Debug)]
#[repr(C)]
pub struct RecastNearestPoint {
    pub center: [f32; 3],
    pub half_extents: [f32; 3],
}

#[derive(Default, Debug)]
#[repr(C)]
pub struct RecastNearestPointResult {
    pub point: [f32; 3],
    pub poly: u32,
}

#[repr(C)]
pub struct RecastNavError {
    pub msg: [u8; 256],
}

impl RecastNavError {
    pub fn zeros() -> RecastNavError {
        RecastNavError { msg: [0; 256] }
    }
}

impl RecastNavError {
    pub fn msg(&self) -> borrow::Cow<str> {
        match std::str::from_utf8(&self.msg) {
            Ok(s) => s.into(),
            Err(err) => format!("fail to decode error from utf8 : reason : {:?}", err).into(),
        }
    }
}

impl fmt::Debug for RecastNavError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "RecastNavError {{ msg: {} }}", self.msg())
    }
}

#[link(name = "RecastC", kind = "static")]
extern "C" {
    pub fn recastc_version() -> *const c_char;

    pub fn recastc_create_query(
        qparam: *const RecastNavMeshData,
        error: *mut RecastNavError,
    ) -> *const c_void;

    /// Return 0 if fail
    pub fn recastc_find_nearest_point(
        query: *const c_void,
        input: *const RecastNearestPoint,
        result: *mut RecastNearestPointResult,
        error: *mut RecastNavError,
    ) -> i32;

    pub fn recastc_free_query(query: *const c_void);
}

#[cfg(test)]

mod tests {
    use super::*;
    use insta::assert_debug_snapshot_matches;
    use std::ffi::CStr;
    use std::ptr;

    fn setup_query(verts: &[u16], indices: &[u16]) -> ptr::NonNull<c_void> {
        let version = unsafe { recastc_version() };
        assert_ne!(version, ptr::null());
        let version = unsafe { CStr::from_ptr(version).to_str().unwrap() };

        assert_debug_snapshot_matches!(version, @"\"0.0.1\"");
        assert_eq!(verts.len() % 3, 0);

        let cell_size = 0.1;
        let cell_height = 0.1;
        let mut bmin = [std::f32::MAX; 3];
        let mut bmax = [std::f32::MIN; 3];

        for &i in indices {
            let idx = i as usize * 3;
            bmin[0] = ((verts[idx + 0] as f32) * cell_size).min(bmin[0]);
            bmin[1] = ((verts[idx + 1] as f32) * cell_height).min(bmin[1]);
            bmin[2] = ((verts[idx + 2] as f32) * cell_size).min(bmin[2]);

            bmax[0] = ((verts[idx + 0] as f32) * cell_size).max(bmax[0]);
            bmax[1] = ((verts[idx + 1] as f32) * cell_height).max(bmax[1]);
            bmax[2] = ((verts[idx + 2] as f32) * cell_size).max(bmax[2]);
        }

        let data = RecastNavMeshData {
            verts: &verts[0] as *const _,
            vert_count: (verts.len() as u32) / 3,
            indices: &indices[0] as *const _,
            triangles_count: (indices.len() as u32) / 3,
            bmin: bmin,
            bmax: bmax,
            walkable_height: 0.1,
            walkable_radius: 0.1,
            walkable_climb: 0.1,
            cell_size,
            cell_height,
        };

        let mut err = RecastNavError::zeros();

        let q = unsafe {
            ptr::NonNull::new(
                recastc_create_query(&data as *const _, &mut err as *mut _) as *mut c_void
            )
        };

        q.unwrap_or_else(|| {
            panic!("Failed on recastc_create_query, reason : {}", err.msg());
        })
    }

    #[test]
    fn test_smoke_test() {
        let verts: &[u16] = &[0, 0, 0, 10, 0, 0, 10, 0, 10, 0, 0, 10];
        let indices: &[u16] = &[0, 1, 2, 0, 2, 3];

        let q = setup_query(verts, indices);

        unsafe {
            recastc_free_query(q.as_ptr());
        }
    }

    #[test]
    fn test_find_nearest_point() {
        let verts: &[u16] = &[0, 0, 0, 10, 0, 0, 10, 0, 10, 0, 0, 10];
        let indices: &[u16] = &[0, 1, 2, 0, 2, 3];

        let q = setup_query(verts, indices);

        let input = RecastNearestPoint {
            center: [0.2, 0.1, 0.5],
            half_extents: [0.2, 0.2, 0.2],
        };

        let mut result = RecastNearestPointResult::default();
        let mut err = RecastNavError::zeros();
        let r = unsafe {
            recastc_find_nearest_point(
                q.as_ptr(),
                &input as *const _,
                &mut result as *mut _,
                &mut err as *mut _,
            )
        };

        unsafe {
            recastc_free_query(q.as_ptr());
        }

        assert!(
            r != 0,
            "Failed on recastc_create_query, reason : {}",
            err.msg()
        );

        assert_debug_snapshot_matches!(result, @r###"RecastNearestPointResult {
    point: [
        0.2,
        0.0,
        0.5
    ],
    poly: 3
}"###);
    }
}
