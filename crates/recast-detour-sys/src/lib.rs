use std::os::raw::c_char;

#[link(name = "RecastC", kind="static")]
extern {
    fn recastc_version() -> *const c_char;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        unsafe {
            dbg!(recastc_version());
        }
    }
}
