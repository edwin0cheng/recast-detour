fn main() {
    #[cfg(not(feature = "skip-build-recast"))]
    {
        use std::fs;
        use std::io;
        use std::path::Path;

        fn print_path(path: &Path) {
            if let Some(path) = path.to_str() {
                println!("cargo:rerun-if-changed={}", path);
            }
        }

        fn print_dirs(dir: &Path) -> io::Result<()> {
            if dir.is_dir() {
                print_path(dir);

                for entry in fs::read_dir(dir)? {
                    let entry = entry?;
                    let path = entry.path();
                    if path.is_dir() {
                        print_dirs(&path)?;
                    } else {
                        print_path(&path);
                    }
                }
            }
            Ok(())
        }

        let dst = cmake::build("recast");

        println!("cargo:rerun-if-changed={}", "build.rs");
        let _ = print_dirs(Path::new("recast"));

        println!("cargo:rustc-link-search=native={}/lib", dst.display());
        println!("cargo:rustc-link-lib=static=RecastC");
        println!("cargo:rustc-link-lib=static=Detour");
    }
}
