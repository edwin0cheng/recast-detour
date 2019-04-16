fn main() {    
	#[cfg(not(feature = "skip-build-recast"))]
	{
    	let dst = cmake::build("recast");
    	println!("cargo:rustc-link-search=native={}/lib", dst.display());
    	println!("cargo:rustc-link-lib=static=RecastC");
    }
}