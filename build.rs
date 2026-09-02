//! Link flags for the Orbbec SDK.
//!
//! librealsense comes through `realsense-sys`, which brings its own build
//! script. The Orbbec SDK has no crate, so its search path has to be worked out
//! here. It is shipped as a zip rather than packaged by any distribution, so
//! `ORBBEC_SDK_DIR` is the normal way to point at it and pkg-config is only a
//! fallback for the few distributions that do package it.

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=ORBBEC_SDK_DIR");

    if std::env::var_os("CARGO_FEATURE_ORBBEC").is_none() {
        return;
    }

    let Some(sdk) = std::env::var_os("ORBBEC_SDK_DIR") else {
        // Left to the linker's own search path, which is right when the SDK was
        // installed system-wide, and produces a clear "library not found" when
        // it was not.
        println!("cargo:rustc-link-lib=dylib=OrbbecSDK");
        return;
    };

    let library_directory = std::path::Path::new(&sdk).join("lib");
    let library_directory = if library_directory.is_dir() {
        library_directory
    } else {
        std::path::PathBuf::from(&sdk)
    };
    let displayed = library_directory.display();

    println!("cargo:rustc-link-search=native={displayed}");
    println!("cargo:rustc-link-lib=dylib=OrbbecSDK");
    // The SDK is only distributed as a shared library, so the binary has to
    // carry the path it was linked against or it will not start off the
    // developer's machine.
    println!("cargo:rustc-link-arg=-Wl,-rpath,{displayed}");
}
