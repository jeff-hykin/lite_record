//! Link flags for the two SDKs that have no crate of their own.
//!
//! librealsense comes through `realsense-sys`, which brings its own build
//! script. The Orbbec SDK is shipped as a zip rather than packaged by any
//! distribution, so `ORBBEC_SDK_DIR` is the normal way to point at it and
//! pkg-config is only a fallback. depthai is C++ with no C API at all, so on top
//! of the link flags this also compiles the shim that gives it one.

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    orbbec();
    oakd();
}

fn orbbec() {
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

/// Compiles `oakd_shim.cpp` and links depthai-core behind it.
///
/// depthai's public headers include several of its own dependencies' headers —
/// nlohmann/json and libnop among them — so a single SDK directory is not
/// enough to compile against. `DEPTHAI_INCLUDE_DIRS` carries those, colon
/// separated, because only whoever assembled the SDK knows where they ended up.
#[cfg(feature = "oakd")]
fn oakd() {
    println!("cargo:rerun-if-changed=src/sensors/oakd_shim.cpp");
    println!("cargo:rerun-if-env-changed=DEPTHAI_DIR");
    println!("cargo:rerun-if-env-changed=DEPTHAI_INCLUDE_DIRS");

    let mut build = cc::Build::new();
    build
        .cpp(true)
        .std("c++17")
        .file("src/sensors/oakd_shim.cpp");

    if let Some(sdk) = std::env::var_os("DEPTHAI_DIR") {
        let root = std::path::PathBuf::from(&sdk);
        build.include(root.join("include"));
        let library_directory = root.join("lib");
        let displayed = library_directory.display();
        println!("cargo:rustc-link-search=native={displayed}");
        println!("cargo:rustc-link-arg=-Wl,-rpath,{displayed}");
    }
    for directory in std::env::var("DEPTHAI_INCLUDE_DIRS")
        .unwrap_or_default()
        .split(':')
        .filter(|directory| !directory.is_empty())
    {
        build.include(directory);
    }

    build.compile("lr_oak_shim");
    println!("cargo:rustc-link-lib=dylib=depthai-core");
}

#[cfg(not(feature = "oakd"))]
fn oakd() {}
