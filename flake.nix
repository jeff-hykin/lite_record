{
    description = "lite_record - one binary that records a RealSense / Orbbec / Mid-360 rig to an mcap, with a browser UI";

    inputs = {
        nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
        rust-overlay.url = "github:oxalica/rust-overlay";
        rust-overlay.inputs.nixpkgs.follows = "nixpkgs";
    };

    outputs = { self, nixpkgs, rust-overlay }:
        let
            systems = [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
            forEachSystem = function: nixpkgs.lib.genAttrs systems (system: function system);
        in
        {
            packages = forEachSystem (system:
                let
                    pkgs = import nixpkgs { inherit system; overlays = [ (import rust-overlay) ]; };

                    # One pinned toolchain for every build here, carrying both musl
                    # targets so the cross builds do not need a second rustc.
                    rustToolchain = pkgs.rust-bin.stable.latest.default.override {
                        targets = [
                            "x86_64-unknown-linux-musl"
                            "aarch64-unknown-linux-musl"
                            "aarch64-unknown-linux-gnu"
                        ];
                    };
                    rustPlatform = pkgs.makeRustPlatform {
                        cargo = rustToolchain;
                        rustc = rustToolchain;
                    };

                    # Imported only for their musl cross toolchains, not to build anything.
                    crossPkgsFor = config: import nixpkgs { inherit system; crossSystem.config = config; };

                    # Listed rather than taken as `./.`, because a plain path copies
                    # the multi-gigabyte `target/` into the store on every build and
                    # a git-filtered path silently drops anything not yet committed.
                    # Both `tests/` and `web/` are here because `src/` reaches into
                    # them with `include_bytes!` / `include_str!`.
                    source = pkgs.lib.fileset.toSource {
                        root = ./.;
                        fileset = pkgs.lib.fileset.unions [
                            ./Cargo.toml
                            ./Cargo.lock
                            ./build.rs
                            ./src
                            ./tests
                            ./web
                            ./examples
                            ./vendor
                        ];
                    };

                    commonArgs = {
                        pname = "lite_record";
                        version = "0.1.0";
                        src = source;
                        cargoLock.lockFile = ./Cargo.lock;
                    };

                    native = rustPlatform.buildRustPackage commonArgs;

                    buildCross = { rustTarget, crossPkgs, features ? [ ], sdkLibraries ? [ ] }:
                        let
                            targetSnake = builtins.replaceStrings [ "-" ] [ "_" ] rustTarget;
                            targetUpper = pkgs.lib.toUpper targetSnake;
                            binDirectory = "${crossPkgs.stdenv.cc}/bin";
                            prefix = crossPkgs.stdenv.cc.targetPrefix;
                            featureFlag = pkgs.lib.optionalString (features != [ ])
                                "--features ${pkgs.lib.concatStringsSep "," features}";

                            # `-sys` crates find their SDK through pkg-config, which refuses a
                            # cross build unless told the .pc files really do describe the
                            # target. Exported here rather than as a derivation attribute
                            # because stdenv already defines this one and would collide.
                            # Nothing compiles against the SDK's headers -- realsense-sys ships
                            # pre-generated bindings -- so only the .so and its .pc matter.
                            allowCross = pkgs.lib.optionalString (sdkLibraries != [ ])
                                "export PKG_CONFIG_ALLOW_CROSS=1";

                            # Works around an upstream bug: jpegxl-src 0.12.0 picks the
                            # C++ runtime to link with `cfg!(target_vendor = "apple")`,
                            # which a build script evaluates against the machine doing
                            # the building, not the machine being built for. Building on
                            # a Mac therefore asks every cross target for clang's
                            # `libc++`, which a gcc toolchain does not ship. `ld` reads
                            # any file it finds as a linker script, so this hands it the
                            # runtime the toolchain actually has. Both names are needed:
                            # the musl targets link `-Bstatic` and so look for the `.a`.
                            cxxRuntimeShim = pkgs.runCommand "libcxx-shim" { } ''
                                mkdir -p $out/lib
                                echo 'INPUT(-lstdc++)' > $out/lib/libc++.so
                                echo 'INPUT(-lstdc++)' > $out/lib/libc++.a
                            '';
                        in
                        rustPlatform.buildRustPackage (commonArgs // {
                            pname = "lite_record-${rustTarget}";

                            # The test binary is built for the foreign target and cannot run here.
                            doCheck = false;

                            buildPhase = ''
                                runHook preBuild
                                ${allowCross}
                                cargo build --release --target ${rustTarget} ${featureFlag}
                                runHook postBuild
                            '';

                            installPhase = ''
                                runHook preInstall
                                mkdir -p $out/bin
                                install -m755 target/${rustTarget}/release/lite_record $out/bin/lite_record
                                runHook postInstall
                            '';

                            # gamut-jxl-sys cmake-builds a vendored libjxl. `dontUseCmakeConfigure`
                            # keeps cmake's setup hook from replacing the configure phase that
                            # buildRustPackage needs to vendor the registry.
                            nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
                            dontUseCmakeConfigure = true;

                            "CARGO_TARGET_${targetUpper}_LINKER" = "${binDirectory}/${prefix}cc";

                            # zstd-sys, lz4-sys and ring compile C from build scripts. Without
                            # these cc-rs reaches for the host clang with --target=<triple>,
                            # which has no matching sysroot and dies on `#include <string.h>`.
                            "CC_${targetSnake}" = "${binDirectory}/${prefix}cc";
                            "CXX_${targetSnake}" = "${binDirectory}/${prefix}c++";
                            "AR_${targetSnake}" = "${binDirectory}/${prefix}ar";

                            PKG_CONFIG_PATH = pkgs.lib.concatMapStringsSep ":"
                                (library: "${pkgs.lib.getDev library}/lib/pkgconfig") sdkLibraries;

                            # Two separate jobs for the sdk libraries here.
                            #
                            # `-L native=` is a workaround for an upstream bug:
                            # librealsense ships a realsense2.pc that hardcodes
                            # `libdir=''${prefix}/lib/x86_64-linux-gnu` -- there is a
                            # literal `#TODO` above that line -- no matter which
                            # architecture it was built for. So pkg-config sends the
                            # linker to an x86 path inside the aarch64 package and
                            # `-lrealsense2` is not found. The true directory is passed
                            # explicitly rather than trusting the .pc.
                            #
                            # The rpath is unrelated: the SDK is a dynamic .so, so the
                            # binary must carry its store path to start on the target.
                            # Deploy with `nix copy --to ssh://<host>` to bring it along.
                            "CARGO_TARGET_${targetUpper}_RUSTFLAGS" =
                                pkgs.lib.concatStringsSep " "
                                    ([ "-L native=${cxxRuntimeShim}/lib" ]
                                        ++ map
                                        (library:
                                            "-L native=${pkgs.lib.getLib library}/lib"
                                            + " -C link-arg=-Wl,-rpath,${pkgs.lib.getLib library}/lib")
                                        sdkLibraries);
                        });

                    # The OAK-D's SDK. Not in nixpkgs, and its own build system
                    # wants to bootstrap vcpkg and reach the network for both its
                    # dependencies and the camera's firmware, neither of which a
                    # nix build may do. Every dependency is taken from nixpkgs
                    # instead, and the two firmware blobs are fetched as fixed
                    # output derivations and dropped where CMake looks before it
                    # downloads.
                    depthaiCoreFor = targetPkgs:
                        let
                            # From cmake/Depthai/DepthaiDeviceSideConfig.cmake and
                            # DepthaiBootloaderConfig.cmake of the tag below. These
                            # decide the artifact URLs, so they move with the version.
                            deviceCommit = "40f5e0b83ee8148d5871e08a74521f05ee0760cd";
                            bootloaderVersion = "0.0.29";
                            artifactory = "https://artifacts.luxonis.com/artifactory";

                            deviceFirmware = pkgs.fetchurl {
                                url = "${artifactory}/luxonis-myriad-snapshot-local/depthai-device-side/${deviceCommit}/depthai-device-fwp-${deviceCommit}.tar.xz";
                                hash = "sha256-528xYKD4M/rBWLPpEgsFYJHGbvCBrupRrMnIUYzck/k=";
                            };
                            bootloaderFirmware = pkgs.fetchurl {
                                url = "${artifactory}/luxonis-myriad-release-local/depthai-bootloader/${bootloaderVersion}/depthai-bootloader-fwp-${bootloaderVersion}.tar.xz";
                                hash = "sha256-LngIu0kXw0BafMaR6U86Djj5ICX4nMmArsXdFIirLcM=";
                            };

                            # depthai-core always builds XLink itself rather than
                            # looking for an installed one, so it needs the source.
                            # Pinned to the commit depthai's own FetchContent names
                            # rather than to the nixpkgs package, which sits on a
                            # different commit.
                            xlinkSource = pkgs.fetchFromGitHub {
                                owner = "luxonis";
                                repo = "XLink";
                                rev = "05de4c0ee77d07c44ec92536625e6d96d866a34a";
                                hash = "sha256-Q8lx6g9lKyuow8QUJFMIHPqEGHa2UoxI5Drk/6n3k6U=";
                            };

                            # nixpkgs' libnop compiles a gtest suite its Makefile
                            # builds by default, which no cross build can link. The
                            # library itself is header only and ships a CMake target
                            # of its own, so depthai is pointed at the source and
                            # left to add_subdirectory it.
                            libnopSource = pkgs.fetchFromGitHub {
                                owner = "luxonis";
                                repo = "libnop";
                                rev = "ab842f51dc2eb13916dc98417c2186b78320ed10";
                                hash = "sha256-d2z/lDI9pe5TR82MxGkR9bBMNXPvzqb9Gsd5jOv6x1A=";
                            };

                            # depthai asks for liblzma as a CMake config package and
                            # nixpkgs' xz ships only a pkg-config file, so the target
                            # it links against is declared by hand.
                            liblzmaShim = pkgs.writeTextDir "liblzmaConfig.cmake" ''
                                add_library(liblzma::liblzma UNKNOWN IMPORTED)
                                set_target_properties(liblzma::liblzma PROPERTIES
                                    IMPORTED_LOCATION "${targetPkgs.lib.getLib targetPkgs.xz}/lib/liblzma${targetPkgs.stdenv.hostPlatform.extensions.sharedLibrary}"
                                    INTERFACE_INCLUDE_DIRECTORIES "${targetPkgs.lib.getDev targetPkgs.xz}/include")
                            '';
                        in
                        targetPkgs.stdenv.mkDerivation {
                            pname = "depthai-core";
                            version = "3.10.0";

                            src = pkgs.fetchFromGitHub {
                                owner = "luxonis";
                                repo = "depthai-core";
                                rev = "v3.10.0";
                                fetchSubmodules = true;
                                hash = "sha256-3ykQ96rnbr3bDQfFS4DHTD2uWxS0CkCNTEU+Qp0UaV4=";
                            };

                            nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];

                            buildInputs = with targetPkgs; [
                                bzip2
                                eigen
                                fp16
                                httplib
                                libarchive
                                libusb1
                                lz4
                                magic-enum
                                neargye-semver
                                nlohmann_json
                                # Not depthai's own dependency: nixpkgs' httplib
                                # config asks for it, and depthai asks for httplib.
                                openssl
                                spdlog
                                xtensor
                                xtl
                                xz
                                yaml-cpp
                                zlib
                            ];

                            # The firmware CMake would otherwise go to the network
                            # for. DownloadAndChecksum() returns early when the
                            # output file already exists, so a copy under the build
                            # directory's `resources` is taken as an already
                            # completed download. The device licence is verified
                            # against the copy committed in the tree, so that same
                            # file is what gets put in place.
                            postPatch = ''
                                mkdir -p build/resources
                                cp ${deviceFirmware} build/resources/depthai-device-fwp-${deviceCommit}.tar.xz
                                cp notices/depthai-device-RVC2-LICENSE build/resources/depthai-device-fwp-${deviceCommit}-LICENSE
                                cp ${bootloaderFirmware} build/resources/depthai-bootloader-fwp-${bootloaderVersion}.tar.xz
                                chmod +w build/resources/*

                                # depthai spells the target the way vcpkg exports
                                # it; every other dependency's nixpkgs name already
                                # matches, so this is the one rename needed.
                                substituteInPlace CMakeLists.txt \
                                    --replace-fail "lz4::lz4" "LZ4::lz4"

                                # Clamping a float array with integer literals
                                # picks Eigen's expression overload rather than
                                # its scalar one, which then has no `.max`. The
                                # bounds are exactly +/-1 either way.
                                substituteInPlace src/utility/ObjectTrackerImpl.cpp \
                                    --replace-fail "array().min(1).max(-1)" "array().min(1.0f).max(-1.0f)"
                            '';

                            cmakeFlags = [
                                "-Dliblzma_DIR=${liblzmaShim}"
                                "-DDEPTHAI_XLINK_LOCAL=${xlinkSource}"
                                # XLink's own libusb lookup, which uses find_library
                                # rather than a CMake config package nixpkgs has none of.
                                "-DXLINK_LIBUSB_SYSTEM=ON"

                                # Take every dependency from nixpkgs.
                                "-DDEPTHAI_BOOTSTRAP_VCPKG=OFF"
                                "-DDEPTHAI_VCPKG_INTERNAL_ONLY=OFF"

                                # libnop is the one interface library nixpkgs cannot
                                # supply, so depthai adds its subdirectory instead.
                                # Naming the source directory is what stops
                                # FetchContent reaching for git.
                                "-DDEPTHAI_LIBNOP_EXTERNAL=OFF"
                                "-DFETCHCONTENT_SOURCE_DIR_LIBNOP=${libnopSource}"
                                "-DFETCHCONTENT_FULLY_DISCONNECTED=ON"
                                # libnop's own CMakeLists asks for 3.2, below what
                                # current CMake will accept without being told.
                                "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"

                                # A headless recorder decodes nothing and shows
                                # nothing, and each of these drags in a library tree
                                # far larger than depthai itself.
                                "-DDEPTHAI_OPENCV_SUPPORT=OFF"
                                "-DDEPTHAI_PCL_SUPPORT=OFF"
                                "-DDEPTHAI_ENABLE_PROTOBUF=OFF"
                                "-DDEPTHAI_ENABLE_REMOTE_CONNECTION=OFF"
                                "-DDEPTHAI_ENABLE_CURL=OFF"
                                "-DDEPTHAI_ENABLE_MP4V2=OFF"
                                "-DDEPTHAI_ENABLE_APRIL_TAG=OFF"
                                "-DDEPTHAI_ENABLE_BACKWARD=OFF"

                                # Downloads a blob keyed by the *build* machine's
                                # architecture, which is wrong for every cross build
                                # and unavailable offline in any case.
                                "-DDEPTHAI_DYNAMIC_CALIBRATION_SUPPORT=OFF"

                                # The OAK-D Pro is a MyriadX, so only the RVC2
                                # firmware is ever loaded onto it.
                                "-DDEPTHAI_ENABLE_DEVICE_RVC4_FW=OFF"

                                "-DDEPTHAI_BUILD_EXAMPLES=OFF"
                                "-DDEPTHAI_BUILD_TESTS=OFF"
                                "-DDEPTHAI_CLANG_FORMAT=OFF"
                                "-DBUILD_SHARED_LIBS=ON"
                            ];

                            meta = {
                                description = "Luxonis DepthAI C++ library for OAK cameras";
                                homepage = "https://github.com/luxonis/depthai-core";
                                license = pkgs.lib.licenses.mit;
                            };
                        };

                    # librealsense installs udev helper scripts wrapped with
                    # v4l-utils on their PATH, and v4l-utils builds `qv4l2` and
                    # `qvidcap` by default, which drags Qt6 into the closure.
                    # Cross-compiling Qt6 to aarch64 takes hours and nothing in a
                    # headless recorder ever opens a window, so the GUI is turned
                    # off. This is the difference between a cross build that
                    # finishes in minutes and one that does not finish at all.
                    aarch64Gnu = (crossPkgsFor "aarch64-unknown-linux-gnu").extend
                        (final: previous: {
                            v4l-utils = previous.v4l-utils.override { withGUI = false; };

                            # libusb's udev support pulls in systemd, and
                            # cross-building systemd from darwin dies inside
                            # sqlite's tcl, which wants mach headers the target
                            # does not have. Nothing here needs udev: both XLink
                            # and librealsense's RSUSB backend open the device
                            # through libusb itself, and libusb keeps hotplug on
                            # Linux through netlink either way.
                            libusb1 = previous.libusb1.override { enableUdev = false; };

                            # httplib renamed MultipartFormDataItems to
                            # UploadFormDataItems in 0.20, and DeviceGate.cpp
                            # still uses the old name. nixpkgs is on 0.30.2, so
                            # the last release that kept it is pinned here.
                            httplib = previous.httplib.overrideAttrs (old: rec {
                                version = "0.18.7";
                                src = pkgs.fetchFromGitHub {
                                    owner = "yhirose";
                                    repo = "cpp-httplib";
                                    tag = "v${version}";
                                    hash = "sha256-DkET7D2hF6xlrYWEGC87rFqEe1JjMS3SHX6QFSi1oQg=";
                                };
                            });

                            # depthai's Version.cpp uses `semver::version` as a
                            # plain class and `semver::prerelease`, both of which
                            # semver 1.x removed, and it passes the pre-release
                            # number as an optional, which only 0.3.1 accepts.
                            # nixpkgs is on 1.0.1, so 0.3.1 is pinned here.
                            neargye-semver = previous.neargye-semver.overrideAttrs
                                (old: rec {
                                    version = "0.3.1";
                                    src = pkgs.fetchFromGitHub {
                                        owner = "Neargye";
                                        repo = "semver";
                                        tag = "v${version}";
                                        hash = "sha256-0HOp+xzo8xcCUUgtSh87N9DXP5P0odBaYXhcDzOiiXE=";
                                    };
                                    # It vendors a Catch2 that current gcc will
                                    # not compile, and the library itself is
                                    # header-only, so nothing installed depends on
                                    # the tests being built.
                                    cmakeFlags = [
                                        "-DSEMVER_OPT_BUILD_TESTS=OFF"
                                        "-DSEMVER_OPT_BUILD_EXAMPLES=OFF"
                                        "-DSEMVER_OPT_INSTALL=ON"
                                    ];
                                    doCheck = false;
                                });

                            # Both of these turn their test suites on
                            # unconditionally but only pull doctest in when the
                            # tests are going to be run, which a cross build never
                            # does. Configuring then fails looking for doctest. Both
                            # are header-only, so dropping the tests costs the
                            # installed headers nothing.
                            xtl = previous.xtl.overrideAttrs (old: {
                                cmakeFlags = [ "-DBUILD_TESTS=OFF" ];
                            });
                            xtensor = previous.xtensor.overrideAttrs (old: {
                                cmakeFlags = (builtins.filter
                                    (flag: flag != "-DBUILD_TESTS=ON")
                                    (old.cmakeFlags or [ ]))
                                    ++ [ "-DBUILD_TESTS=OFF" ];
                            });

                            # By default librealsense reaches the D4xx IMU through the
                            # kernel's HID-to-IIO bridge, which needs hid-sensor-hub,
                            # hid-sensor-accel-3d and hid-sensor-gyro-3d. Raspberry Pi
                            # OS ships none of them, so the motion module is invisible
                            # and every request naming it fails to resolve. The RSUSB
                            # backend talks to the same hardware over libusb instead
                            # and needs no kernel support at all.
                            #
                            # Dropping v4l-utils follows from that. It is not a
                            # buildInput at all -- nixpkgs only puts it on the PATH
                            # of `rs-enum.sh` and `rs_ipu6_d457_bind.sh`, two V4L2
                            # enumeration helpers that an RSUSB build never calls.
                            # It still costs the whole systemd-minimal-libs and
                            # libbpf chain, which does not cross-compile from
                            # darwin, and that alone is what made this the one
                            # target a Mac could not build.
                            librealsense = (previous.librealsense.override {
                                v4l-utils = previous.emptyDirectory;
                            }).overrideAttrs (old: {
                                cmakeFlags = (old.cmakeFlags or [ ])
                                    ++ [ "-DFORCE_RSUSB_BACKEND=ON" ];
                            });
                        });

                    # depthai ships no .pc file, only a CMake config, so it is
                    # found through `DEPTHAI_DIR` rather than the `sdkLibraries`
                    # pkg-config path every other SDK here uses.
                    withDepthai = package: package.overrideAttrs (old: {
                        DEPTHAI_DIR = depthaiCoreFor aarch64Gnu;

                        # depthai's public headers include their dependencies'
                        # headers directly, and cc-rs is invoked by a build script
                        # that stdenv adds no target include paths to, so each one
                        # has to be named.
                        DEPTHAI_INCLUDE_DIRS = pkgs.lib.concatMapStringsSep ":"
                            (library: "${pkgs.lib.getDev library}/include")
                            (with aarch64Gnu; [ nlohmann_json spdlog fmt xtensor xtl ]);
                    });
                in
                {
                    lite_record = native;
                    default = native;
                    linux-x86 = buildCross {
                        rustTarget = "x86_64-unknown-linux-musl";
                        crossPkgs = crossPkgsFor "x86_64-unknown-linux-musl";
                    };
                    # Every camera in one binary, so unplugging a sensor is not a
                    # rebuild. A backend that finds no hardware just reports
                    # itself disengaged, which costs nothing at runtime.
                    linux-arm64 = withDepthai (buildCross {
                        rustTarget = "aarch64-unknown-linux-gnu";
                        crossPkgs = aarch64Gnu;
                        features = [ "realsense" "oakd" "livox" ];
                        sdkLibraries = [ aarch64Gnu.librealsense ];
                    });
                    # The encode-cost bench as its own aarch64 binary, so the codec
                    # decision can be measured on the Pi without putting a rust
                    # toolchain there. It only touches `image` and `msgs`, so it does
                    # not need the realsense feature or its SDK.
                    linux-arm64-bench = (buildCross {
                        rustTarget = "aarch64-unknown-linux-musl";
                        crossPkgs = crossPkgsFor "aarch64-unknown-linux-musl";
                    }).overrideAttrs (old: {
                        pname = "lite_record-bench-aarch64-unknown-linux-musl";
                        buildPhase = ''
                            runHook preBuild
                            cargo build --release --target aarch64-unknown-linux-musl --example encode_cost
                            runHook postBuild
                        '';
                        installPhase = ''
                            runHook preInstall
                            mkdir -p $out/bin
                            install -m755 target/aarch64-unknown-linux-musl/release/examples/encode_cost $out/bin/encode_cost
                            runHook postInstall
                        '';
                    });
                    depthai-arm64 = depthaiCoreFor aarch64Gnu;
                }
                # Only ever consumed by the Linux-only `cameras` shell, and its
                # dependency set is Linux-shaped, so it is not offered elsewhere.
                // pkgs.lib.optionalAttrs pkgs.stdenv.hostPlatform.isLinux {
                    depthai-native = depthaiCoreFor pkgs;
                });

            devShells = forEachSystem (system:
                let
                    pkgs = import nixpkgs { inherit system; overlays = [ (import rust-overlay) ]; };
                    rustToolchain = pkgs.rust-bin.stable.latest.default.override {
                        targets = [
                            "x86_64-unknown-linux-musl"
                            "aarch64-unknown-linux-musl"
                            "aarch64-unknown-linux-gnu"
                        ];
                        extensions = [ "rust-src" "rust-analyzer" ];
                    };
                    linkerFor = config:
                        let crossPkgs = import nixpkgs { inherit system; crossSystem.config = config; };
                        in "${crossPkgs.stdenv.cc}/bin/${crossPkgs.stdenv.cc.targetPrefix}cc";
                in
                {
                    default = pkgs.mkShell {
                        packages = [ rustToolchain pkgs.pkg-config ];

                        CARGO_TARGET_X86_64_UNKNOWN_LINUX_MUSL_LINKER = linkerFor "x86_64-unknown-linux-musl";
                        CARGO_TARGET_AARCH64_UNKNOWN_LINUX_MUSL_LINKER = linkerFor "aarch64-unknown-linux-musl";
                        CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER = linkerFor "aarch64-unknown-linux-gnu";

                        shellHook = ''
                            echo "lite_record dev shell (Rust $(rustc --version | cut -d' ' -f2))"
                            echo ""
                            echo "  ./run/build                        -- every target into dist/"
                            echo "  nix build .#linux-arm64            -- aarch64 gnu, every camera + livox"
                            echo "  nix build .#linux-x86              -- x86_64 musl, static, no camera SDK"
                            echo "  nix develop .#cameras              -- the SDKs, for --features realsense,oakd,livox"
                            echo ""
                        '';
                    };

                }
                # The default shell builds the featureless binary, which is what
                # most work needs and costs no SDK. The camera features only
                # compile against an SDK that is actually present, so testing them
                # takes this shell. Linux only, because librealsense declares no
                # darwin platform and asking for it there fails evaluation rather
                # than the build, which would take `nix flake check` down with it.
                // pkgs.lib.optionalAttrs pkgs.stdenv.hostPlatform.isLinux {
                    cameras = pkgs.mkShell {
                        packages = [ rustToolchain pkgs.pkg-config ];

                        PKG_CONFIG_PATH = "${pkgs.lib.getDev pkgs.librealsense}/lib/pkgconfig";
                        DEPTHAI_DIR = self.packages.${system}.depthai-native;
                        DEPTHAI_INCLUDE_DIRS = pkgs.lib.concatMapStringsSep ":"
                            (library: "${pkgs.lib.getDev library}/include")
                            (with pkgs; [ nlohmann_json spdlog fmt xtensor xtl ]);

                        # librealsense's own realsense2.pc hardcodes an
                        # x86_64-linux-gnu libdir, so the true one is named here
                        # for the same reason the cross builds have to.
                        RUSTFLAGS = "-L native=${pkgs.lib.getLib pkgs.librealsense}/lib";

                        shellHook = ''
                            echo "lite_record camera shell -- cargo test --features realsense,oakd,livox"
                        '';
                    };
                });
        };
}
