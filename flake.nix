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

                    # crate2nix, one derivation per crate, so editing this crate does not
                    # rebuild its dependency tree. `Cargo.nix` is generated and committed
                    # rather than produced at eval time: import-from-derivation would make
                    # every evaluation -- `nix flake show` included -- wait on a build.
                    # Regenerate after any Cargo.lock change with
                    #   nix run github:nix-community/crate2nix -- generate -f ./Cargo.toml -o Cargo.nix
                    #
                    # Cross works by handing this the CROSS pkgs rather than by exporting
                    # CC_<triple>/CXX_<triple>/AR_<triple> by hand: a cross stdenv already
                    # sets those, and its cc wrapper already contributes every buildInput's
                    # include and library paths. Same shape as ~/repos/g1_ps1.
                    crate2nixFor = { targetPkgs, crateOverrides ? { }, rootFeatures ? [ "default" ] }:
                        (import ./Cargo.nix {
                            pkgs = targetPkgs;
                            inherit rootFeatures;
                            buildRustCrateForPkgs = cratePkgs: cratePkgs.buildRustCrate.override {
                                defaultCrateOverrides = cratePkgs.defaultCrateOverrides // crateOverrides;
                            };
                        }).rootCrate.build;

                    # gamut-jxl-sys cmake-builds a vendored libjxl, and finds the source
                    # through `DEP_JXL_PATH`. Cargo would set that from jpegxl-src's `links`
                    # key; buildRustCrate names DEP_ vars after the *crate* instead, so it
                    # never arrives and the build script panics with "Source directory
                    # .../libjxl does not exist". Same defect dimos works around in
                    # dimos/mapping/dim_slam/rust/flake.nix for DEP_CUVSLAM_LIB_DIR.
                    #
                    # The hash is the one crate2nix put in Cargo.nix for jpegxl-src, so the
                    # source here and the source it compiles against cannot drift.
                    jpegxlSource = pkgs.runCommand "jpegxl-src-libjxl-0.12.0" { } ''
                        tar -xzf ${pkgs.fetchurl {
                            url = "https://static.crates.io/crates/jpegxl-src/jpegxl-src-0.12.0.crate";
                            sha256 = "02hqjr37d4sw94w8k0rnd53rii8i1b0vgc7bvqbqas3gwd8nmx86";
                        }}
                        mv jpegxl-src-0.12.0/libjxl $out
                    '';

                    commonCrateOverrides = targetPkgs: {
                        gamut-jxl-sys = attrs: {
                            nativeBuildInputs = (attrs.nativeBuildInputs or [ ]) ++ [ pkgs.cmake ];
                            DEP_JXL_PATH = "${jpegxlSource}";
                        };
                    };

                    # Upstream bug, and it survives the move to crate2nix: jpegxl-src 0.12.0
                    # picks its C++ runtime with `cfg!(target_vendor = "apple")`, which a build
                    # script evaluates against the machine doing the BUILDING, not the one being
                    # built for. So building on a Mac makes every cross target ask for clang's
                    # libc++, which a gcc toolchain does not ship, and the link dies with
                    #   cannot find -lc++
                    # right after libgamut_jxl_sys. `ld` reads any file it finds as a linker
                    # script, so this hands it the runtime the toolchain actually has. Both names
                    # are needed: the musl targets link -Bstatic and so look for the `.a`.
                    #
                    # Left out of the first crate2nix attempt on purpose, to find out whether a
                    # real cross stdenv made it unnecessary. It did not -- x86 musl failed exactly
                    # as above. Kept because a build said so, not because it was inherited.
                    cxxRuntimeShim = pkgs.runCommand "libcxx-shim" { } ''
                        mkdir -p $out/lib
                        echo 'INPUT(-lstdc++)' > $out/lib/libc++.so
                        echo 'INPUT(-lstdc++)' > $out/lib/libc++.a
                    '';

                    # Only cross targets need it; a Mac really does have libc++.
                    crossCrateOverrides = targetPkgs: (commonCrateOverrides targetPkgs) // {
                        lite_record = attrs: {
                            extraRustcOpts = (attrs.extraRustcOpts or [ ])
                                ++ [ "-L native=${cxxRuntimeShim}/lib" ];
                        };
                    };

                    # The crate2nix native build, offered alongside the old one until the
                    # cross targets are ported too -- so a regression is a comparison
                    # rather than a bisect.
                    nativeC2N = crate2nixFor {
                        targetPkgs = pkgs;
                        crateOverrides = commonCrateOverrides pkgs;
                    };

                    # musl, x86_64. No camera SDK, so nothing here needs a per-crate
                    # override beyond the shared jpegxl and libc++ ones.
                    #
                    # NOT `isStatic = true`. Rust's musl target already links crt-static, so
                    # the binary comes out static either way, and asking nixpkgs for a static
                    # cross as well makes rustc emit `-static-pie` -- which then cannot link
                    # the toolchain's own non-PIE libstdc++.a:
                    #   relocation R_X86_64_32 against `__gxx_personality_v0` can not be used
                    #   when making a PIE object
                    # and libstdc++ is unavoidable here because gamut-jxl-sys pulls it in.
                    x86MuslPkgs = crossPkgsFor "x86_64-unknown-linux-musl";
                    linuxX86C2N = crate2nixFor {
                        targetPkgs = x86MuslPkgs;
                        crateOverrides = crossCrateOverrides x86MuslPkgs;
                    };

                    # The Pi. Every camera in one binary, so unplugging a sensor is not a
                    # rebuild; a backend that finds no hardware reports itself disengaged.
                    #
                    # Only two crates need anything beyond the cross stdenv:
                    #   realsense-sys  -- the SDK, found through pkg-config. PKG_CONFIG_ALLOW_CROSS
                    #                     because pkg-config refuses a cross build otherwise, and
                    #                     an explicit -L because librealsense's own realsense2.pc
                    #                     hardcodes libdir=''${prefix}/lib/x86_64-linux-gnu whatever
                    #                     it was built for (there is a literal #TODO above the line).
                    #   lite_record    -- depthai, whose public headers include their dependencies'
                    #                     headers directly and whose build script stdenv adds no
                    #                     target include paths to, so each one has to be named.
                    linuxArm64C2N = crate2nixFor {
                        targetPkgs = aarch64Gnu;
                        rootFeatures = [ "default" "realsense" "oakd" "livox" ];
                        crateOverrides = (crossCrateOverrides aarch64Gnu) // {
                            realsense-sys = attrs: {
                                nativeBuildInputs = (attrs.nativeBuildInputs or [ ]) ++ [ pkgs.pkg-config ];
                                buildInputs = (attrs.buildInputs or [ ]) ++ [ aarch64Gnu.librealsense ];
                                PKG_CONFIG_ALLOW_CROSS = 1;
                                extraLinkFlags = [ "-L${pkgs.lib.getLib aarch64Gnu.librealsense}/lib" ];
                            };
                            lite_record = attrs: {
                                extraRustcOpts = (attrs.extraRustcOpts or [ ])
                                    ++ [ "-L native=${cxxRuntimeShim}/lib" ];
                                DEPTHAI_DIR = depthaiCoreFor aarch64Gnu;
                                DEPTHAI_INCLUDE_DIRS = pkgs.lib.concatMapStringsSep ":"
                                    (library: "${pkgs.lib.getDev library}/include")
                                    (with aarch64Gnu; [ nlohmann_json spdlog fmt xtensor xtl ]);
                            };
                        };
                    };

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
                    # The encode-cost bench as its own aarch64 binary, so the codec
                    # decision can be measured on the Pi without putting a rust toolchain
                    # there. It only touches `image` and `msgs`, so it needs neither the
                    # camera SDKs nor their features -- plain musl is enough.
                    linuxArm64BenchC2N =
                        let musl = crossPkgsFor "aarch64-unknown-linux-musl"; in
                        crate2nixFor {
                            targetPkgs = musl;
                            rootFeatures = [ "default" "bench" ];
                            crateOverrides = crossCrateOverrides musl;
                        };

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
                in
                {
                    lite_record = nativeC2N;
                    default = nativeC2N;

                    linux-x86 = linuxX86C2N;
                    linux-arm64 = linuxArm64C2N;
                    linux-arm64-bench = linuxArm64BenchC2N;
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
