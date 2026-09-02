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

                            "CARGO_TARGET_${targetUpper}_LINKER" = "${binDirectory}/${prefix}cc";

                            # zstd-sys, lz4-sys and ring compile C from build scripts. Without
                            # these cc-rs reaches for the host clang with --target=<triple>,
                            # which has no matching sysroot and dies on `#include <string.h>`.
                            "CC_${targetSnake}" = "${binDirectory}/${prefix}cc";
                            "CXX_${targetSnake}" = "${binDirectory}/${prefix}c++";
                            "AR_${targetSnake}" = "${binDirectory}/${prefix}ar";
                        } // pkgs.lib.optionalAttrs (sdkLibraries != [ ]) {
                            nativeBuildInputs = [ pkgs.pkg-config ];

                            PKG_CONFIG_PATH = pkgs.lib.concatMapStringsSep ":"
                                (library: "${pkgs.lib.getDev library}/lib/pkgconfig") sdkLibraries;

                            # Two separate jobs here.
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
                                pkgs.lib.concatMapStringsSep " "
                                    (library:
                                        "-L native=${pkgs.lib.getLib library}/lib"
                                        + " -C link-arg=-Wl,-rpath,${pkgs.lib.getLib library}/lib")
                                    sdkLibraries;
                        });

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

                            # By default librealsense reaches the D4xx IMU through the
                            # kernel's HID-to-IIO bridge, which needs hid-sensor-hub,
                            # hid-sensor-accel-3d and hid-sensor-gyro-3d. Raspberry Pi
                            # OS ships none of them, so the motion module is invisible
                            # and every request naming it fails to resolve. The RSUSB
                            # backend talks to the same hardware over libusb instead
                            # and needs no kernel support at all.
                            librealsense = previous.librealsense.overrideAttrs
                                (old: {
                                    cmakeFlags = (old.cmakeFlags or [ ])
                                        ++ [ "-DFORCE_RSUSB_BACKEND=ON" ];
                                });
                        });
                in
                {
                    lite_record = native;
                    default = native;
                    linux-x86 = buildCross {
                        rustTarget = "x86_64-unknown-linux-musl";
                        crossPkgs = crossPkgsFor "x86_64-unknown-linux-musl";
                    };
                    linux-arm64 = buildCross {
                        rustTarget = "aarch64-unknown-linux-musl";
                        crossPkgs = crossPkgsFor "aarch64-unknown-linux-musl";
                    };
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
                    linux-arm64-realsense = buildCross {
                        rustTarget = "aarch64-unknown-linux-gnu";
                        crossPkgs = aarch64Gnu;
                        features = [ "realsense" "livox" ];
                        sdkLibraries = [ aarch64Gnu.librealsense ];
                    };
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
                            echo "  nix build .#linux-arm64            -- aarch64 musl, static, no camera SDK"
                            echo "  nix build .#linux-x86              -- x86_64 musl, static, no camera SDK"
                            echo "  nix build .#linux-arm64-realsense  -- aarch64 gnu, librealsense + livox"
                            echo ""
                        '';
                    };
                });
        };
}
