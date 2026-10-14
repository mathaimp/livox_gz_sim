{
  description = "dev shell with pixi and cuda out of the box";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    nix-gl-host.url = "github:numtide/nix-gl-host";
  };

  outputs =
    {
      self,
      nixpkgs,
      nix-gl-host,
    }:
    let
      pkgs = import nixpkgs {
        system = "x86_64-linux";
      };
    in
    {
      devShells.x86_64-linux.default =
        with pkgs;
        mkShell {

          packages = [
            nix-gl-host.defaultPackage.x86_64-linux
            pixi
            # git wrapper: scrub LD_LIBRARY_PATH so git resolves libs from
            # its own closure — any LDP-visible pcre2 (nix or conda) makes
            # the linker print "no version information available", which
            # lazygit treats as a fatal error. Nothing git runs needs LDP.
            (pkgs.writeShellScriptBin "git" ''
              exec env -u LD_LIBRARY_PATH ${pkgs.git}/bin/git "$@"
            '')
          ];

          shellHook = ''
              export SSL_CERT_FILE=/etc/ssl/certs/ca-bundle.crt

              # Build additional library paths
              EXTRA_LIBS="${
                pkgs.lib.makeLibraryPath [
                  pkgs.libGL
                  pkgs.stdenv.cc.cc.lib
                ]
              }"

            export LD_LIBRARY_PATH="$EXTRA_LIBS:$LD_LIBRARY_PATH"

            export SCOUT_NIX_LIBS="${
              pkgs.lib.makeLibraryPath [
                pkgs.libGL
                pkgs.glfw
                pkgs.wayland
                pkgs.libx11
              ]
            }"

            # GPU driver libs (libcuda, GLX, EGL) via nixglhost. Run with a
            # clean LD_LIBRARY_PATH: it is self-poisoned when its own output
            # is already on the path.
            export LD_LIBRARY_PATH="$(env -u LD_LIBRARY_PATH nixglhost -p):$SCOUT_NIX_LIBS:$LD_LIBRARY_PATH"
          '';
        };
    };
}
