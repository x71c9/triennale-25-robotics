{ pkgs ? import <nixpkgs> {} }:

let
  dynamixel_sdk = pkgs.callPackage ./dynamixel_sdk.nix {};
  myPython = pkgs.python311.withPackages (ps: with ps; [
    dynamixel_sdk
    ps.pyserial
  ]);
in
pkgs.mkShell {
  buildInputs = [ myPython ];

  shellHook = ''
    echo "Python 3.11 shell with dynamixel_sdk from source"
  '';
}

