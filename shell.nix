# vim: set ts=2 sts=2 sw=2 et :
with import <nixpkgs> {};
stdenv.mkDerivation rec {
  name = "gem5";
  env = buildEnv { name = name; paths = buildInputs; };

  M5_PATH = toString (../. + "/gem5-system-files/aarch-system-20170426/");
  NIX_HARDENING_ENABLE = "";

  buildInputs = [
    gcc8 binutils git
    python2Full
    python2Packages.six
    scons swig zlib m4 protobuf boost
    # For tcmalloc
    gperftools
    gdb
  ];
}
