#!/usr/bin/env bash
# Shared Linux build entry point; orchestration and publication belong to the forge.
set -Eeuo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
build=${COVERAGE_BUILD_DIR:-$root/build-coverage}
deps=${COVERAGE_DEPS_DIR:-$root/deps/build-coverage}
out=${COVERAGE_OUTPUT_DIR:-$root/coverage-output}
export CC=clang-18 CXX=clang++-18
build_jobs=${COVERAGE_BUILD_JOBS:-2}
if [[ ! $build_jobs =~ ^[1-4]$ ]]; then
  echo 'COVERAGE_BUILD_JOBS must be between 1 and 4.' >&2
  exit 2
fi
build_jobs=$(python3 - "$build_jobs" <<'PY'
import os, sys
print(min(int(sys.argv[1]), len(os.sched_getaffinity(0))))
PY
)
export CMAKE_BUILD_PARALLEL_LEVEL=$build_jobs CTEST_PARALLEL_LEVEL=2
# CPython's PGO configure looks for an unversioned tool unless this is explicit.
LLVM_PROFDATA=$(command -v llvm-profdata-18)
export LLVM_PROFDATA

# A local prefix needs the same input binding as a CI cache. Never silently use
# an old prefix after dependency sources, build tools or the profile change.
deps_key=$(python3 - "$root" <<'PY'
import hashlib
import subprocess
import sys

root = sys.argv[1]
subprocess.run(["git", "-C", root, "diff", "--exit-code", "HEAD", "--", "deps", "cmake/modules"],
               check=True, stdout=subprocess.DEVNULL)
commands = (["git", "-C", root, "ls-tree", "-r", "HEAD", "deps", "cmake/modules"],
            ["clang-18", "--version"], ["ld.lld-18", "--version"],
            ["cmake", "--version"], ["ninja", "--version"], ["uname", "-m"])
digest = hashlib.sha256(b"linux-clang18-release-core-cad-deps-v2-gtk3\0")
for command in commands:
    digest.update(subprocess.check_output(command))
    digest.update(b"\0")
print(digest.hexdigest())
PY
)
deps_only=false
if [[ ${1:-} == --deps-key && $# == 1 ]]; then
  printf '%s\n' "$deps_key"
  exit 0
elif [[ ${1:-} == --deps-only && $# == 1 ]]; then
  deps_only=true
elif (( $# != 0 )); then
  echo 'Usage: build_coverage.sh [--deps-key|--deps-only]' >&2
  exit 2
fi

# Some dependency recipes count CPUs themselves instead of honoring --parallel.
# Bound the entire process tree, including nested builds; local default is two.
if [[ ${ORCA_COVERAGE_AFFINITY_SET:-0} != 1 ]]; then
  cpus=$(python3 -c 'import os,sys; print(",".join(map(str, sorted(os.sched_getaffinity(0))[:int(sys.argv[1])])))' "$build_jobs")
  export ORCA_COVERAGE_AFFINITY_SET=1
  exec taskset --cpu-list "$cpus" bash "$0" "$@"
fi
if [[ $deps_only != true ]]; then
  # These canonical tests otherwise skip their comma-decimal locale cases.
  python3 - <<'PY'
import locale
try:
    locale.setlocale(locale.LC_NUMERIC, "de_DE.UTF-8")
except locale.Error:
    raise SystemExit("Coverage tests require de_DE.UTF-8; install locale data and generate that locale first.")
if locale.localeconv()["decimal_point"] != ",":
    raise SystemExit("de_DE.UTF-8 must provide a comma decimal separator for the locale tests.")
PY
fi
mkdir -p "$out"
stage() {
  local name=$1; shift
  /usr/bin/time -v -o "$out/$name.time.txt" "$@" 2>&1 | tee "$out/$name.log"
}
{
  git -C "$root" rev-parse HEAD
  git -C "$root" status --porcelain --untracked-files=no
  clang-18 --version
  llvm-cov-18 --version
  llvm-profdata-18 --version
  cmake --version
  ninja --version
  taskset -pc $$
  locale -a
  df -h "$out"
} > "$out/environment.txt"
common=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang-18
        -DCMAKE_CXX_COMPILER=clang++-18 -DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=lld-18
        -DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=lld-18)
if command -v ccache >/dev/null; then
  common+=(-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
fi
# Instrument only project code. Dependencies keep the repository's ordinary build
# settings. GLFW and GTK3 wxWidgets are required by root configuration and the
# canonical color-support tests even with the application GUI disabled.
marker=$deps/OrcaSlicer_dep/.coverage-ready
if [[ ! -f "$marker" || $(cat "$marker") != "$deps_key" ]]; then
  stage deps-configure cmake -S "$root/deps" -B "$deps" -G Ninja "${common[@]}" \
    -DSLIC3R_CAD=ON -DDEP_WX_GTK3=ON
  stage deps-build cmake --build "$deps" --parallel 1 --target \
    dep_Boost dep_Eigen dep_TBB dep_Cereal dep_CGAL dep_Draco dep_NLopt \
    dep_OpenVDB dep_SLVS dep_OCCT dep_OpenCV dep_Assimp dep_python3 dep_libnoise dep_Qhull \
    dep_GLFW dep_wxWidgets
  printf '%s\n' "$deps_key" > "$marker"
fi
if [[ $deps_only == true ]]; then
  exit 0
fi
stage configure cmake -S "$root" -B "$build" -G Ninja "${common[@]}" \
  -DSLIC3R_GUI=OFF -DSLIC3R_CAD=ON -DSLIC3R_STATIC=ON -DBUILD_TESTS=ON \
  -DCMAKE_PREFIX_PATH="$deps/OrcaSlicer_dep/usr/local" -DDEP_BUILD_DIR="$deps" \
  -DSLICER_ENABLE_LLVM_SOURCE_COVERAGE=ON
cp "$build/CMakeCache.txt" "$out/CMakeCache.txt"
stage release-build cmake --build "$build" --config Release --parallel "$build_jobs" --target \
  libslic3r_tests fff_print_tests sla_print_tests libnest2d_tests filament_group_tests
stage collect python3 "$root/scripts/slicer_coverage.py" run \
  --root "$root" --build-dir "$build" --source-root src/libslic3r \
  --object "$build/tests/libslic3r/libslic3r_tests" \
  --object "$build/tests/fff_print/fff_print_tests" \
  --object "$build/tests/sla_print/sla_print_tests" \
  --object "$build/tests/libnest2d/libnest2d_tests" \
  --object "$build/tests/filament_group/filament_group_tests" \
  --manifest "$build/slicer-coverage/libslic3r.sources.txt" \
  --manifest "$build/slicer-coverage/libslic3r_cgal.sources.txt" \
  --compile-commands "$build/compile_commands.json" \
  --test-command 'ctest --test-dir {build}/tests -C Release --parallel 2 --timeout 1200 -LE "NotWorking|RequiresApp" --no-tests=error --output-on-failure' \
  --llvm-cov llvm-cov-18 --llvm-profdata llvm-profdata-18 \
  --measure-only --html --timeout 7200 --out "$out/measurement"
