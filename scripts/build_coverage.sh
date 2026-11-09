#!/usr/bin/env bash
# Shared Linux build entry point; orchestration and publication belong to the forge.
set -Eeuo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
build=${COVERAGE_BUILD_DIR:-$root/build-coverage}
deps=${COVERAGE_DEPS_DIR:-$root/deps/build-coverage}
out=${COVERAGE_OUTPUT_DIR:-$root/coverage-output}
export CC=clang-18 CXX=clang++-18
export CMAKE_BUILD_PARALLEL_LEVEL=2 CTEST_PARALLEL_LEVEL=2

# Some dependency recipes count CPUs themselves instead of honoring --parallel.
# Keep the entire process tree on two available CPUs, including nested builds.
if [[ ${ORCA_COVERAGE_AFFINITY_SET:-0} != 1 ]]; then
  cpus=$(python3 -c 'import os; print(",".join(map(str, sorted(os.sched_getaffinity(0))[:2])))')
  export ORCA_COVERAGE_AFFINITY_SET=1
  exec taskset --cpu-list "$cpus" bash "$0" "$@"
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
  df -h "$out"
} > "$out/environment.txt"
common=(-DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=clang-18
        -DCMAKE_CXX_COMPILER=clang++-18 -DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=lld-18
        -DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=lld-18)
if command -v ccache >/dev/null; then
  common+=(-DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache)
fi
# Instrument only project code. Dependencies keep the repository's ordinary build
# settings; GUI/media-only dependency targets are unnecessary for this scope.
if [[ ! -f "$deps/OrcaSlicer_dep/.coverage-ready" ]]; then
  stage deps-configure cmake -S "$root/deps" -B "$deps" -G Ninja "${common[@]}" -DSLIC3R_CAD=ON
  stage deps-build cmake --build "$deps" --parallel 1 --target \
    dep_Boost dep_Eigen dep_TBB dep_Cereal dep_CGAL dep_Draco dep_NLopt \
    dep_OpenVDB dep_SLVS dep_OCCT dep_OpenCV dep_Assimp dep_python3 dep_libnoise dep_Qhull
  touch "$deps/OrcaSlicer_dep/.coverage-ready"
fi
stage configure cmake -S "$root" -B "$build" -G Ninja "${common[@]}" \
  -DSLIC3R_GUI=OFF -DSLIC3R_CAD=ON -DSLIC3R_STATIC=ON -DBUILD_TESTS=ON \
  -DCMAKE_PREFIX_PATH="$deps/OrcaSlicer_dep/usr/local" -DDEP_BUILD_DIR="$deps" \
  -DSLICER_ENABLE_LLVM_SOURCE_COVERAGE=ON
cp "$build/CMakeCache.txt" "$out/CMakeCache.txt"
stage release-build cmake --build "$build" --config Release --parallel 2 --target \
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
