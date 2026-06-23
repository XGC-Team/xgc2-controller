#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

bash -n .xgc2/scripts/*.sh

nested_git="$(
  find . \
    -path ./.git -prune -o \
    -path './*/build' -prune -o \
    -path './*/devel' -prune -o \
    -path './*/install' -prune -o \
    -path ./.work -prune -o \
    -path ./debs -prune -o \
    -name .git -print
)"
if [[ -n "${nested_git}" ]]; then
  echo "Nested .git directory found." >&2
  echo "${nested_git}" >&2
  exit 1
fi

if git ls-files | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.work|debs)(/|$)' >/dev/null; then
  echo "Generated build artifacts are tracked." >&2
  git ls-files | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.work|debs)(/|$)' >&2
  exit 1
fi

required_files=(
  .clang-format
  .clang-tidy
  .github/workflows/build-debs.yml
  .xgc2/product.yml
  .xgc2/scripts/build_debs_in_docker.sh
  .xgc2/scripts/check_core_libraries.sh
  .xgc2/scripts/check_cpp_quality.sh
  .xgc2/scripts/check_installed_packages.sh
  .xgc2/scripts/check_package_compliance.sh
  .xgc2/scripts/check_version_bump.sh
  .xgc2/scripts/package_debs.sh
  .xgc2/scripts/publish_apt_repo.sh
  .xgc2/scripts/setup_xgc2_apt_source.sh
  multirotor_controller/CMakeLists.txt
  multirotor_controller/package.xml
  multirotor_controller/launch/uav_nmpc_controller.launch
  multirotor_controller/config/uav_nmpc.yaml
  reference_trajectory/CMakeLists.txt
  reference_trajectory/package.xml
  reference_trajectory/msg/UavFlatTrajectory.msg
  reference_trajectory/msg/UavBsplineTrajectory.msg
  reference_trajectory/include/reference_trajectory/nmpc_reference_trajectory.h
  reference_trajectory/launch/uav_reference_trajectory.launch
)

for file in "${required_files[@]}"; do
  if [[ ! -f "${file}" ]]; then
    echo "Missing required file: ${file}" >&2
    exit 1
  fi
done

grep -q "id: xgc2-controller" .xgc2/product.yml
grep -Eq '^version: [0-9]+\.[0-9]+\.[0-9]+-[0-9]+$' .xgc2/product.yml
grep -q "<name>multirotor_controller</name>" multirotor_controller/package.xml
grep -q "<name>reference_trajectory</name>" reference_trajectory/package.xml
grep -q "run_tests_reference_trajectory run_tests_multirotor_controller" .github/workflows/build-debs.yml
grep -q "check_version_bump.sh --ci" .github/workflows/build-debs.yml
grep -q "PACKAGE=\"ros-\${ROS_DISTRO}-xgc2-controller\"" .xgc2/scripts/package_debs.sh
grep -q "REFERENCE_ROS_PACKAGE=\"reference_trajectory\"" .xgc2/scripts/package_debs.sh
grep -q "xgc2-acados (>= 0.1.0-3~focal)" .xgc2/product.yml
grep -q "xgc2-acados (>= 0.1.0-3~focal)" .xgc2/scripts/package_debs.sh

if grep -R --exclude='check_package_compliance.sh' "ros-noetic-xgc2-reference" \
  .github .xgc2 README.md multirotor_controller reference_trajectory >/dev/null; then
  echo "Deprecated ros-noetic-xgc2-reference dependency found." >&2
  exit 1
fi

echo "Package compliance checks passed."
