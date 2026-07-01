#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

cd "${REPO_ROOT}"

bash -n .xgc2/scripts/*.sh

nested_git="$(
  find . \
    -path ./.git -prune -o \
    -path ./multirotor-controller -prune -o \
    -path ./ugv-controller -prune -o \
    -path './*/build' -prune -o \
    -path './*/devel' -prune -o \
    -path './*/install' -prune -o \
    -path ./.work -prune -o \
    -path ./debs -prune -o \
    -name .git -print
)"
if [[ -n "${nested_git}" ]]; then
  echo "Unexpected nested .git path found." >&2
  echo "${nested_git}" >&2
  exit 1
fi

if git ls-files | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.work|debs)(/|$)' >/dev/null; then
  echo "Generated build artifacts are tracked." >&2
  git ls-files | grep -E '(^|/)(build|devel|install|\.catkin_tools|\.work|debs)(/|$)' >&2
  exit 1
fi

required_files=(
  .gitmodules
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
  README.md
)

for file in "${required_files[@]}"; do
  if [[ ! -f "${file}" ]]; then
    echo "Missing required file: ${file}" >&2
    exit 1
  fi
done

for path in \
  px4_multirotor_controller \
  multirotor_reference_trajectory \
  unicycle_ugv_controller \
  unicycle_reference_trajectory; do
  if [[ -e "${path}" ]]; then
    echo "Controller source path should live only in split repos: ${path}" >&2
    exit 1
  fi
done

git config -f .gitmodules --get submodule.multirotor-controller.url \
  | grep -qx 'git@github.com:lxk36/xgc2-multirotor-controller.git'
git config -f .gitmodules --get submodule.multirotor-controller.branch | grep -qx noetic
git config -f .gitmodules --get submodule.ugv-controller.url \
  | grep -qx 'git@github.com:lxk36/xgc2-ugv-controller.git'
git config -f .gitmodules --get submodule.ugv-controller.branch | grep -qx noetic
git ls-files --stage multirotor-controller | grep -q '^160000 '
git ls-files --stage ugv-controller | grep -q '^160000 '

grep -q "id: xgc2-controller" .xgc2/product.yml
grep -Eq '^version: [0-9]+\.[0-9]+\.[0-9]+-[0-9]+$' .xgc2/product.yml
grep -q "ros-noetic-xgc2-multirotor-controller (>= 1.0.13-1)" .xgc2/product.yml
grep -q "ros-noetic-xgc2-ugv-controller (>= 1.0.1-1)" .xgc2/product.yml
grep -q "PACKAGE=\"ros-\${ROS_DISTRO}-xgc2-controller\"" .xgc2/scripts/package_debs.sh
grep -q "Architecture: all" .xgc2/scripts/package_debs.sh
grep -q "ros-\${ROS_DISTRO}-xgc2-multirotor-controller (>= 1.0.13-1)" .xgc2/scripts/package_debs.sh
grep -q "ros-\${ROS_DISTRO}-xgc2-ugv-controller (>= 1.0.1-1)" .xgc2/scripts/package_debs.sh
grep -q "check_version_bump.sh --ci" .github/workflows/build-debs.yml

echo "Package compliance checks passed."
