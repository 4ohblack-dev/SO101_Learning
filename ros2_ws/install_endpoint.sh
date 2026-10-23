#!/usr/bin/env bash
# Run in Ubuntu-22.04: bash ros2_ws/install_endpoint.sh
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
endpoint_dir="$workspace_dir/src/ROS-TCP-Endpoint"
output_dir="$HOME/ros2_builds/RosArmSimulation"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humbleが見つかりません。Ubuntu-22.04で実行してください。' >&2
  exit 1
fi
for tool in git colcon rosdep; do
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "必要なコマンドがありません: $tool" >&2
    echo 'sudo apt update && sudo apt install git ros-dev-tools を実行してください。' >&2
    exit 1
  fi
done

source /opt/ros/humble/setup.bash
if [[ -d "$endpoint_dir/.git" ]]; then
  if [[ "$(git -C "$endpoint_dir" branch --show-current)" != main-ros2 ]]; then
    echo '既存のEndpointがmain-ros2ブランチではありません。変更内容を確認してください。' >&2
    exit 1
  fi
  echo '既存のmain-ros2チェックアウトを使用します（変更・更新はしません）。'
elif [[ -e "$endpoint_dir" ]]; then
  echo "clone先が既に存在します。内容を確認してください: $endpoint_dir" >&2
  exit 1
else
  mkdir -p "$workspace_dir/src"
  git clone --branch main-ros2 --single-branch \
    https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git "$endpoint_dir"
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update --rosdistro humble
rosdep install --from-paths "$endpoint_dir" --ignore-src --rosdistro humble -r -y

cd "$workspace_dir"
colcon --log-base "$output_dir/log" build \
  --base-paths "$endpoint_dir" \
  --build-base "$output_dir/build" \
  --install-base "$output_dir/install" \
  --packages-select ros_tcp_endpoint

source "$output_dir/install/setup.bash"
actual_prefix="$(ros2 pkg prefix ros_tcp_endpoint)"
if [[ "$actual_prefix" != "$output_dir/install/ros_tcp_endpoint" ]]; then
  echo "想定外のパッケージが選択されています: $actual_prefix" >&2
  exit 1
fi
ros2 pkg executables ros_tcp_endpoint | grep -Fx 'ros_tcp_endpoint default_server_endpoint'
echo "導入確認完了: $actual_prefix"
echo '起動するには次を実行してください:'
printf 'source /opt/ros/humble/setup.bash\nsource "%s/install/setup.bash"\n' "$output_dir"
echo 'ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000'
