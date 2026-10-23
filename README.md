# RosArmSimulation

UnityとROS 2を使ったロボットシミュレーション用のプロジェクトです。

## VS Codeで開く

VS Codeの「ファイル → ファイルからワークスペースを開く」で `RosArmSimulation.code-workspace` を選びます。Unityと `ros2_ws` を同じツリーで編集できます。相対パスを使用しているため、プロジェクトを別の場所へcloneしても利用できます。

Windowsでこのワークスペースから新しいターミナルを開くと、既定でWSLのUbuntu-22.04が起動します。プロジェクトルートで `source ros2_ws/use_udp_only.bash` を実行してROS 2環境を読み込んでください。Endpointと受信確認にはターミナルを2つ使います。

この設定はWindows版VS Code内のターミナルをWSLにするものです。Python拡張機能自体もUbuntu側で動かしたい場合は、WSL拡張機能からWSL内で開き直してください。既存の `.vscode` フォルダ設定は保持しています。

## フォルダ構成

```text
RosArmSimulation/
├─ Assets/              # Unityのシーン・C#スクリプト
├─ Packages/            # Unityのパッケージ設定
├─ ProjectSettings/     # Unityのプロジェクト設定
├─ ros2_ws/
│  └─ src/              # ROS 2のパッケージ・スクリプト
└─ README.md
```

ROS 2パッケージは `ros2_ws/src/` の下に追加します。ROS 2本体はUbuntu側の `/opt/ros/humble` を使用します。

## UbuntuとROS 2の起動

PowerShellでUbuntu 22.04に入ります。

```powershell
wsl -d Ubuntu-22.04
```

Ubuntu内でROS 2を読み込み、プロジェクトに移動します。

```bash
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/yohku/Documents/Unity/RosArmSimulation
```

## ROS 2パッケージのビルド

パッケージを追加した後、Ubuntu内で次のコマンドを実行します。生成物はUbuntu側に保存し、ソースコードはこのプロジェクトで管理します。`colcon` と各パッケージの依存関係が必要です。

```bash
source /opt/ros/humble/setup.bash
cd /mnt/c/Users/yohku/Documents/Unity/RosArmSimulation/ros2_ws
colcon --log-base "$HOME/ros2_builds/RosArmSimulation/log" build \
  --base-paths src \
  --build-base "$HOME/ros2_builds/RosArmSimulation/build" \
  --install-base "$HOME/ros2_builds/RosArmSimulation/install"
source "$HOME/ros2_builds/RosArmSimulation/install/setup.bash"
```

## TCP Endpointの導入

Ubuntu 22.04内で以下を実行します。ROS 2用の `main-ros2` ブランチを取得し、依存関係をインストールして、Ubuntu側にビルドします。途中でsudoのパスワード入力が必要になることがあります。

```bash
cd /mnt/c/Users/yohku/Documents/Unity/RosArmSimulation
bash ros2_ws/install_endpoint.sh
```

スクリプトはパッケージの配置先と実行ファイルの登録まで確認します。既存の同名チェックアウトがあれば更新せず使用します。古いワークスペースを読み込んでいないUbuntuターミナルで実行してください。

導入後の起動コマンド:

```bash
source /opt/ros/humble/setup.bash
source "$HOME/ros2_builds/RosArmSimulation/install/setup.bash"
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
  -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
```

## 通信確認済みの起動方法

2026-10-23、WSL 2 / Ubuntu 22.04.5 / ROS 2 HumbleとUnity 6000.4.10f1で、UDP設定適用後にUnityからROS 2への文字列送信成功を確認しました。共有メモリの初期化エラーを避けるため、両方のUbuntuターミナルで `use_udp_only.bash` を読み込みます。

ターミナルA（Endpointを起動したままにする）:

```bash
cd /mnt/c/Users/yohku/Documents/Unity/RosArmSimulation
source ros2_ws/use_udp_only.bash
source "$HOME/ros2_builds/RosArmSimulation/install/setup.bash"
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
  -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
```

別のUbuntu-22.04ターミナルB:

```bash
cd /mnt/c/Users/yohku/Documents/Unity/RosArmSimulation
source ros2_ws/use_udp_only.bash
ros2 topic echo /unity_ping std_msgs/msg/String
```

Unityで `Assets/Scenes/SO100ImportTest.unity` を開いて再生します。`RosBridgeTest` に送信スクリプトを配置済みです。`data: Hello from Unity` が表示されれば成功です。標準メッセージ型はConnector内蔵のものを使用し、重複生成しないでください。

## GitHubからの復元

```bash
git clone https://github.com/4ohblack-dev/SO101_Learning.git
```

clone先のルートをUnity Hubから既存プロジェクトとして開きます。Unityのキャッシュは自動生成されます。上記の絶対パスは自分のclone先へ読み替えてください。

TCP Endpoint本体は外部依存としてGit管理対象から除外し、取得元と今回使用したコミットを `ros2_ws/dependencies.repos` に記録しています。Ubuntu側で `vcs` が利用可能なら、初回は次で同じコミットを取得できます（既存のEndpointがある場合は実行不要）。

```bash
cd ros2_ws
vcs import src < dependencies.repos
```

この方法ではdetached HEADになるため、ビルドは本READMEの手動ビルド手順を使用してください。`install_endpoint.sh` は `main-ros2` ブランチを使う簡易導入用で、固定コミットの再現とは異なります。

GitにはAssets（.metaを含む）、Packages、ProjectSettings、ROS用スクリプト・設定を保存します。Library、Logs、UserSettings、Backups、ビルド生成物は保存しません。外部パッケージやモデルのライセンスはそれぞれの配布元に従います。
