# hello_ros2 開発環境

ROS 2 の開発環境を Docker Compose で起動します。環境は2つあります。

- `ros2-jazzy`：ROS 2 Jazzy（Ubuntu 24.04）と Gazebo Harmonic。`src` のサンプルはこちらで動かします。
  今後の開発もこちらで進めます。
- `ros2-dev`：ROS 2 Humble（Ubuntu 22.04）と Tesseract のソースビルド。Tesseract はまだ Jazzy に移していません。

以下の Tesseract の説明は `ros2-dev` についてです。
既存の Tesseract のソースビルドを維持し、RViz・turtlesim・URDF の表示に使える
ブラウザ経由のデスクトップ（noVNC）を追加しています。macOS では XQuartz は不要です。

## 検証記録（2026-10-26）

- `docker compose --progress plain build ros2-dev` 成功（既存キャッシュ利用、全ソースの再コンパイルは未実施）。
- `docker compose up -d --wait --wait-timeout 90` 成功、コンテナは `healthy`。
- コンテナ内で cereal の CMake 設定・コンパイル・シリアライズ往復確認に成功。
- ROS 2 Humble、Tesseract のパッケージ検出とメッセージの Python import を確認。
- turtlesim を実際に起動し、別ノードで `/turtle1/pose` を受信。
- noVNC の HTTP 200 とブラウザ接続を確認。
- `simple_arm.urdf` を RViz に表示し、Global Status が Ok、関節操作画面も表示されることを確認。

## 構成

```text
hello_ros2/
├── Dockerfile                 # Humble / Tesseract / GUI ツールの構築（ros2-dev）
├── compose.yml                # 開発コンテナ、ポート、保存領域
├── docker/
│   ├── jazzy/Dockerfile        # Jazzy / Gazebo Harmonic / ros2_control の構築（ros2-jazzy）
│   ├── entrypoint.sh           # ROS 2 とワークスペースの環境読み込み
│   └── supervisord.conf        # 仮想画面、ウィンドウ管理、noVNC
└── src/
    ├── hello_py/               # Python の talker と listener
    ├── hello_gazebo/           # Gazebo で箱を落とすサンプル
    ├── hello_arm/              # Gazebo で3関節アームと5本指ハンドを動かすサンプル
    └── simple_arm.urdf         # 既存のアームモデル
```

`src` はホストと共有します。`build`・`install`・`log` は Docker の名前付きボリュームに
保存されます。Tesseract は `/tesseract_ws`、開発用ワークスペースは `/workspace` です。
Tesseract ROS 2 はリリース 0.35.0 のコミットに固定しています。
そのリリースに同梱された依存定義は旧版 0.33.0 を参照するため、
[公式の修正後の依存定義](https://github.com/tesseract-robotics/tesseract_ros2/blob/568b3761b05ddfdf42eb16aa2b9a0cf08abe663d/dependencies.repos)
に合わせた `docker/tesseract-dependencies.repos` を利用します。
Ubuntu 22.04 向けに不足する依存キーは `docker/rosdep-jammy.yaml` で補います。
Gazebo の公式 APT リポジトリと、Tesseract 公式 Dockerfile が使用する
`ppa:levi-armstrong/tesseract-robotics` から追加の開発ライブラリを取得します。
Taskflow は取得済みの v3.5.0 ソースから直接インストールします。
cereal は配布パッケージの CMake 設定が `/include` を参照してビルドに失敗する場合が
あるため、公式 v1.3.2 のコミットを検証して `/usr/local` にインストールします。
Tesseract のビルドにはその `cereal_DIR` を明示的に渡します。
事前に `docker/check-cereal` で CMake の参照先とシリアライズの動作を確認します。

## 起動

Docker Desktop の起動後、このフォルダで実行します。

```bash
docker compose up -d --build
docker compose ps
docker compose exec ros2-dev bash
```

初回は Tesseract と依存ライブラリのコンパイルに時間がかかります。
Docker Desktop のメモリはまず 8 GB 以上を目安に設定してください。
ビルドはパッケージを順番に処理し、各コンパイルの並列数を 2 に制限しています。
Intel Mac 用に `linux/amd64` を指定しています。Apple Silicon ではエミュレーションで
実行するため、ビルドがさらに遅くなる場合があります。

画面は次の URL をブラウザで開いてください。

<http://localhost:6080/vnc.html?autoconnect=true&resize=scale>

画面の右クリックメニューからターミナルを開くか、ホストのターミナルで
`docker compose exec ros2-dev bash` を実行して作業します。
画面のポートはホストの `127.0.0.1` に公開し、VNC ポートは公開していません。

## 動作確認

```bash
docker compose exec ros2-dev bash -ic 'ros2 doctor --report'
docker compose exec ros2-dev bash -ic 'ros2 pkg prefix tesseract_msgs && ros2 pkg prefix tesseract_rosutils'
docker compose exec ros2-dev bash -ic 'ros2 run turtlesim turtlesim_node'
```

最後のコマンドを実行すると、ブラウザの画面に turtlesim が表示されます。
終了はコマンドを実行したターミナルで Ctrl+C を押します。

既存のアームを RViz で表示する場合:

```bash
docker compose exec ros2-dev bash -ic 'ros2 launch urdf_tutorial display.launch.py model:=/workspace/src/simple_arm.urdf'
```

## Jazzy + Gazebo Harmonic 環境

```bash
docker compose up -d --build ros2-jazzy
docker compose exec ros2-jazzy bash
```

画面は <http://localhost:6081/vnc.html?autoconnect=true&resize=scale> で開きます。
Humble 環境の 6080 とは別のポートで、ビルド成果物も別のボリュームに保存します。
ポートは `.env` の `NOVNC_PORT_JAZZY` で変更できます。
Gazebo は ROS のパッケージとして同梱されており、ROS の環境を読み込むと `gz sim` が使えます。
次の段階で使う ros2_control と gz_ros2_control も入れてあります。

検証記録（2026-11-07）：Gazebo Sim 8.15（Harmonic）で次を確認しました。

- 箱の自由落下は理論値との差が最大 1.2 mm で、高さ 0.500 m で静止。
- アームと5本指ハンドの自動動作。指の追従誤差は平均 0.011 rad。
- 手動指令の保持。肩 0.8 rad の指令に対して 0.8004 rad。
- 画面なしの実時間比は約 1.0、画面ありでは約 0.35〜0.55（ソフトウェア描画）。

## サンプルパッケージ

`src` には次の3つのサンプルがあります。`ros2-jazzy` のコンテナ内でまとめてビルドします。
Gazebo のサンプルは Harmonic 用の名前（`gz-sim-*`、`gz.msgs`）で書いています。
Humble 環境の Gazebo Fortress 6.18 もこの名前を受け付け、箱とアームが動くことを確認済みです。

```bash
cd /workspace && colcon build --symlink-install && source install/setup.bash
```

### hello_py: Python の talker と listener

`chatter` トピックで文字列を1秒ごとに送受信する、最小の Python ノードです。
2つのターミナルで実行します。

```bash
ros2 run hello_py talker
ros2 run hello_py listener
```

### hello_gazebo: Gazebo で箱を落とす

Gazebo Harmonic で高さ 3 m から箱を落とし、その高さを ROS 2 のノードが表示します。
Gazebo の位置情報は `ros_gz_bridge` で `/model/box/odometry` トピックに変換します。

```bash
ros2 launch hello_gazebo hello_gazebo.launch.py
```

ブラウザの画面に Gazebo が開きます。起動直後は一時停止しているので、
左下の再生ボタンを押すと箱が落ち、ターミナルに `Box height` が表示されます。
ソフトウェア描画のため、画面が出るまで 30 秒ほどかかる場合があります。
画面なしで実行する場合は `headless:=true` を付けます。この場合はすぐに落下が始まります。

### hello_arm: Gazebo で3関節アームと5本指ハンドを動かす

`simple_arm.urdf` を元に、土台の旋回・肩・肘の3関節を持つアームの先に5本指のハンドを付けて
Gazebo に出します。`arm_mover` が各関節をサイン波で動かし、`hand_mover` が親指から順に
指を曲げて握り、一斉に開く動作を8秒周期で繰り返します。
モデルは `urdf/arm.urdf.xacro` で、各指は根元と先の2関節を持ち、同じマクロから作ります。

```bash
ros2 launch hello_arm hello_arm.launch.py
```

- 関節の目標角度は `/arm/joint1/cmd_pos` などのトピックに `std_msgs/Float64` [rad] で送ります。
  Gazebo の位置制御プラグインがその角度に追従させます。
- 指の目標角度は `/hand/<指>_<proximal|distal>/cmd_pos` に送ります。指は `thumb`・`index`・`middle`・
  `ring`・`little` で、0 が伸ばした状態、正の値で手のひら側へ曲がります（上限 1.57 rad）。
- 関節の現在角度は `/joint_states` に出ます。`arm_mover` と `hand_mover` が目標と実測を定期的に表示します。
- 手動で動かす場合は `auto_move:=false` を付けて起動し、別のターミナルから目標角度を送ります。

```bash
ros2 launch hello_arm hello_arm.launch.py auto_move:=false
ros2 topic pub --once /arm/joint2/cmd_pos std_msgs/msg/Float64 "{data: 0.8}"
ros2 topic pub --once /hand/index_proximal/cmd_pos std_msgs/msg/Float64 "{data: 1.2}"
```

アームは Gazebo がワールドを読み込み終えてから生成されます。
ソフトウェア描画のため、画面にアームが出るまで 30 秒ほどかかる場合があります。
`headless:=true` を付けると画面なしで実行できます。

## ROS 2 パッケージの開発

新しいパッケージは `src` の下に作成します。コンテナ内のターミナルで実行する例:

```bash
cd /workspace/src
ros2 pkg create --build-type ament_python hello_ros2_nodes
cd /workspace
rosdep update
apt-get update
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
colcon build --symlink-install
source /workspace/install/setup.bash
```

新しい対話シェルでは ROS 2・開発ワークスペース（Humble では Tesseract も）の環境を自動で読み込みます。
既に開いているシェルにはビルド後に `source /workspace/install/setup.bash` が必要です。
対話シェル以外で ROS コマンドを実行する場合は
`docker compose exec ros2-dev /usr/local/bin/ros2-dev-entrypoint ros2 pkg list` のように
エントリポイントを経由してください。

ROS 通信は同じコンテナ内で完結する設定です。別のコンテナや実機と通信する場合は、
Docker のネットワーク設定に加えて、Humble では `ROS_LOCALHOST_ONLY`、
Jazzy では `ROS_AUTOMATIC_DISCOVERY_RANGE` を `compose.yml` で調整する必要があります。

## 管理

```bash
docker compose logs -f ros2-dev
docker compose stop
docker compose start
docker compose down
```

`down` でもビルド成果物は保存されます。成果物を削除してやり直すときだけ
`docker compose down -v` を使ってください。

ポートが競合する場合は `.env.example` を `.env` にコピーして `NOVNC_PORT` を変更します。
`ROS_DOMAIN_ID` も同じファイルで設定できます。設定変更後は
`docker compose up -d` を実行します。
