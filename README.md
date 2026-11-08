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
│   ├── jazzy/Dockerfile        # Jazzy / Gazebo Harmonic / ros2_control / MoveIt 2 / MuJoCo の構築（ros2-jazzy）
│   ├── entrypoint.sh           # ROS 2 とワークスペースの環境読み込み
│   └── supervisord.conf        # 仮想画面、ウィンドウ管理、noVNC
└── src/
    ├── hello_py/               # Python の talker と listener
    ├── hello_gazebo/           # Gazebo で箱を落とすサンプル
    ├── hello_arm/              # Gazebo で3関節アームと5本指ハンドを動かすサンプル
    ├── agri_arm/               # 収穫用6軸アームのモデルと ros2_control（Jazzy 専用）
    ├── agri_arm_moveit_config/ # agri_arm の MoveIt 2 設定と収穫の動作計画デモ（Jazzy 専用）
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
ros2_control、gz_ros2_control、MoveIt 2、MuJoCo（Python パッケージ）も入れてあります。

検証記録（2026-11-07）：Gazebo Sim 8.15（Harmonic）で次を確認しました。

- 箱の自由落下は理論値との差が最大 1.2 mm で、高さ 0.500 m で静止。
- アームと5本指ハンドの自動動作。指の追従誤差は平均 0.011 rad。
- 手動指令の保持。肩 0.8 rad の指令に対して 0.8004 rad。
- 画面なしの実時間比は約 1.0、画面ありでは約 0.35〜0.55（ソフトウェア描画）。

## サンプルパッケージ

`src` には次のパッケージがあります。`ros2-jazzy` のコンテナ内でまとめてビルドします。
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

### agri_arm: 収穫用6軸アームを ros2_control で動かす

`docs/harvesting_robot_hardware_comparison.md` の暫定構成を元にした6軸アームです。
Jazzy 環境専用です（gz_ros2_control を使うため）。

| 関節 | 動き | 想定モジュール | URDF のトルク上限 |
|---|---|---|---|
| joint1 | 基部の旋回 | RMD-X6-P20-60 | 20 N·m（定格） |
| joint2 | 肩 | RMD-X8-P20-120 | 43 N·m（定格） |
| joint3 | 肘 | RMD-X6-P20-60 | 20 N·m（定格） |
| joint4 | 前腕の回転 | XM540-W270 | 6 N·m（仮の基準。最大 10.6 N·m） |
| joint5 | 手首の曲げ | XM540-W270 | 6 N·m（仮の基準） |
| joint6 | 手先の回転 | XM430-W350 | 2.5 N·m（仮の基準。12 V での最大 4.1 N·m） |

DYNAMIXEL の値は最大トルクより低く置いた仮の基準で、連続して出せるトルクの裏付けはありません。
手首の割合は実機の余裕を示すものではないので、連続運転の試験で置き換えます。

- 腕の長さは上腕 0.35 m、肘から手首 0.30 m、手首から手先 0.19 m です。
  リンクの質量と長さは最初の見積もりで、CAD と実部品が決まったら更新します。
- ハンド・カッター・カメラは、手先に付けた 0.65 kg の1つの塊として扱っています。
  果実や配線の重さは `payload:=0.35` のように手先へ追加できます（既定は 0 kg）。
- `joint_trajectory_controller` が `/arm_controller/follow_joint_trajectory` で軌道を受け取ります。
  関節の状態は `joint_state_broadcaster` が `/joint_states` に出し、トルク（effort）も含みます。
- `pose_cycle` が収穫を想定した姿勢を順に指令し、各姿勢で保持中のトルクと、
  上の表のトルクに対する割合を表示します。

```bash
ros2 launch agri_arm agri_arm_gazebo.launch.py
```

`headless:=true` で画面なし、`auto_move:=false` で自動動作なしになります。

検証記録（2026-11-08）：姿勢を保持中のトルクは、URDF の質量配置から手計算した静的トルクと
全関節で 0.02 N·m 以内で一致しました。腕を水平に伸ばした最悪の姿勢では次のとおりです。
手先の合計 1 kg は、資料にある「果実・配線込み約 1 kg」に相当します。

| 条件 | 肩 | 肘 | 手首の曲げ |
|---|---|---|---|
| 手先 0.65 kg（荷重なし） | 12.5 N·m（定格の29%） | 4.2 N·m（21%） | 0.94 N·m（仮の基準の16%） |
| 手先 1.0 kg（`payload:=0.35`） | 15.3 N·m（36%） | 5.9 N·m（29%） | 1.59 N·m（27%） |

ただし、Gazebo の位置指令は関節を目標角度へほぼ理想的に追従させるため、
加減速時のトルクやモータの応答遅れはまだ評価できていません。

#### トルク指令モードでの加減速の評価

`control_mode:=effort` にすると、各関節を PID でトルク指令し、加減速に必要なトルクも
シミュレーションに現れます（設定は `config/controllers_effort.yaml`）。
トルクは上の表の値で頭打ちにするので、速すぎる軌道は追従誤差として現れます。

```bash
ros2 launch agri_arm agri_arm_gazebo.launch.py control_mode:=effort auto_move:=false headless:=true
# 別の端末で
ros2 run agri_arm dynamics_test --ros-args -p use_sim_time:=true
```

`dynamics_test` は、負担の大きい4種類の動き（水平へ下ろす、水平から上げる、腕を伸ばしたまま
基部を ±1 rad 旋回、腕を伸ばしたまま手首を ±1.2 rad 曲げる）を、5次多項式の滑らかな軌道で行います。
最も速い関節が URDF の速度上限の 25%・50%・100% に達する3段階で動かし、
動作中のトルクの最大値、上の表のトルクに対する割合、追従誤差を表示します。

検証記録（2026-11-08）：速度上限 100% のときの最大トルクです（括弧内は上の表の値に対する割合）。

| 条件 | 肩（水平から上げる） | 肘 | 手首の曲げ | 基部の旋回 |
|---|---|---|---|---|
| 手先 0.65 kg | 14.8 N·m（34%） | 5.1 N·m（26%） | 1.2 N·m（20%） | 3.2 N·m（16%） |
| 手先 1.0 kg（`payload:=0.35`） | 18.2 N·m（42%） | 7.1 N·m（36%） | 2.0 N·m（33%） | 4.5 N·m（23%） |

- 静止時との差は、肩で約 2.3〜2.9 N·m でした。肩の最大角加速度は約 2.4 rad/s² です。
- 基部の旋回のトルクは、腕を伸ばしたときの慣性（約 0.7 kg·m²）× 角加速度 3.3 rad/s² と
  関節の粘性・摩擦から手計算した約 3.3 N·m と合っています。
- トルクが上限に張り付いた関節はありませんでした。動作中の最大追従誤差は肩で 0.013 rad
  （荷重ありで 0.016 rad）、基部の旋回で 0.017 rad（同 0.024 rad）です。

注意点：

- Gazebo にはモータの回転子の慣性（減速比の2乗で関節に効く）を入れる仕組みがありません。
  この影響は次の「MuJoCo で回転子の慣性を入れた確認」で評価しました。
- 腕を真上に伸ばした姿勢では、基部・前腕の回転・手先の回転の3軸が一直線に並びます。
  そのときの軸まわりの慣性は約 0.004 kg·m² と小さいため、これらの関節のゲインは低く抑えています。
  回転子の慣性がないことも、ゲインを上げられない原因です（MuJoCo で確認済み、次節）。
- 新しい軌道は、前の目標から補間を始めます（`interpolate_from_desired_state`）。
  測った角度から始めると、重力によるわずかなたわみが目標の段差になり、
  肩で重力トルクと同じくらいのトルクの跳ね（12 → 25 N·m）が起きていました。
- モータのトルクと回転数の関係や電流の制限は、まだ入れていません。

#### MuJoCo で回転子の慣性を入れた確認

減速機付きモータの回転子の慣性は、減速比の2乗倍になって関節に効きます。
Gazebo ではこれを入れられないため、同じ URDF を MuJoCo に読み込み、関節の armature として加えて確かめます。
Gazebo も ROS のノードも使わず、数秒で終わります。

```bash
ros2 run agri_arm mujoco_dynamics                 # 手先 0.65 kg
ros2 run agri_arm mujoco_dynamics --payload 0.35  # 手先 1.0 kg
```

`dynamics_test` と同じ4種類の動きについて、次の2つを行います。

- **逆動力学：** 軌道どおりに動かすのに必要なトルクを、回転子の慣性ありとなしで求めます。
  上の表のトルクに対する最大値と RMS、短時間の最大トルクに対する割合を表示します。
  上の表の値は、RMD では定格トルク、DYNAMIXEL では連続運転を想定した暫定値です。
- **順方向のシミュレーション：** `controllers_effort.yaml` のゲインと、基部・手首を硬くしたゲイン
  （`STIFF`）で、1 ms 周期の PID 制御を行います。最後に、真上の姿勢で各関節を 0.02 rad ずらして
  保持させ、安定するかを見ます。

回転子の慣性（関節側に換算した値）は次のとおりです。

| 関節 | モジュール | armature [kg·m²] | 根拠 |
|---|---|---|---|
| 基部・肘 | RMD-X6-P20-60 | 0.025 | 製品ページの Inertia 0.66 kg·cm² を回転子側とみなし、× 減速比 19.612² |
| 肩 | RMD-X8-P20-120 | 0.058 | 同じく 1.5 kg·cm² × 19.612² |
| 手首の回転・曲げ | XM540-W270 | 0.039 | 非公開。似た構成の MX-106 の同定値 0.0266（[Rhoban BAM](https://github.com/Rhoban/bam)）× (272.5/225)² |
| 手先の回転 | XM430-W350 | 0.037 | 非公開。MX-64 の同定値 0.0120 × (353.5/200)²。モータが小さいので多めの見積もり |

検証記録（2026-11-08、MuJoCo 3.17.0）：

- 慣性なしの結果は Gazebo とほぼ一致しました。例えば、肩を水平から上げる動きのシミュレーションでは、
  速度上限 100% で MuJoCo 14.6 N·m、Gazebo 14.8 N·m でした。両方のモデルが同じ動力学を表していると考えられます。
- **計画した軌道では、回転子の慣性によるトルクの増加はわずかです。** 速度上限 100% でも、
  増加は肩で 0.1〜0.2 N·m、手首の曲げで 0.1 N·m です。関節の最大角加速度は 2.4〜6.2 rad/s² と小さく、
  回転子の慣性分のトルクは「armature × 角加速度」で 0.1〜0.24 N·m にしかなりません。
  手先 1.0 kg でも、最も厳しい肩は定格の 42%（RMS 29%）、短時間の最大トルクの 15% でした。
  例えば XM540 で暫定値 6 N·m の 10% を回転子の慣性が占めるのは、角加速度が約 15 rad/s² を超えたときです。
- **制御の安定性には大きく効きます。** 真上の姿勢で硬いゲインにすると、慣性なしでは基部・前腕・手先が
  2.5〜7 rad/s で発振しました。Gazebo で見た暴走と同じ現象です。慣性ありでは同じゲインで安定しました。
  慣性を推定値の 30% まで下げても安定し、10% では発振しました。
- 硬いゲインでは追従誤差も小さくなりました。速度上限 100% のとき、基部の旋回で 0.017 → 0.004 rad、
  手首の曲げで 0.012 → 0.006 rad です。
- `controllers_effort.yaml` のゲインは Gazebo 用に低いまま残しています。Gazebo には回転子の慣性がないためです。

注意点：

- RMD の「Inertia」が回転子側の値かどうかは、製品ページに書かれていません。メーカーへの確認が必要です。
- DYNAMIXEL の値は、別型番の同定値を減速比で換算した推定です。実機のステップ応答から同定し直します。
- 減速機の効率、バックラッシ、モータのトルクと回転数の関係は、まだ入れていません。
  回転子の慣性は、衝突時や非常停止時の負荷にも効きます。今回は評価していません。

### agri_arm_moveit_config: MoveIt 2 で障害物を避けて果実へ向かう

agri_arm に MoveIt 2 をつなぎ、支柱を避ける軌道を計画して Gazebo で実行します。
Jazzy 環境専用です。

```bash
ros2 launch agri_arm_moveit_config harvest_moveit.launch.py
```

- Gazebo の世界 `worlds/harvest_world.sdf` には、太さ 4 cm・高さ 0.9 m の支柱と、
  その左右 0.2 m の果実（見た目だけで当たり判定なし）を置いています。
- `harvest_demo` は、同じ支柱と床を MoveIt の計画用シーンに入れ、
  ホーム → 左の果実 → 右の果実 → ホームの順に動かします。手先は果実の 4 cm 手前で止めます。
- 果実の位置と向き（手先を +x 方向へ向ける）から、当たり判定付きの逆運動学で関節角を求めます。
  初期値に肘を上げた自然な姿勢を与え、肩を後ろへ振り切るような解を避けています。
- その関節角へ向かう軌道を OMPL（RRTConnect）で計画し、`arm_controller` で実行します。
- 各動作について、計画時間、関節角を直線的に補間した場合に支柱へ当たるか、
  実行中に測った関節角が当たり判定に引っかかったか、手先の位置誤差を表示します。
- RViz の MotionPlanning パネルから、目標姿勢をドラッグして手動で計画・実行もできます。
  手動で使うときは `demo:=false` にします。このときも支柱と床は計画用シーンに登録されます。

`headless:=true` で Gazebo の画面なし、`rviz:=false` で RViz なし、`cycles:=3` で3周して終了、
`payload:=0.35` で手先に荷重を追加、`control_mode:=effort` でトルク指令になります。
トルク指令でも 2 周（6 動作）がすべて成功し、手先の誤差は 0.4〜0.8 mm でした。

設定の要点：

- `config/agri_arm.srdf` の当たり判定を省くリンクの組は、
  `moveit_setup_assistant` の `collisions_updater` で 20000 回の無作為な姿勢から求めました。
- `config/ompl_planning.yaml` で `longest_valid_segment_fraction` を 0.001 にしています。
  既定の 0.005 では当たり判定の間隔が数 cm になり、細い支柱を見落とした軌道ができて、
  最後の検証で弾かれて再計画になることがありました（8 動作中 1 回）。
- `config/joint_limits.yaml` の加速度上限はモータ試験前の控えめな仮の値です。

検証記録（2026-11-08）：画面なしで 5 周（15 動作）と、手先荷重 0.35 kg で 2 周を実行しました。

- すべての動作で計画と実行に成功し、再試行は不要でした。計画時間は 0.04〜0.11 s です。
- 左右の果実の間を関節角の直線補間で動くと、41 点中 11 点で支柱に当たります。
  計画した軌道は支柱を回り込み、実行中の関節角でも当たりはありませんでした。
- 果実手前の目標位置に対する手先の誤差は 0.3〜1.5 mm でした。
- 画面ありでも Gazebo と RViz の両方で同じ動きを確認しました。

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
