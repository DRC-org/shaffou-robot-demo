# shaffou-robot-demo

しゃっふぉうロボット（r-θ-z アーム）の展示用ファームウェアです。PS5 コントローラで操作します。

| 基板 | マイコン | 役割 | PlatformIO 環境 |
|---|---|---|---|
| CWMC（中央制御） | ESP32-DevKitC | PS5 の受信、指令の計算、LCD 表示、CAN 送信 | `cwmc` |
| DCMD ×3 | UNO R4 Minima ベース | r / θ / z の DC モーター駆動 | `dcmd` |
| SVMD | UNO R4 Minima ベース | エンドエフェクタ回転（SV0）・把持（SV1） | `svmd` |

## 書き込み

VS Code の PlatformIO 拡張なら、下のバーで環境（`env:cwmc` など）を選んで Upload します。CLI の場合は次のとおりです。

```bash
pio run -e cwmc -t upload
```

```bash
pio run -e dcmd -t upload
```

```bash
pio run -e svmd -t upload
```

DCMD は 3 枚とも同じファームです。軸は DIP スイッチで決めます（SW0 が bit0、ON が 1）。

| 軸 | DIP の値 | CAN ID |
|---|---|---|
| r | 0（全部 OFF） | 0x100 |
| θ | 1（SW0 のみ ON） | 0x101 |
| z | 2（SW1 のみ ON） | 0x102 |

DIP スイッチは起動時に 1 回だけ読みます。変えたらリセットしてください。

## 初回の準備

### 1. PS5 コントローラの MAC アドレスを調べて設定する

ライブラリ（ps5-esp32）は、コントローラ本体の Bluetooth MAC アドレスに ESP32 から接続しに行きます。

1. コントローラの「クリエイト」ボタンと PS ボタンを同時に長押しし、ライトバーが速く点滅するペアリングモードにします。
2. PC かスマホと Bluetooth でペアリングし、アドレスを確認します。
   - Windows：コントロールパネル → デバイスとプリンター → 「Wireless Controller」を右クリック → プロパティ → 「Bluetooth」タブの「一意の識別子」
   - macOS：システム情報 → Bluetooth → Wireless Controller の「アドレス」
   - Android：設定 → 接続済みのデバイス → Wireless Controller の歯車アイコン（下のほうにアドレスが出ます）
   - Linux：`bluetoothctl devices`
3. 確認できたら、PC やスマホ側でペアリングを解除します。残っていると、コントローラがそちらに接続しに行ってしまいます。
4. [src/cwmc/config.h](src/cwmc/config.h) の `kPs5Mac` を書き換えて、CWMC に書き込みます。

```cpp
constexpr const char* kPs5Mac = "AA:BB:CC:DD:EE:FF";
```

接続できないときは、CWMC を起動した状態でコントローラをペアリングモード（クリエイト＋PS 長押し）にしてみてください。

### 2. 非常停止（REF）の向きを確かめる

DCMD は REF ピンの値で非常停止を判定します。初期設定は「LOW = 非常停止中」です。

1. DCMD を PC につなぎ、シリアルモニタ（115200 bps）を開きます。
2. 非常停止ボタンを押したときと戻したときで、`REF=0` と `REF=1` のどちらになるかを見ます。
3. 押したときに `REF=1` になる場合は、[src/dcmd/main.cpp](src/dcmd/main.cpp) の `kRefEstopLevel` を `HIGH` に変えます。

非常停止中は RGB LED が赤になり、CWMC の LCD に `!!E-STOP (REF)!!` と表示されます。

### 3. 実機で調整する項目

| 項目 | 場所 | 初期値 |
|---|---|---|
| 各軸の回転方向 | `src/cwmc/config.h` の `kAxisDir` | すべて +1 |
| 最大 duty（低速 / 高速） | `kDutyLow` / `kDutyHigh` | 30% / 50% |
| duty の変化の速さ | `kDutyRampPerSec` | 150 %/s |
| 把持の開 / 閉の角度 | `kGripOpenDeg` / `kGripCloseDeg` | 30° / 120° |
| 回転サーボの範囲・初期角度 | `kRotateMinDeg` / `kRotateMaxDeg` / `kRotateInitDeg` | 0° / 180° / 90° |
| サーボのパルス幅 | `src/svmd/main.cpp` の `kPulseMinUs` / `kPulseMaxUs` | 500 / 2400 µs |
| DCMD 側の duty 上限 | `src/dcmd/main.cpp` の `kMaxDutyHard` | 80% |

サーボの端でジーッと鳴り続ける（端に押し付けている）場合は、パルス幅の範囲を狭めてください。

## 操作方法

| 入力 | 動作 |
|---|---|
| 左スティック 上下 | r（伸縮） |
| 左スティック 左右 | θ（旋回） |
| 右スティック 上下 | z（上下） |
| 右スティック 左右 | エンドエフェクタ回転（倒している間だけ回る） |
| R1 / L1 | 把持を閉じる / 開く（押している間だけ動く） |
| △ | 低速 / 高速の切り替え（起動時は低速） |
| × | 全軸停止（ソフト非常停止）。解除するまで動かない |
| OPTIONS | ソフト非常停止の解除 |

## LCD 表示

```text
PS5:OK LO MD3/3      PS5 の接続 / 速度モード / 応答のある DCMD の数
r+30 t-12 z+00       各軸の duty [%]（2 秒ごとに下と切り替え）
ROT 090 GRIP 030     サーボの角度 [deg]
```

停止中は 1 行目が次のようになります。

- `!!E-STOP (REF)!!`：非常停止ボタンが押されている
- `STOP  OPTIONS:GO`：× によるソフト非常停止中
- `CAN INIT ERROR`：CAN の初期化に失敗した

## 安全機能

- PS5 の接続が切れたら、全軸を即停止します。
- DCMD は、指令が 200 ms 途絶えると自分でモーターを止めます（CWMC は 20 ms ごとに送信）。
- 非常停止を解除したあと、DCMD は「duty 0」の指令を一度受けるまで動きません。スティックを倒したまま解除しても急に動き出さないためです。
- duty は CWMC 側で徐々に変化させます（停止だけは即時）。
- SVMD は、起動直後は信号を出さず、最初の指令を受けてから動き出します。指令が途絶えても角度を保持します（つかんだ物を落とさないため）。

## RGB LED（DCMD）

| 色 | 状態 |
|---|---|
| 緑 | 運転可 |
| 青 | 指令待ち（CWMC からの通信なし） |
| 黄 | 非常停止の解除後、duty 0 の指令待ち |
| 赤 | 非常停止中 |
| 赤点滅 | CAN の初期化に失敗 |

## CAN プロトコル

1 Mbps、標準 ID、データ長 8 バイトです。定義は [lib/protocol/protocol.h](lib/protocol/protocol.h) にあります。

| byte | 内容 |
|---|---|
| 0 | command |
| 1〜4 | value（int32、ビッグエンディアン） |
| 5〜7 | 予備（0） |

| ID | 向き | command | value |
|---|---|---|---|
| 0x100 + 軸 | CWMC → DCMD | 0x00：duty 設定 ／ 0x01：停止 | duty[%] × 100 |
| 0x180 + 軸 | DCMD → CWMC | 0x80：状態（100 ms 周期） | bit0 非常停止、bit1 指令途絶、bit2 再アーム待ち |
| 0x300 | CWMC → SVMD | チャンネル（0：回転、1：把持） | 角度[deg] × 10 |

## ディレクトリ構成

```text
platformio.ini         3 環境の定義
lib/protocol/          CAN プロトコル（全基板共通）
src/cwmc/              中央制御基板
src/dcmd/              DC モータドライバ基板
src/svmd/              サーボドライバ基板
```
