# HumanoidWalkingController

Capture pointを用いて軌道生成を行うChoreonoidのコントローラ。オンラインで軌道生成を行うことを目標としている。

現状できていることは

- Capture pointを用いたSSP(Single Support Phase)のみの軌道生成
- 遊脚軌道生成
- Capture pointフィードバック
- 逆運動学
- センサ読み取り

最低限2足歩行に必要な機能はあるため、適当な初期姿勢を与えれば歩く。

未完了の項目としては

- DSP(Double Support Phase)の実装
- フィルタ実装
- ダンピング制御
- 足の姿勢の軌道生成(向きを変えたりとか)
  - これは一軸回転法を適用する予定 

など

足の着地位置姿勢を6次元ベクトル(x,y,z,オイラー角)で事前に与える必要がある。
