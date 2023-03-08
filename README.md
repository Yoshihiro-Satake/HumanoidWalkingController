# HumanoidWalkingController


https://user-images.githubusercontent.com/88638744/223683360-9450633f-318b-491a-8e81-8b9d53169d51.mp4


Capture pointを用いて軌道生成を行うChoreonoidのコントローラ．オンラインで軌道生成を行うことを目標としている．

運動パターン生成の理論は以下のEnglsberger氏の論文の内容をそのまま用いている．特にオリジナルの理論は無い．

https://ieeexplore.ieee.org/abstract/document/6943128?casa_token=V5CWzbbecTAAAAAA:DOjvnv26hT0y8XPRZMTDgL3FBQ8s5KOocWARBSkEwYb6j0qozZ1x9WDVOjdpBM48k0AjXWnLgA

上の論文のさらに元となった論文

https://ieeexplore.ieee.org/abstract/document/6094435?casa_token=NhdUgzJ94owAAAAA:XossL8QbJ3sxt6-fopm736Bu2oMekd1dyQqxlWygFwg83XQEq7JmXvmb0RseBT_NJKU0meyCZg

上2つの理論は重心軌道の計算法こそ同じだが，足をつく場所の計画の仕方が違う．
一番上の理論に関しては田崎先生の解説記事があるので，学生は以下を読んでからEnglsberger氏の論文を読むといいかもしれない．

https://www.jstage.jst.go.jp/article/jrsj/36/2/36_36_122/_article/-char/ja/

大事なことは
- Capture Pointはロボットの重心位置と速度から求まる床上の点．
- Capture PointはZMPに反発するように動き，重心はCapture Pointに追従するように動く．
- したがってCapture Pointの軌道を作れば重心とZMPの軌道が求まる．
以上の3つ．Capture Pointの軌道を求めることで重心とZMPの両方の軌道が得られるのが利点．
個人的には両脚支持期(Double Support Phase)を入れるのに非常に強力な手法だと感じた．

現状できていることは

- Capture pointを用いたSSP(Single Support Phase)のみの軌道生成
- 遊脚軌道生成（サイクロイド使用）
- Capture pointフィードバック
- 逆運動学
- 足首6軸力センサ読み取り

最低限2足歩行に必要な機能はあるため、適当な初期姿勢を与えれば歩く。

未完了の項目としては

- DSP(Double Support Phase)の実装
- フィルタ実装
- 足の姿勢の軌道生成(一軸回転法的な)

など

足の着地位置姿勢を6次元ベクトル(x,y,z,オイラー角)で事前に与える必要がある．

DSPの実装が楽であることが利点の1つなのに実装できていない(卒論では実装した)...

就活が終わったら趣味で再開するかもしれない
