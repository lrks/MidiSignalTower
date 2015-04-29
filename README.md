# MidiSignalTower
シグナルタワーを改造し、MIDI音源化しました。3和音まで出せます。
![Piano](https://raw.githubusercontent.com/lrks/MidiSignalTower/master/sample.gif "電子ピアノとの接続")

## ハードウェア
以下のものを組み合わせて作っています。

  * PATLITE LME系シグナルタワー
  * LPC1114
  * YAMAHA YMZ294

シグナルタワーの改造箇所は以下です。

  * シグナルタワー内部のブザーを鳴らすための発振回路をバイパスする
  * ランプを自由に制御できるようにする

この上で、ランプ、YMZ294経由のブザー、YMZ294そのものをLPC1114で制御しています。シフトレジスタをケチったら、GPIOが足りなくなりそうで焦りました。


## ソフトウェア
このリポジトリ内に入っています。

ピン変化割り込みを使い、

   * プリセット曲
   * ボゴソート(https://www.youtube.com/watch?v=kPRA0W1kECg)
   * クイックソート(同上)
 
を演奏する「デモ機能」というものも入れたつもりですが、動かなかったので、タイマ割り込みでポーリングみたいなことをしていました。

しかし、間隔が短いとMIDI受信で取りこぼしが起こるようになり、間隔を伸ばしたら、動作しなくなってしまいました。おしまい。


## デモ
YouTubeに上げました。

[![シグナルタワーで「Hexagon Force」の演奏](http://img.youtube.com/vi/wn7WAOdVWSI/0.jpg)](http://www.youtube.com/watch?v=wn7WAOdVWSI)


## 今後の予定
余裕があれば、今後はこんなことをします。

  * 回路図の公開
  * デモ機能をなんとかする
