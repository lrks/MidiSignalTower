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
MIDIメッセージに反応します。

## デモ
YouTubeに上げました。

[![シグナルタワーで「Hexagon Force」の演奏](http://img.youtube.com/vi/wn7WAOdVWSI/0.jpg)](http://www.youtube.com/watch?v=wn7WAOdVWSI)

