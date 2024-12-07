# klibrary
基本的なドキュメントは，サブモジュールのtutrcosに基準します．
そちらも参照してください．

## CubeMXでの設定の注意点
* クロック設定 
タブ：clock configurationから設定できる．
基本的に高めの設定が望ましい．あまり低いと，UARTなどの受信処理が追い付かずエラーになりプログラムが停止する場合がある．\
例(stm32f446ret6の場合)：設定画面上部右寄りのHCLK(MHz)を128に
* CAN
CANのボーレート周りの設定は下記サイトを利用して行ってください．
http://www.bittiming.can-wiki.info
プルダウンメニューから，「ST Microelectronics bxCAN」を選択し，Clock Rateとkbit/sを入力してください．\
f446reの場合Clock Rateは，CubeMXのタブ：clock configuration内のAPB1 peripheral clock(MHz)がCANに入力されているようです．
出力された表の行のどれか（accuracyが同じ場合はどれでもいい）を選んで，

| web site |  | CubeMX |
|---|---|---|
| Prescaler | → | Prescaler (for Time Quantum) |
| Seg 1 | → | Time Quanta in Bit Segment 1 |
| Seg 2 | → | Time Quanta in Bit Segment 2 |

のように設定してください．

* FreeRTOS
tutrcosのドキュメントのmain_threadを作成する要領で別のスレッドも作れます．