# rest_nlp_finish
## Overview
レストラン終了の音声認識と発話のパッケージです。

## Description
* main.py：　メインの処理

* `/rest_nlp_finish/src/log/`：　発話の文のログが書き込まれます。

* `/rest_nlp_finish/src/dictionary/`：　単語辞書と文法辞書があります。

* `/rest_nlp_finish/src/beep/`：　ビープ音のwavファイルがあります。

## Usage

```
roslaunch rest_nlp_finish rest_nlp_finish.launch
```

「wait]か「stop」か「stop the test」を認識させる。

## Node
**`name` rest_nlp_finish_main**

### Subscribe Topic
* **`/restaurant/activate`** テーブルに着いたメッセージを受け取る （ rest_start_node.msg/Activate ）

### Publish Topic
* **`/restaurant/activate`** お客さんが商品を受け取った ( rest_start_node.msg/Activate )
