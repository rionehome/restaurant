# rest_here_you_are
## Overview
お客さんに商品を持って行った時の発話。

## Description
* main.py：　メインの処理

* `/rest_here_you_are/src/log/`：　発話の文のログが書き込まれます。

* `/rest_here_you_are/src/dictionary/`：　単語辞書と文法辞書があります。

* `/rest_here_you_are/src/beep/`：　ビープ音のwavファイルがあります。

## Usage

```
roslaunch rest_here_you_are rest_here_you_are.launch
```

## Node
**`name` rest_here_you_are_main**

### Subscribe Topic
* **`/restaurant/activate`** テーブルに着いたメッセージを受け取る （ rest_start_node.msg/Activate ）

### Publish Topic
* **`/restaurant/activate`** お客さんが商品を受け取った ( rest_start_node.msg/Activate )



