# rest_here_you_are
## Overview
お客さんに商品を持って行った時の発話。

## Description
* main.py：　メインの処理

* `/rest_here_you_are/src/log/`：　発話の文のログが書き込まれます。

* `/rest_here_you_are/src/dictionary/`：　単語辞書と文法辞書があります。

## Usage

```
roslaunch rest_here_you_are rest_here_you_are.launch
```

## Node
**`name` rest_here_you_are_main**

### Subscribe Topic
* **`/rest_here_you_are/reach_customer`** Hotwordの受け取り （ rest_start_node.msg/Activate ）

### Publish Topic
* **`/rest_here_you_are/go_to_kitchen`** お客さんがいる方向の角度を送信 ( rest_start_node.msg/Activate )



