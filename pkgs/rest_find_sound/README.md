# rest_find_sound
## Overview
RestaurantのHotword検出のメッセージを受け取ったら、音源定位で角度を取得して、大声で発話するパッケージです。その後、角度を送ります。

## Setup
sudo pip install pyusb

## Description
* start.py：　メインの処理

* `/rest_find_sound/src/log/`：　発話の文のログが書き込まれます。

* `/rest_find_sound/src/dictionary/`：　単語辞書と文法辞書があります。

## Usage

```
roslaunch rest_find_sound rest_find_sound.launch
```

## Node
**`name` rest_find_sound_start**

### Subscribe Topic
* **`rest_find_sound/find_sound`** Hotwordの受け取り （ std_msgs/String ）

### Publish Topic
* **`rest_find_sound/go_to_customer`** お客さんがいる方向の角度を送信 ( std_msgs/String )



