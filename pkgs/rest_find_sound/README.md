# rest_find_sound
## Overview
RestaurantのHotwordを検出したら音源定位で角度を取得して、大声で発話するパッケージです。その後、角度を送ります。

## Setup
sudo pip install pyusb

## Description
* start.py：　メインの処理

* getting_array.py:　音源定位

* `/rest_find_sound/src/log/`：　発話の文のログが書き込まれます。

* `/rest_find_sound/src/dictionary/`：　単語辞書と文法辞書があります。

* `/rest_find_sound/src/beep/`：　wavファイルがあります。

## Usage

```
roslaunch rest_find_sound rest_find_sound.launch
```

## Node
**`name` rest_find_sound_start**

### Subscribe Topic
* **`/restaurant/activate`** 音声認識待機の合図の受け取り （ rest_start_node.msg/Activate ）

### Publish Topic
* **`/restaurant/activate`** お客さんがいる方向の角度を送信 ( rest_start_node.msg/Activate )



