# restaurant_get_order
## Overview
  restaurantにおける,対話者からのメッセージ処理を行う.


## Description
rest.pyは対話者からのメッセージを処理行う。音声認識部分をgoogle_assistantを用いる.  
get_order.pyは回答者から受け取った文章に対して,オーダーを抜き出す。    rest_sphinx.pyは音声認識部分をpocketsphinxで行う.

## Usage
```
roslaunch restaurant_get_order rest.launch
```
上記のコマンドを実行し,`rest_ctrl`にString型のメッセージを投げると起動する。
```
roslaunch restaurant_get_order rest_sphinx.launch
```
音声認識にpocketsohinxを用いる場合は上のコマンド


## Node
**`name` restaurant_get_order**

### Subscribe Topic

* **`restaurant_getO`** メインノードの立ち上げ（ std_msgs/String ）

* **`yes_no/recognition_result`** yes/noの音声認識結果の受け取り（ std_msgs/String ）

* **`sound_system/recognition/result`** 音声認識結果の受け取り（ std_msgs/String ）

* **`restaurant_nlp/finish_speaking`** 発話終了を受け取る( std_msgs/Bool )

### Publish Topic

* **`restaurant_getO/resume/start`** 音声認識開始 ( std_msgs/String )

* **`yes_no/recognition_start`** yes/noの音声認識開始（ std_msgs/Bool ）

<!--* **`help_me_carry/send_place`** 場所情報の送信（ std_msgs/String )-->

* **`/restaurant_nlp/speak`** 文章の発話( std_msgs/String )




**`name` rest_sphinx.py**
### Subscribe Topic

* **`restaurant_getO`** メインノードの立ち上げ（ std_msgs/String ）

* **`yes_no/recognition_result`** yes/noの音声認識結果の受け取り（ std_msgs/String ）

* **`restaurant_getO/resume/result`** 音声認識結果の受け取り（ std_msgs/String ）

* **`restaurant_nlp/finish_speaking`** 発話終了を受け取る( std_msgs/Bool )

### Publish Topic

* **`restaurant_getO`** 音声認識開始 ( std_msgs/Bool )

* **`yes_no/recognition_start`** yes/noの音声認識開始（ std_msgs/Bool ）

<!--* **`help_me_carry/send_place`** 場所情報の送信（ std_msgs/String )-->

* **`/restaurant_nlp/speak`** 文章の発話( std_msgs/String )





**`name` speak.py**

### Subscribe Topic
* **`/restaurant/speak`** 発話するための文章を受け取る(std_msgs/Bool)

### Publish Topic
* **`restaurant_nlp/finish_speaking`** 発話終了を送信(std_msgs/String)
