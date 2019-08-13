# Restaurant

コミットはdevelopブランチで！！！


## Usage
```
roslaunch restaurant restaurant.launch
````
## Subscribe Topic

* **`/natural_language_processing/start`**　（ std_msgs/String ）  
空文字をsubscribeすることで起動する.  
テーブルの左右の判定からcall_ducker開始まで.

* **`/natural_language_processing/restart_call_ducker`**　（ std_msgs/String ）  
空文字をsubscribeすることで起動する.  
call_duckerをやり直すためにキッチンに戻る.

* **`/natural_language_processing/get_order`**　（ std_msgs/String ）  
空文字をsubscribeすることで起動する.  
注文を聞く.テーブルの位置を登録する.

* **`/natural_language_processing/restart_get_order`**  (std_msgs/String)  
空文字をsubscribeすることで起動する.  
注文が聞き取れなかった場合にもう一度聞く.

* **`/natural_language_processing/finish_get_order`**  (std_msgs/String)   
空文字をsubscribeすることで起動する.  
オーダーを伝えるためにキッチンに移動, キッチンでオーダーを復唱しバーマンにロボットの上に商品を置いてもらう, テーブルに移動してお客さんに商品を渡す.

* **`/natural_language_processing/restart_here_you_are`**  (std_msgs/String)   
注文の商品名(（例)tea, tunaなど）をsubscribeすることで起動する.  
少し待機してから、お客さんに商品を取れたかどうかを、もう一度聞く.

* **`/natural_language_processing/finish_delivery`**  (std_msgs/String)  
空文字をsubscribeすることで起動する.  
商品を渡し終えたのでスタート位置に戻る.

* **`/call_ducker/finish`**  (std_msgs/Bool)  
TrueかFalseをsubscribeすることで起動する.  
       True: ロボットを呼んでいたお客さんかどうかの確認.  
       False: call_ducker失敗.

## Publish Topic
* **`/call_ducker/control`**  (std_msgs/String)  
call_ducker開始用のpublisher.  
開始時に"start"の文字列をpublishする.

## Service
* **`/rest_judge_bar/detect`**  （ sound_system/StringService )  
テーブルの左右の判定.


