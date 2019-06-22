# coding=utf-8
import rospy
import time
from std_msgs.msg import String, Float64MultiArray, Int32
from rest_start_node.msg import Activate

move_status = "stop"


def pub_move(angle):
	array = Float64MultiArray()
	array.data.append(0)
	array.data.append(0)
	array.data.append(angle)
	array.data.append(1)
	move.publish(array)


def activate_callback(activate):
	if activate.id == 0:
		# 声がする方向を探す
		find_sound.publish("start")
		# 手を振っている人間がいる方向を探す
		# find_human_image_method.publish("wave")
		# find_human_image.publish(True)
		pass


def find_sound_callback(result):
	global move_status
	# 移動
	print result.data
	pub_move(int(result.data))
	time.sleep(1)
	move_status = "turn"

	while move_status == "turn":
		time.sleep(0.1)

	print "finish"
	next = Activate()
	next.id = 1
	pub_next.publish(next)


def move_signal_callback(signal):
	global move_status
	if signal.data == 0 and move_status == "turn":
		move_status = "stop"


if __name__ == '__main__':
	rospy.init_node("rest_respond_human")

	rospy.Subscriber("/restaurant/activate", Activate, activate_callback)
	rospy.Subscriber("rest_find_sound/go_to_customer", String, find_sound_callback)
	rospy.Subscriber("/move/amount/signal", Int32, move_signal_callback)
	pub_next = rospy.Publisher("/restaurant/activate", Activate, queue_size=10)
	find_sound = rospy.Publisher("rest_find_sound/find_sound", String, queue_size=10)
	find_human_image = rospy.Publisher("/human_detection/start", String, queue_size=10)
	find_human_image_method = rospy.Publisher("/human_detection/method", String, queue_size=10)
	move = rospy.Publisher("/move/amount", Float64MultiArray, queue_size=10)

	rospy.spin()
