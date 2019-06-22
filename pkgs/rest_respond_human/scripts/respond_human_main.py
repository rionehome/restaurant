# coding=utf-8
import rospy
from std_msgs.msg import String
from rest_start_node.msg import Activate


def activate_callback(activate):
	if activate.id == 0:
		# 声がする方向を探す
		find_sound.publish("start")
		# 手を振っている人間がいる方向を探す
		# find_human_image_method.publish("wave")
		# find_human_image.publish(True)
		pass


def find_sound_callback(result):
	print result.data


if __name__ == '__main__':
	rospy.Subscriber("/restaurant/activate", Activate, activate_callback)
	rospy.Subscriber("rest_find_sound/go_to_customer", String, find_sound_callback)
	next = rospy.Publisher("/restaurant/activate", Activate, queue_size=10)
	find_sound = rospy.Publisher("rest_find_sound/find_sound", String, queue_size=10)
	find_human_image = rospy.Publisher("/human_detection/start", String, queue_size=10)
	find_human_image_method = rospy.Publisher("/human_detection/method", String, queue_size=10)
