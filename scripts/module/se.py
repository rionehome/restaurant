import rospkg
import subprocess


class SE:
    def __init__(self):
        self.se_path = "{}/etc/SE/".format(rospkg.RosPack().get_path('restaurant'))
        self.DISCOVERY = self.se_path + "discovery.wav"
    
    @staticmethod
    def play(se):
        # type: (str) -> None
        subprocess.call(["aplay", se], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
