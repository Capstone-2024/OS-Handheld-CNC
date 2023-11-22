
from PyQt5.QtCore import *


class SettingsConfig(QObject):

    def __init__(self):
        super(SettingsConfig, self).__init__()
        self.SPEED = '0.4m/s'
        self.BIT_TYPE = '8bits'

    def set_param(self, key, val):
        setattr(self, key, val)

    def get_param(self, key):
        value = getattr(self, key)
        return value
