import os
import threading
import vision
from queue import Queue
import time
from KeyListener import KeyListener
import cv2





if __name__ == '__main__':
    #os.environ['BLINKA_FT232H'] = '1'
    listener = KeyListener(None)
    listener.start()
    vision.init_vision()

    while listener.running():
        cv2.waitKey(1)

    # vision.finish_vision()

