import os
import threading
import vision
from queue import Queue
import time
from KeyListener import KeyListener
import cv2



# def update_point(cap, window_name, listener):
#     global frame
#     if listener.running():
#         frame = vision.captqurar_imagen(cap)
#         vision.mostrar_frame(window_name)
#         threading.Timer(0.4, update_point, args=(cap, window_name, listener)).start()


if __name__ == '__main__':
    #os.environ['BLINKA_FT232H'] = '1'
    listener = KeyListener(None)
    listener.start()
    vision.init_vision()

    while listener.running():
        cv2.waitKey(1)

    # vision.finish_vision()

