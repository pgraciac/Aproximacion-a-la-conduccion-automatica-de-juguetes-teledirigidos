import os
import vision
from KeyListener import KeyListener
from gpio import right_button, left_button, up_button, down_button
import cv2



# def update_point(cap, window_name, listener):
#     global frame
#     if listener.running():
#         frame = vision.captqurar_imagen(cap)
#         vision.mostrar_frame(window_name)
#         threading.Timer(0.4, update_point, args=(cap, window_name, listener)).start()


if __name__ == '__main__':
    #os.environ['BLINKA_FT232H'] = '1'
    

    warning = False
    listener = KeyListener(None)
    listener.start()
    vision.init_vision()
    # update_point(cap, window_name, listener)
    while listener.running():
        cv2.waitKey(1)
        # vision.mostrar_frame()
        # if vision.current_mark != None:
        #     mostrar_frame(window_name)

    vision.finish_vision()
    up_button.value = True
    down_button.value = True
    right_button.value = True
    left_button.value = True

