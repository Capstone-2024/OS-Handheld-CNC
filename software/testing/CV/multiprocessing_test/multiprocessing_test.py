import cv2
import multiprocessing as mp
import time

def grab_frames(queue):
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        queue.put(frame)
    

def display_frames(queue, fps):
    cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
    start_time = time.time()
    while True:
        frame = queue.get()
        fps = 1.0 / (time.time() - start_time)
        print("FPS: ", fps)
        start_time = time.time()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    queue = mp.Queue()
    fps = mp.Value('d', 0.0)
    p1 = mp.Process(target=grab_frames, args=(queue))
    p2 = mp.Process(target=display_frames, args=(queue, fps))
    p1.start()
    p2.start()
    p1.join()
    p2.join()

