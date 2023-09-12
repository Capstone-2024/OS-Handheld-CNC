import cv2
import numpy as np

def offset(img): 
    last_color = 0
    left_change_index = []
    right_change_index = []

    for row in thresh: 
        i = 0 

        for item in row: 
            if i > 0:
                if item != last_color: 
                    if i >= img.shape[0]/2: 
                        right_change_index.append(i)
                    else:
                        left_change_index.append(i)
                            
                last_color = item
            
            i += 1
    return min(left_change_index)
    print(max(right_change_index))

img = cv2.imread("./raw/3/new.jpg")

gray = cv2.cvtColor(img ,cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (11,11), 3)
_, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY)

top_left = thresh.copy()[0:int(img.shape[0]/2), 0:int(img.shape[1]/2)]
bottom_left = thresh.copy()[0:int(img.shape[0]/2), int(img.shape[1]/2):img.shape[1]]
top_right = thresh.copy()[int(img.shape[0]/2):img.shape[0], 0:int(img.shape[1]/2)]
bottom_right = thresh.copy()[int(img.shape[0]/2):img.shape[0], int(img.shape[1]/2):img.shape[1]]

while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
    cv2.imshow("Frame", top_left)

print(offset(top_left))

cropped = top_left[0:top_left.shape[0], 137:top_left.shape[1]]

while cv2.waitKey(1) & 0xFF != ord('q'): # Press Q to quit
    cv2.imshow("Frame", cropped)