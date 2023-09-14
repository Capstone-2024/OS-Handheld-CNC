import cv2
from PIL import Image

# Need to have a definition of workpiece size (x mm, y mm)
# Then divide that by length/width of A4 paper, round up and then generate the marker image

def create_marker_board(columns, rows): # workpiece_x, workpiece_y
    # A4 Paper at 300 DPI
    image_x = 2480 # px
    image_y = 3508 # px
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    board = cv2.aruco.GridBoard((columns, rows), 0.04, 0.01, dictionary)
    image = board.generateImage((image_x, image_y), None, 10, 1)
    cv2.imshow("Aruco Board", image)
    cv2.imwrite("board.jpg", image)

    pdf = Image.fromarray(image)
    pdf.save("board.pdf", "PDF", resolution=100)
    
    cv2.waitKey(0)

if __name__ == "__main__": 
    create_marker_board(5, 7)