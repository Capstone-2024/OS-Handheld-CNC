
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *


class ImageViewer(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.zoom_factor = 1.25

    def set_image(self, image_path):
        pixmap = QPixmap(image_path)
        self.scene.clear()
        self.scene.addPixmap(pixmap)
        self.setSceneRect(QRectF(pixmap.rect()))

    # 放大
    def zoom_in(self):
        self.scale(self.zoom_factor, self.zoom_factor)

    # 缩小
    def zoom_out(self):
        self.scale(1 / self.zoom_factor, 1 / self.zoom_factor)

    def zoom_to_coordinate(self, x, y, distance=50):
        # 显示的矩形基准宽度为50
        zoom_rect = QRectF(x - distance, y - distance, distance * 2, distance * 2)  # 以指定坐标为中心，创建一个矩形区域
        self.fitInView(zoom_rect, Qt.KeepAspectRatio)

    def reset_zoom(self):
        self.resetTransform()
