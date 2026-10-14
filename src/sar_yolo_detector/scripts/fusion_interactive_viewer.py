#!/usr/bin/env python3
"""Low-load Qt preview with original-frame click/ROI selection; no inference."""
import copy
import json
import math
import sys
import threading

import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
from PyQt5 import QtCore, QtGui, QtWidgets

from sar_yolo_detector.interactive_view import FramePair, image_point
from sar_yolo_detector.srv import SelectFusionROI, SelectFusionROIRequest


class Canvas(QtWidgets.QWidget):
    selected = QtCore.pyqtSignal(object, object, object)
    cancel = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setMinimumSize(320, 240)
        self.image = None
        self.header = None
        self.origin = None
        self.end = None
        self.frozen_header = None

    def set_frame(self, image, header):
        if self.origin is None:
            self.image, self.header = image, header
            self.update()

    def target_rect(self):
        if self.image is None:
            return QtCore.QRectF()
        scale = min(self.width()/self.image.width(), self.height()/self.image.height())
        w, h = self.image.width()*scale, self.image.height()*scale
        return QtCore.QRectF((self.width()-w)/2, (self.height()-h)/2, w, h)

    def map_point(self, point):
        if self.image is None:
            return None
        return image_point(point.x(), point.y(), self.width(), self.height(),
                           self.image.width(), self.image.height())

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtCore.Qt.black)
        if self.image is None:
            painter.setPen(QtCore.Qt.white)
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, 'Waiting for video + original frame metadata…')
            return
        painter.drawImage(self.target_rect(), self.image)
        if self.origin is not None and self.end is not None:
            rect = self.target_rect()
            sx, sy = rect.width()/self.image.width(), rect.height()/self.image.height()
            painter.setPen(QtGui.QPen(QtCore.Qt.yellow, 2, QtCore.Qt.DashLine))
            painter.drawRect(QtCore.QRectF(rect.left()+self.origin[0]*sx,
                                          rect.top()+self.origin[1]*sy,
                                          (self.end[0]-self.origin[0])*sx,
                                          (self.end[1]-self.origin[1])*sy).normalized())

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.RightButton:
            self.cancel_gesture()
            self.cancel.emit()
        elif event.button() == QtCore.Qt.LeftButton and self.header is not None:
            point = self.map_point(event.pos())
            if point is not None:
                self.origin = self.end = point
                self.frozen_header = copy.deepcopy(self.header)

    def mouseMoveEvent(self, event):
        if self.origin is not None:
            point = self.map_point(event.pos())
            if point is not None:
                self.end = point
                self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton and self.origin is not None:
            end = self.map_point(event.pos())
            if end is not None:
                self.selected.emit(self.frozen_header, self.origin, end)
            self.cancel_gesture()

    def cancel_gesture(self):
        self.origin = self.end = self.frozen_header = None
        self.update()


class Viewer(QtWidgets.QMainWindow):
    response = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle('SAR Fusion — click / drag to select')
        self.resize(850, 700)
        self.frames = FramePair()
        self.bridge = CvBridge()
        self.busy = False
        self.max_age = float(rospy.get_param('~selection_max_age_sec', .75))
        self.last_frame_wall = None
        self.canvas = Canvas()
        self.setCentralWidget(self.canvas)
        self.toolbar = self.addToolBar('Selection')
        self.toolbar.setMovable(False)
        cancel = self.toolbar.addAction('取消选择 / Cancel (Esc)')
        cancel.triggered.connect(self.clear)
        cancel.setShortcut(QtGui.QKeySequence('Esc'))
        self.state = QtWidgets.QLabel('  等待画面 / Waiting for video')
        self.toolbar.addWidget(self.state)
        self.statusBar().showMessage('左键点击目标；拖框选择区域；右键或 Esc 取消。拖框时仅冻结显示，不暂停算法。')
        self.response.connect(self.statusBar().showMessage)
        self.response.connect(self.command_finished)
        self.canvas.selected.connect(self.select)
        self.canvas.cancel.connect(self.clear)
        self.click = rospy.Publisher(rospy.get_param('~select_pixel_topic'), PointStamped, queue_size=1)
        self.roi = rospy.ServiceProxy(rospy.get_param('~select_roi_service'), SelectFusionROI)
        self.image_sub = rospy.Subscriber(rospy.get_param('~image_topic'), Image,
                                          self.on_image, queue_size=1, buff_size=4*1024*1024)
        self.evidence_sub = rospy.Subscriber(rospy.get_param('~evidence_topic'), String,
                                             self.on_evidence, queue_size=1)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(80)

    def on_image(self, message):
        # Bound image memory even if an incorrectly configured source is huge.
        if len(message.data) <= 16*1024*1024:
            self.frames.add_image(message)

    def on_evidence(self, message):
        try:
            if len(message.data) <= 65536:
                self.frames.add_evidence(json.loads(message.data))
        except (KeyError, TypeError, ValueError):
            rospy.logwarn_throttle(5., 'Viewer received invalid original-frame metadata')

    def tick(self):
        if rospy.is_shutdown():
            self.close()
            return
        pair = self.frames.take()
        if pair is None:
            if self.last_frame_wall is not None and QtCore.QDateTime.currentMSecsSinceEpoch()-self.last_frame_wall > 2000:
                self.state.setText('  视频中断 / Video stale')
            return
        message, evidence = pair
        try:
            rgb = self.bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
            rgb = rgb.copy(order='C')
            image = QtGui.QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QtGui.QImage.Format_RGB888).copy()
            original = evidence['capture_header']
            header = Header(seq=int(original['seq']), frame_id=original['frame_id'],
                            stamp=rospy.Time(int(original['stamp']['secs']), int(original['stamp']['nsecs'])))
            self.canvas.set_frame(image, header)
            self.last_frame_wall = QtCore.QDateTime.currentMSecsSinceEpoch()
            identity = int(evidence.get('selected_id', -1))
            status = evidence.get('reason', '')
            self.state.setText('  {}×{} | ID {} | {}'.format(image.width(), image.height(), identity, status))
        except Exception as exc:
            rospy.logwarn_throttle(5., 'Viewer image conversion failed: %s', exc)

    def select(self, header, start, end):
        if self.busy:
            self.statusBar().showMessage('BUSY：等待上一条框选命令完成')
            return
        age = (rospy.Time.now()-header.stamp).to_sec()
        if header.stamp.is_zero() or not math.isfinite(age) or age > self.max_age or age < -.1:
            self.statusBar().showMessage('画面已过期：请在实时画面快速点击或拖框；未替换为其他帧。')
            return
        if math.hypot(end[0]-start[0], end[1]-start[1]) < 4:
            message = PointStamped(header=header)
            message.point.x, message.point.y = end
            self.click.publish(message)
            self.statusBar().showMessage('点击已发送（原图 {:.0f}, {:.0f}）；请以顶部 ID 和绿色框确认选中结果。'.format(*end))
            return
        x, y = int(min(start[0], end[0])), int(min(start[1], end[1]))
        right, bottom = int(max(start[0], end[0])), int(max(start[1], end[1]))
        if right-x < 4 or bottom-y < 4:
            self.statusBar().showMessage('框选区域不得小于 4×4 原图像素')
            return
        request = SelectFusionROIRequest(header=header)
        request.roi.x_offset, request.roi.y_offset = x, y
        request.roi.width, request.roi.height = right-x, bottom-y
        self.submit(request)

    def clear(self):
        self.canvas.cancel_gesture()
        if not self.busy:
            self.submit(SelectFusionROIRequest(clear_selection=True))
        else:
            self.statusBar().showMessage('BUSY：请待上一条命令完成后取消')

    def submit(self, request):
        self.busy = True
        self.statusBar().showMessage('正在提交取消/框选命令…')
        def run():
            try:
                self.roi.wait_for_service(timeout=2.)
                result = self.roi(request)
                text = '{} | ID {} | {}'.format('成功' if result.success else '拒绝', result.selected_target_id, result.message)
            except (rospy.ROSException, rospy.ServiceException) as exc:
                text = '选择接口不可用：{}'.format(exc)
            self.response.emit(text)
        # One bounded in-flight command; never stall Qt on inference/service locks.
        self.worker = threading.Thread(target=run, daemon=True)
        self.worker.start()

    @QtCore.pyqtSlot(str)
    def command_finished(self, text):
        self.busy = False

    def closeEvent(self, event):
        self.timer.stop()
        self.image_sub.unregister()
        self.evidence_sub.unregister()
        self.click.unregister()
        rospy.signal_shutdown('Interactive viewer closed')
        event.accept()


def main():
    rospy.init_node('fusion_interactive_viewer', disable_signals=True)
    app = QtWidgets.QApplication(rospy.myargv(argv=sys.argv))
    window = Viewer()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
