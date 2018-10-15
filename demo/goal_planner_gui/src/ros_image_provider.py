import rospy

from collections import OrderedDict

from PyQt5.QtCore import QObject, Q_ARG, Q_RETURN_ARG
from PyQt5.QtCore import QSize
from PyQt5.QtCore import QVariant
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtGui import QImage
from PyQt5.QtQuick import QQuickImageProvider
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

from planner_control_msgs.srv import AddRemTopic, AddRemTopicResponse
from planner_control_msgs.srv import ChangeFixationpointColor
from planner_control_msgs.srv import ChangeFixationpointColorResponse
from planner_control_msgs.srv import ChangeFixationpointDiameter
from planner_control_msgs.srv import ChangeFixationpointDiameterResponse
from planner_control_msgs.srv import ChangeFixationpointRingThickness
from planner_control_msgs.srv import ChangeFixationpointRingThicknessResponse
from planner_control_msgs.srv import QueryTopic, QueryTopicResponse
from planner_control_msgs.srv import ShowIndex
from planner_control_msgs.srv import ShowIndexResponse
from planner_control_msgs.srv import ShowTopic, ShowTopicResponse


class RosImageProvider(QQuickImageProvider):
    def __init__(self, cli_args,  *__args):
        super(RosImageProvider, self).__init__(QQuickImageProvider.Image,
                                               *__args)

        self.ros_topic_dict = OrderedDict()
        self.active_subscriber = None
        self.qimage = QImage(1, 1, QImage.Format_RGB888)
        self.qimage.fill(int('0xFFFFFF', 16))
        self.widget = None
        self.qml_image = None
        self.qml_fixationpoint = None

        self.ros_topic_dict['control_view'] = 'default'

        if cli_args.image_topic is not None:
            for i in cli_args.image_topic:
                if not i.startswith("__"):
                    self.ros_topic_dict[i] = 'Image'

        if cli_args.cimage_topic is not None:
            for i in cli_args.cimage_topic:
                if not i.startswith("__"):
                    self.ros_topic_dict[i] = 'CompressedImage'

        self.list_topics_srv = rospy.Service('goal_planner_gui/list_topic',
                                             QueryTopic,
                                             self._list_topic)
        self.add_itopic_srv = rospy.Service('goal_planner_gui/add_itopic',
                                            AddRemTopic,
                                            self._add_itopic)
        self.add_citopic_srv = rospy.Service('goal_planner_gui/add_citopic',
                                             AddRemTopic,
                                             self._add_citopic)
        self.rem_topic_srv = rospy.Service('goal_planner_gui/rem_topic',
                                           AddRemTopic,
                                           self._rem_topic)
        self.show_by_topic_srv = rospy.Service(
            'goal_planner_gui/show_by_topic', ShowTopic, self.show_by_topic
        )
        self.show_by_index_srv = rospy.Service(
            'goal_planner_gui/show_by_index', ShowIndex, self.show_by_index
        )

        self.change_fixationpoint_color_srv = rospy.Service(
            'goal_planner_gui/change_fixationpoint_color', ChangeFixationpointColor,
            self.change_fixationpoint_color
        )

        self.change_fixationpoint_midpoint_diameter_srv = rospy.Service(
            'goal_planner_gui/change_fixationpoint_midpoint_diameter', ChangeFixationpointDiameter,
            self.change_fixationpoint_midpoint_diameter
        )

        self.change_fixationpoint_ring_diameter_srv = rospy.Service(
            'goal_planner_gui/change_fixationpoint_ring_diameter', ChangeFixationpointDiameter,
            self.change_fixationpoint_ring_diameter
        )

        self.change_fixationpoint_ring_thickness_srv = rospy.Service(
            'goal_planner_gui/change_fixationpoint_ring_thickness', ChangeFixationpointRingThickness,
            self.change_fixationpoint_ring_thickness
        )

    def connect_gui(self, widget):
        self.widget = widget
        self.qml_image = self.widget.view.rootObject().findChild(
            QObject, "video_widget"
        )
        self.qml_fixationpoint = self.widget.view.rootObject().findChild(
            QObject, 'fixationpoint'
        )

    def change_fixationpoint_color(self, color):
        # for x in QColor.colorNames():
        #     print(x)
        if any(x == color.Color for x in QColor.colorNames()):
            meta = self.qml_fixationpoint.metaObject()
            res = meta.invokeMethod(
                self.qml_fixationpoint, "change_fixationpoint_color",
                Qt.BlockingQueuedConnection, Q_RETURN_ARG(QVariant),
                Q_ARG(QVariant, color.Color)
            )

            return ChangeFixationpointColorResponse('Success.')
        else:
            return ChangeFixationpointColorResponse(
                'Specified color "{}" is not contained in'
                ' "QColor.colorNames()".'.format(color.Color) +
                'See "http://doc.qt.io/qt-5/qcolor.html" for available colors.'
            )

    def change_fixationpoint_midpoint_diameter(self, diameter):
        if int(diameter.Diameter) < 0:
            return ChangeFixationpointDiameterResponse('Failure. Only positive values are allowed.')
        else:
            meta = self.qml_fixationpoint.metaObject()
            res = meta.invokeMethod(
                self.qml_fixationpoint, "change_fixationpoint_midpoint_diameter",
                Qt.BlockingQueuedConnection, Q_RETURN_ARG(QVariant),
                Q_ARG(QVariant, diameter.Diameter)
            )

            return ChangeFixationpointDiameterResponse('Success.')

    def change_fixationpoint_ring_diameter(self, diameter):
        if int(diameter.Diameter) < 0:
            return ChangeFixationpointDiameterResponse('Failure. Only positive values are allowed.')
        else:
            meta = self.qml_fixationpoint.metaObject()
            res = meta.invokeMethod(
                self.qml_fixationpoint, "change_fixationpoint_ring_diameter",
                Qt.BlockingQueuedConnection, Q_RETURN_ARG(QVariant),
                Q_ARG(QVariant, diameter.Diameter)
            )

            return ChangeFixationpointDiameterResponse('Success.')

    def change_fixationpoint_ring_thickness(self, thickness):
        if int(thickness.Thickness) < 0:
            return ChangeFixationpointRingThicknessResponse('Failure. Only positive values are allowed.')
        else:
            meta = self.qml_fixationpoint.metaObject()
            res = meta.invokeMethod(
                self.qml_fixationpoint, "change_fixationpoint_ring_thickness",
                Qt.BlockingQueuedConnection, Q_RETURN_ARG(QVariant),
                Q_ARG(QVariant, thickness.Thickness)
            )

        return ChangeFixationpointRingThicknessResponse('Success.')

    def show_by_topic(self, topic):
        """Pass a topic name (or 'control_view') to switch the GUI view
        to that topic. Returns a string for success and failure.
        """
        
        if topic.Topic == 'control_view':
            if self.active_subscriber is not None:
                self.active_subscriber.unregister()
                self.active_subscriber = None
            self.set_gui_to_main_view()

            # clear qimage
            self.qimage.fill(int('0xFFFFFF', 16))
            meta = self.qml_image.metaObject()
            res = meta.invokeMethod(
                self.qml_image, 'call_image_provider',
                Qt.BlockingQueuedConnection
            )

            return ShowTopicResponse('Success.')
        else:
            try:
                value = self.ros_topic_dict[topic.Topic]
                if value == "Image":
                    if self.active_subscriber is not None:
                        self.active_subscriber.unregister()

                    self.active_subscriber = rospy.Subscriber(
                        topic.Topic, Image, self._fill_ibuffer,
                        queue_size=1
                    )
                    self.set_gui_to_video_view()

                    return ShowTopicResponse('Success.')
                else:
                    if self.active_subscriber is not None:
                        self.active_subscriber.unregister()

                    self.active_subscriber = rospy.Subscriber(
                        topic.Topic, CompressedImage, self._fill_cibuffer,
                        queue_size=1
                    )
                    self.set_gui_to_video_view()

                    return ShowTopicResponse('Success.')
            except KeyError:
                return ShowTopicResponse(
                    "Failure. The given topic name is not known."
                )

    def show_by_index(self, index):
        """Pass a dictionary index to switch the GUI view to that topic.
               Returns a string for success and failure.
        """

        number = int(index.Index)
        if number == 0:
            if self.active_subscriber is not None:
                self.active_subscriber.unregister()

            self.active_subscriber = None
            self.set_gui_to_main_view()

            # clear qimage
            self.qimage.fill(int('0xFFFFFF', 16))
            meta = self.qml_image.metaObject()
            res = meta.invokeMethod(
                self.qml_image, 'call_image_provider',
                Qt.BlockingQueuedConnection
            )

            return ShowIndexResponse('Success.')
        elif 0 < number < len(self.ros_topic_dict):
            pair = self.ros_topic_dict.items()[number]
            if pair[1] == "Image":
                if self.active_subscriber is not None:
                    self.active_subscriber.unregister()

                self.active_subscriber = rospy.Subscriber(
                    pair[0], Image, self._fill_ibuffer, queue_size=1
                )
                self.set_gui_to_video_view()

                return ShowIndexResponse('Success.')
            else:
                if self.active_subscriber is not None:
                    self.active_subscriber.unregister()

                self.active_subscriber = rospy.Subscriber(
                    pair[0], CompressedImage, self._fill_cibuffer,
                    queue_size=1
                )
                self.set_gui_to_video_view()

                return ShowIndexResponse('Success.')
        else:
            return ShowIndexResponse("Failure. The given index is out of"
                                     " bounds.")

    def requestImage(self, p_str, qsize):
        return self.qimage, QSize(self.qimage.width(), self.qimage.height())

    def _list_topic(self, image_topic_list):
        res_string = ""
        for key, value in self.ros_topic_dict.iteritems():
                res_string += key + ": " + value + "; "

        return QueryTopicResponse(res_string)

    def _add_itopic(self, add_rem_topic):
        if add_rem_topic.Topic == 'control_view':
            return AddRemTopicResponse('"control_view" can\'t be modified')

        self.ros_topic_dict[add_rem_topic.Topic] = 'Image'

        return AddRemTopicResponse('Success')

    def _add_citopic(self, add_rem_topic):
        if add_rem_topic.Topic == 'control_view':
            return AddRemTopicResponse('"control_view" can\'t be modified')

        self.ros_topic_dict[add_rem_topic.Topic] = 'CompressedImage'

        return AddRemTopicResponse('Success')

    def _rem_topic(self, add_rem_topic):
        if add_rem_topic.Topic == "control_view":
            return AddRemTopicResponse('Failure. "control_view" is not '
                                       'removable.')

        try:
            del self.ros_topic_dict[add_rem_topic.Topic]
        except KeyError:
            return AddRemTopicResponse('Failure. Specified topic is not in '
                                       'list')

        return AddRemTopicResponse('Success')

    def _fill_ibuffer(self, image):
        # Do error check on image format
        cv_bridge = CvBridge()
        try:
            np_array = cv_bridge.imgmsg_to_cv2(image, 'rgb8')
        except CvBridgeError as cbe:
            raise cbe

        self.qimage = QImage(np_array.data, np_array.shape[1],
                             np_array.shape[0],
                             QImage.Format_RGB888)

        meta = self.qml_image.metaObject()
        res = meta.invokeMethod(
            self.qml_image, 'call_image_provider',
            Qt.BlockingQueuedConnection
        )

    def _fill_cibuffer(self, compressed_image):
        # Do error check on image format
        cv_bridge = CvBridge()
        try:
            np_array = cv_bridge.compressed_imgmsg_to_cv2(
                compressed_image, 'rgb8'
            )
        except CvBridgeError as cbe:
            raise cbe

        self.qimage = QImage(np_array.data, np_array.shape[1],
                             np_array.shape[0],
                             QImage.Format_RGB888)

        meta = self.qml_image.metaObject()
        res = meta.invokeMethod(
            self.qml_image, 'call_image_provider',
            Qt.BlockingQueuedConnection
        )

    def set_gui_to_video_view(self):
        root_object = self.widget.view.rootObject()
        meta = root_object.metaObject()
        res = meta.invokeMethod(
            root_object,  "switch_to_video_view",
            Qt.BlockingQueuedConnection
        )

    def set_gui_to_main_view(self):
        root_object = self.widget.view.rootObject()
        meta = root_object.metaObject()
        res = meta.invokeMethod(
            root_object,  "switch_to_main_view",
            Qt.BlockingQueuedConnection
        )
