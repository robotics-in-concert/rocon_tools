#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import sys
import rospy
import rocon_python_comms
import rocon_python_utils
import rocon_console.console as console
import rocon_std_msgs.msg as rocon_std_msgs

##############################################################################
# Qt Classes
##############################################################################

qt_available = True

try:
    import signal
    from PyQt4 import QtGui, QtCore

    class Window(QtGui.QWidget):

        def __init__(self, name, description, version, icon):
            super(Window, self).__init__()
            self._description = description
            self._icon = icon
            self._name = name
            self._version = version
            self.initUI()

        def initUI(self):
            pixmap = QtGui.QPixmap(self._icon)
            self.pic = QtGui.QLabel(self)
            self.pic.setPixmap(pixmap)
            self.pic.resize(pixmap.width(), pixmap.height())
            #self.pic.setGeometry(10, 10, pixmap.width(), pixmap.height())

            self.text = QtGui.QLabel(self)
            self.text.resize(200, pixmap.height())
            self.text.move(pixmap.width() + 10, 0)
            self.text.setText("<b>Name:</b> %s<br/><b>Description:</b> %s<br/><b>Rocon Version:</b> %s" % (self._name, self._version, self._description))
            self.text.setWordWrap(True)
            self.text.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)

            self.setWindowTitle(self._name)
            self.setWindowIcon(QtGui.QIcon(self._icon))
            self.putUnderMouse()

        def putUnderMouse(self):
            mouse = QtGui.QCursor.pos()
            self.move(mouse.x() - (210 + self.pic.geometry().width()) / 2,
                        mouse.y() - (self.pic.geometry().height()) / 2)

except ImportError:
    qt_available = False


##############################################################################
# Main
##############################################################################

def main(node_name='master_info', title='Master Information'):
    '''
      Establishes a connection and prints master information to the console.
    '''

    rospy.init_node(node_name)
    try:
        topic_name = rocon_python_comms.find_topic('rocon_std_msgs/MasterInfo', timeout=rospy.rostime.Duration(5.0), unique=True)
    except rocon_python_comms.NotFoundException as e:
        print(console.red + "failed to find unique topic of type 'rocon_std_msgs/MasterInfo' [%s]" % str(e) + console.reset)
        sys.exit(1)

    master_info_proxy = rocon_python_comms.SubscriberProxy(topic_name, rocon_std_msgs.MasterInfo)
    try:
        master_info_proxy.wait_for_publishers()
    except rospy.exceptions.ROSInterruptException:
        rospy.logwarn("Concert Info : ros shut down before concert info could be found.")

    master_info = rocon_std_msgs.MasterInfo()
    while not rospy.is_shutdown():
        result = master_info_proxy(rospy.Duration(0.2))
        if result:
            master_info = result
            break
        rospy.rostime.wallsleep(1.0)  # human time

    console.pretty_println(title, console.bold)
    print(console.cyan + "  Name       : " + console.yellow + master_info.name + console.reset)
    print(console.cyan + "  Description: " + console.yellow + master_info.description + console.reset)
    print(console.cyan + "  Icon       : " + console.yellow + master_info.icon.resource_name + console.reset)
    print(console.cyan + "  Version    : " + console.yellow + master_info.version + console.reset)

    if qt_available:
        icon = rocon_python_utils.ros.find_resource_from_string(master_info.icon.resource_name)
        signal.signal(signal.SIGINT, signal.SIG_DFL)  # make sure this comes after the rospy call, otherwise it will handle signals.
        app = QtGui.QApplication(sys.argv)
        window = Window(master_info.name, master_info.description, master_info.version, icon)
        window.show()
        sys.exit(app.exec_())
