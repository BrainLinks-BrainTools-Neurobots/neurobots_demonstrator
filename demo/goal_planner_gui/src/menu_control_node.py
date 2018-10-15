#!/usr/bin/env python

import sys
import rospy
from planner_control_msgs.srv import menu_control

import logging
logger = logging.getLogger('root')

try:
    import tty, termios
except ImportError:
    # Probably Windows.
    try:
        import msvcrt
    except ImportError:
        # FIXME what to do on other platforms?
        # Just give up here.
        raise ImportError('getch not available')
    else:
        getch = msvcrt.getch
else:
    def getch():
        """getch() -> key character

        Read a single keypress from stdin and return the resulting character. 
        Nothing is echoed to the console. This call will block if a keypress 
        is not already available, but will not wait for Enter to be pressed. 

        If the pressed key was a modifier key, nothing will be detected; if
        it were a special function key, it may return the first character of
        of an escape sequence, leaving additional characters in the buffer.
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch



def control_move(keysignal):
    signal_dict = {1:"up", 2:"down", 3:"select", 4:"abort", 5:"abort/down-contextsensitive"}
    logger.debug("sending %s to server" % signal_dict[keysignal])
#    rospy.init_node('neurobots_menu_control')
    rospy.wait_for_service('menu_control_signal')    
    try:
        control_service = rospy.ServiceProxy('menu_control_signal', menu_control)
        assert keysignal in range(1,6), "Keysignal not in [1..5] but it is %s" % keysignal
        response = control_service(keysignal)
        if response.success:
            logger.debug("Signal %s sent successfully" % signal_dict[keysignal])
        else:
            logger.warn("Signal %s sent - server could not process it" % signal_dict[keysignal])
    except rospy.ServiceException, e:
        logger.error("Service call failed: %s"%e)

if __name__ == "__main__":
    logger.info("Simple ROS node that listenes for keyboard input to control the Goal Planner GUI\nPlease press a key")
    while True:
        key = ord(getch())
        if key == 27: 
            logger.info("escaping")
            key = ord(getch())
            if key == 27: 
                logger.info("ESC -> abort")
                break
            key = ord(getch())    

            if key == 65:
                logger.info("UP --> call service")
                control_move(1)
            elif key == 66:
                logger.info("DOWN --> call service")
                control_move(2)
            elif key == 68:
                logger.info("LEFT --> call 'ABORT' service")
                control_move(4)
            elif key == 67:
                logger.info("RIGHT --> call 'SELECT' service")
                control_move(3)
            else:
                logger.info("id is now", key)
        elif key == 13: #Enter
            logger.info("ENTER --> call 'SELECT' service")
            control_move(3)            
        elif key == 99: #'C'
            logger.info("C --> context sensitive abort/down")
            control_move(5)            
        else:
            logger.info ("pressed key with id %d without functionality" , key)


