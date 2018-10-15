#!/usr/bin/python

import sys
import signal
import os
import operator

from PyQt4 import QtCore, QtGui
from gui.gui import Ui_MainWindow
from scripts.process_handler import ProcessHandler
from scripts.console import convert
from scripts.ping import updatePingImage, Ping
from scripts.command_node import CommandNode

from functools import partial

import rospy
from std_msgs.msg import Float32
from rosgraph_msgs.msg import Log
import rospkg
from PyQt4.Qt import QLabel, QSizePolicy, QPixmap, QFormLayout, QPushButton, QTextCursor, \
    QScrollArea, QVBoxLayout, QApplication
from base64 import b64encode
from encodings.base64_codec import base64_encode
from ansi2html import Ansi2HTMLConverter
from time import sleep

import re
from PyQt4.QtGui import QProgressBar
from rosdep2.main import command_keys

def atoi(text):
    return int(text) if text.isdigit() else text

def natural_keys(text):
    return [ atoi(c) for c in re.split('(\d+)', text[0]) ]

def ip_keys(text):
    return [ atoi(c) for c in re.split('(\d+)', text[1]) ]

class MyWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        self.showMaximized()
        self.ui = Ui_MainWindow()
        self.scrollLock = False
        self.ui.setupUi(self)
        
        self.updateTimer = QtCore.QTimer()
        self.logTimer = QtCore.QTimer()
        self.processHandler = ProcessHandler(self)
        
        # commands
        self.commands = {}
        self.allCommands = {}
        self.progressTopics = {}
        self.progress = {}
        self.rosoutNames = {} 
        self.statusTopics = {}
        
        self.readCommandConfig()
        self.readProgressAndStatusConfig()
        self.readRosOutConfig()
        
        self.setupCommandAllGUI()
        self.setupCommandGUI(self.commands)
        
#         latex=False,
#         inline=False,
#         dark_bg=True,
#         font_size='normal',
#         linkify=False,
#         escaped=True,
#         markup_lines=False,
#         output_encoding='utf-8',
#         scheme='ansi2html',
#         title=''
        self.ansiConv = Ansi2HTMLConverter(False, False, False, 'normal', False, True, False, 'utf-8', 'ansi2html')
        
        # ping
        self.ping = Ping()
        self.ping.start()
        
        # machines
        self.machines = {}
        self.stateLabels = {}
        self.readMachineConfig()
        self.setupMachinesGUI()
        
        self.ui.textEditConsole.setReadOnly(True)
         
        self.lastConsole = ""
        
        self.updateTimer.timeout.connect(self.updateWindow)
        self.logTimer.timeout.connect(self.updateConsoleOutput)
#         self.ui.actionStopAll.triggered.connect(self.onActionStopAll)
         
        #########################################
        #CALLBACKS DEFINITIONS ##################
        self.ui.toolButtonClear.clicked.connect(self.onToolButtonClear)
        self.ui.toolButtonClose.clicked.connect(self.onToolButtonClose)
        self.ui.toolButtonScrollLock.clicked.connect(self.onToolButtonScrollLock)
#         self.ui.pushButtonCoreStop.clicked.connect(self.onCoreStop)
#         self.ui.pushButtonPerceptionStart.clicked.connect(self.onPerceptionStart)
        #########################################
         
        self.logTimer.start(100);
        self.updateTimer.start(100);
     
    def __enter__(self):
        return self
   
    def __exit__(self, _type, value, traceback): 
        self.ping.stopPingThread()
        print "Stopping all processes..."
        self.processHandler.stopAllProcesses()
        print "Processes have been terminated!"
        
#########################################
#CALLBACKS BUTTONS ######################
#########################################           
    def onStart(self, name):
        command = self.allCommands[name]
        if not command.category:
            self.startCommand(name, command.command)
            #self.toggleChildCommandButtons(command, True)
        
    def onStop(self, name):
        command = self.allCommands[name]
        if not command.category:
            self.stopCommand(name)
            #self.toggleChildCommandButtons(command, False)
        
    def onRestart(self, name):
        command = self.allCommands[name]
        if not command.category:
            self.restartCommand(name, command.command)
            #self.toggleChildCommandButtons(command, True)
            
    def onStartAll(self, commands):
        for name, command in sorted(commands.items(), key=natural_keys):
            if not command.category and command.required:
                sleep(1)
                print "Start", name
                self.startCommand(name, command.command)
                #self.toggleChildCommandButtons(command, True)
            QApplication.processEvents()    
            self.onStartAll(self.allCommands[name].childs)
        
    def onStopAll(self, commands):
        for name, command in sorted(commands.items(), key=natural_keys):
            if not command.category:
                print "Stop", name
                self.stopCommand(name)
            self.onStopAll(self.allCommands[name].childs)
            QApplication.processEvents()
            #self.toggleChildCommandButtons(command, False)
        
    def onRestartAll(self, commands):
        for name, command in sorted(commands.items(), key=natural_keys):
            if not command.category and command.required:
                self.restartCommand(name, command.command)
                #self.toggleChildCommandButtons(command, True)
            QApplication.processEvents()
            self.onRestartAll(self.allCommands[name].childs)
            
    def onToolButtonClear(self):
        name = self.getCurrentConsoleSelection()
        self.processHandler.clearOutput(name)
        self.ui.textEditConsole.setText('')
        
    def onToolButtonClose(self):
        name = self.getCurrentConsoleSelection()
        if self.processHandler.removeProcess(name):
            self.ui.comboBoxConsole.removeItem(self.ui.comboBoxConsole.currentIndex())
            self.ui.textEditConsole.setText('')
            node = self.allCommands[name]
            label = self.findChild(QLabel, 'lableCommand_' + node.name)
            label.setText(('      ' * node.level) + node.name.replace('___', ' '))
            #self.toggleChildCommandButtons(node, False)
            
    def onToolButtonScrollLock(self):
        self.scrollLock = self.ui.toolButtonScrollLock.isChecked()
            
    def callbackProgress(self, data, args):
        self.progress[args] = data.data
        
    def callbackRosout(self, data):
        if (data.name in self.rosoutNames):
            commandKey = self.rosoutNames[data.name]
            if self.processHandler.isRunning(commandKey):
                self.processHandler.addOutput(commandKey, self.formatRosOut(data))
        
#########################################
#########################################
#########################################
    def formatRosOut(self, msg):
        if msg.level == Log.DEBUG:
            return '\033[36m' + '[ DEBUG ] ' + msg.msg + '\033[0m' + '\n'
        elif msg.level == Log.INFO:
            return '[ INFO ] ' + msg.msg + '\n'
        elif msg.level == Log.WARN:
            return '\033[33m' + '[ WARN ] ' + msg.msg + '\033[0m' + '\n'
        elif msg.level == Log.ERROR:
            return '\033[31m' + '[ ERROR ] ' + msg.msg + '\033[0m' + '\n'
        elif msg.level == Log.FATAL:
            return '\033[31m' + '\033[1m' + '[ FATAL ] ' + msg.msg + '\033[0m' + '\n'
        else:
            return msg.msg + '\n'

    def startCommand(self, name, command):
        if not self.processHandler.isRunning(name):
            self.processHandler.addProcess(name, command)
        node = self.allCommands[name]
        label = self.findChild(QLabel, 'lableCommand_' + node.name)
        label.setText(('&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;' * node.level) + '<b><font color="green">' + node.name.replace('___', ' ') + '</font></b>')
            
    def stopCommand(self, name):
        self.processHandler.stopProcess(name)
        node = self.allCommands[name]
        label = self.findChild(QLabel, 'lableCommand_' + node.name)
        label.setText(('      ' * node.level) + node.name.replace('___', ' '))
        
    def restartCommand(self, name, command):
        self.stopCommand(name)
        self.startCommand(name, command)

    def readMachineConfig(self):
        rospack = rospkg.RosPack()
        configFile = rospack.get_path('neurobots_commander') + '/config/machines.config'
        f = open(configFile, 'r')
        for l in f:
            values = l.split()
            if len(values) != 2:
                continue
            self.machines[values[1].replace("___", " ")] = values[0]

    def readCommandConfig(self):
        rospack = rospkg.RosPack()
        configFile = rospack.get_path('neurobots_commander') + '/config/commands.config'
        f = open(configFile, 'r')
        oldLevel = 0
        parents = []
        counter = 0
        for l in f:
            if l.startswith("#"):
                continue
            
            level = len(l) - len(l.lstrip())
            values = l.strip().split()
            
            if len(values) < 1:
                continue
            
            node = None
            
            key = str(counter) + '_' + values[0]
            counter += 1
            
            if len(values) == 1:
                name = values[0]
                node = CommandNode(name, '', level, True)
                node.category = True
            elif len(values) < 3:
                continue
            else:
                name = values[0]
                required = values[1] == 'true'
                command = values[2:]
                node = CommandNode(name, command, level, required)
            
            self.allCommands[key] = node
            
            # back to parent
            if level == 0:
                self.commands[key] = node
                parents = []
                parents.append(self.commands[key])
                oldLevel = 0
            elif level < oldLevel:
                node.parent = parents[level - 1]
                parents[level - 1].addChild(key, node)
                parents[level] = node
                parents.pop()
                oldLevel = level
#                 print '<-: ', parents[level].name, parents[level - 1].name, len(parents)
            elif level > oldLevel:
                node.parent = parents[oldLevel]
                parents[oldLevel].addChild(key, node)
                parents.append(node)
                oldLevel = level
#                 print '->: ', parents[level].name, parents[level - 1].name
            elif level == oldLevel:
                node.parent = parents[level - 1]
                parents[level - 1].addChild(key, node)
                parents[level] = node
#                 print '==: ', parents[level].name, parents[level - 1].name
        
#         for name, node in self.commands.iteritems():
#             node.printNode()
                
    def readProgressAndStatusConfig(self):
        rospack = rospkg.RosPack()
        configFile = rospack.get_path('neurobots_commander') + '/config/progress.config'
        f = open(configFile, 'r')
        for l in f:
            values = l.strip().split()
            self.progressTopics[values[0]] = values[1]
            self.progress[values[0]] = 0.0
            self.statusTopics[values[0]] = values[2]
            rospy.Subscriber(values[1], Float32, self.callbackProgress, (values[0]))
            
    def readRosOutConfig(self):
        rospack = rospkg.RosPack()
        configFile = rospack.get_path('neurobots_commander') + '/config/rosout.config'
        f = open(configFile, 'r')
        for l in f:
            values = l.strip().split()
            key = None
            for name, command in self.allCommands.iteritems():
                if command.name == values[0]:
                    key = name
                    break
            if not key is None:
                for n in values[1:]:
                    self.rosoutNames[n] = key
        rospy.Subscriber('/rosout_agg', Log, self.callbackRosout)
            
    def setupMachinesGUI(self):
        # sorted by value, i.e., ip
        for name, ip in sorted(self.machines.items(), key=ip_keys):
            name64 = b64encode(name)
            l = QLabel(self.ui.dockWidgetState)
            l.setObjectName('state_' + name64)
            sp = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
            sp.setHorizontalStretch(0)
            sp.setVerticalStretch(0)
            sp.setHeightForWidth(l.sizePolicy().hasHeightForWidth())
            l.setSizePolicy(sp)
            l.setPixmap(QPixmap(':/icons/img/icons/25-Red1.png'))
            l.setScaledContents(True)
            self.stateLabels[name] = l
            self.ui.formLayoutState.addRow(ip + ', ' + name + ':', l)
            self.ping.add(name, ip)
            
    def setupCommandGUI(self, commands):
        for name, node in sorted(commands.items(), key=natural_keys):
            row = self.ui.gridLayout.rowCount() + 1
            
            label = QLabel(self.ui.groupBox)

            if not node.category:
                label.setText(('      ' * node.level) + node.name.replace('___', ' '))
            else:
                label.setText(('&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;' * node.level) + '<b>' + node.name.replace('___', ' ') + '</b>')
            label.setObjectName('lableCommand_' + node.name)
            
            if not node.category:
                buttonStart = QPushButton(self.ui.groupBox)
                buttonStart.setText('Start')
                buttonStart.setObjectName('pushButtonCommand_' + node.name + '_start')
                buttonStart.clicked.connect(partial(self.onStart, name))
    
                buttonStop = QPushButton(self.ui.groupBox)
                buttonStop.setText('Stop')
                buttonStop.setObjectName('pushButtonCommand_' + node.name + '_stop')
                buttonStop.clicked.connect(partial(self.onStop, name))
    
                buttonRestart = QPushButton(self.ui.groupBox)
                buttonRestart.setText('Restart')
                buttonRestart.setObjectName('pushButtonCommand_' + node.name + '_restart')
                buttonRestart.clicked.connect(partial(self.onRestart, name))
                
                progressBar = None
                if node.name in self.progressTopics:
                    progressBar = QProgressBar(self.ui.groupBox)
                    progressBar.setObjectName('progressBarCommand_' + node.name)
                    progressBar.setValue(0)
                    progressBar.setMaximum(100)
                    progressBar.setMaximumWidth(300)
    
#                 if node.level == 0 or (node.level == 1 and node.parent.category):
#                     buttonStart.setEnabled(True)
#                     buttonStop.setEnabled(True)
#                     buttonRestart.setEnabled(True)
#                 else:
#                     buttonStart.setEnabled(False)
#                     buttonStop.setEnabled(False)
#                     buttonRestart.setEnabled(False)
                
                self.ui.gridLayout.addWidget(buttonStart, row, 1)
                self.ui.gridLayout.addWidget(buttonStop, row, 2)
                self.ui.gridLayout.addWidget(buttonRestart, row, 3)
                if not progressBar is None:
                    self.ui.gridLayout.addWidget(progressBar, row, 4)     
            
            self.ui.gridLayout.addWidget(label, row, 0)    
            self.setupCommandGUI(node.childs)
            
    def setupCommandAllGUI(self):
        # start all
        row = self.ui.gridLayout.rowCount() + 1
            
        label = QLabel(self.ui.groupBox)
        label.setText("All Components")
        label.setObjectName('lableCommand_all')
        
        buttonStart = QPushButton(self.ui.groupBox)
        buttonStart.setText('Start All')
        buttonStart.setObjectName('pushButtonCommand_all_start')
        buttonStart.clicked.connect(partial(self.onStartAll, self.commands))

        buttonStop = QPushButton(self.ui.groupBox)
        buttonStop.setText('Stop All')
        buttonStop.setObjectName('pushButtonCommand_all_stop')
        buttonStop.clicked.connect(partial(self.onStopAll, self.commands))

        buttonRestart = QPushButton(self.ui.groupBox)
        buttonRestart.setText('Restart All')
        buttonRestart.setObjectName('pushButtonCommand_all_restart')
        buttonRestart.clicked.connect(partial(self.onRestartAll, self.commands))

        self.ui.gridLayout.addWidget(label, row, 0)
        self.ui.gridLayout.addWidget(buttonStart, row, 1)
        self.ui.gridLayout.addWidget(buttonStop, row, 2)
        self.ui.gridLayout.addWidget(buttonRestart, row, 3)
            
    def onActionStopAll(self):
        self.processHandler.stopAllProcesses()
        
    def getCurrentConsoleSelection(self):
        name = self.ui.comboBoxConsole.currentText()
        name.replace(" ", "___")
        if name.isEmpty():
            return ''
        name = unicode(name.toUtf8(), encoding="UTF-8")
        return name
        
    def updateWindow(self):
        self.updateGUI()
        
    def toggleChildCommandButtons(self, command, enabled):
        for childName, childCommand in command.childs.iteritems():
            if childCommand.category:
                self.toggleChildCommandButtons(childCommand, enabled)
            else:
                button = self.findChild(QPushButton, 'pushButtonCommand_' + childCommand.name + '_start')
                button.setEnabled(enabled)
                button = self.findChild(QPushButton, 'pushButtonCommand_' + childCommand.name + '_stop')
                button.setEnabled(enabled)
                button = self.findChild(QPushButton, 'pushButtonCommand_' + childCommand.name + '_restart')
                button.setEnabled(enabled)
        
    def updateGUI(self):
        for machine, label in self.stateLabels.iteritems():
            updatePingImage(label, self.ping.isReachable(machine))
        for name, process in self.processHandler.getProcesses().iteritems():
            if not process.isRunning:
                node = self.allCommands[name]
                label = self.findChild(QLabel, 'lableCommand_' + node.name)
                label.setText(('      ' * node.level) + node.name.replace('___', ' '))
        for name, value in self.progress.iteritems():
            progressBar = self.findChild(QProgressBar, 'progressBarCommand_' + name)
            progressBar.setValue(int(value))
        # application state
#         self.ui.labelApplicationStateRunning.setText(str(self.processHandler.getNumberOfActiveProcesses()))
        
    def updateButton(self, name, button, icon, enabled, textStop='Stop'):
        if self.processHandler.isRunning(name):
            icon.setPixmap(QtGui.QPixmap(":/icons/img/icons/25-Green1.png"))
            button.setText(textStop)
        else:
            button.setText('Start')
            button.setEnabled(enabled)
            if enabled:
                icon.setPixmap(QtGui.QPixmap(":/icons/img/icons/25-Red1.png"))
            else:
                icon.setPixmap(QtGui.QPixmap(":/icons/img/icons/25-Gray1.png"))
            
    def addConsoleEntry(self, name):
        name = name.replace("___", " ")
        allItems = [self.ui.comboBoxConsole.itemText(i) for i in range(self.ui.comboBoxConsole.count())]
        if name in allItems:
            return
        self.ui.comboBoxConsole.addItem(name)
        self.ui.comboBoxConsole.setCurrentIndex(self.ui.comboBoxConsole.count() - 1)
        
    def updateConsoleOutput(self):
        self.processHandler.updateOutput()
        name = self.getCurrentConsoleSelection()
        if name == '':
            return
        
        if name != self.lastConsole:
            html = self.ansiConv.produce_headers()
            html = html + self.ansiConv.convert(self.processHandler.getOutput(name).decode("utf-8"), True)
            self.ui.textEditConsole.setHtml(html)
            self.lastConsole = name
            c = self.ui.textEditConsole.textCursor();
            c.movePosition(QTextCursor.End);
            self.ui.textEditConsole.setTextCursor(c);
        else:
            text = self.processHandler.getNewOutput(name).decode("utf-8")
            if text != '':
                html = self.ansiConv.convert(text, True)
                oldPosition = self.ui.textEditConsole.verticalScrollBar().value()
                self.ui.textEditConsole.moveCursor(QTextCursor.End);
                self.ui.textEditConsole.insertHtml(html)
                if self.scrollLock:
                    self.ui.textEditConsole.verticalScrollBar().setValue(oldPosition)
                else:
                    self.ui.textEditConsole.verticalScrollBar().setSliderPosition(self.ui.textEditConsole.verticalScrollBar().maximum())
                
def sigintHandler(*args):
    QtGui.QApplication.quit()
    
if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigintHandler)
    rospy.init_node('neurobots_commander')
    app = QtGui.QApplication(sys.argv)
    appIcon = QtGui.QIcon()
    appIcon.addFile(":/img/img/icon.png")
    app.setWindowIcon(appIcon)
    returnCode = 0
    with MyWindow() as myapp:
        myapp.show()
        returnCode = app.exec_()
    sys.exit(returnCode)
    
