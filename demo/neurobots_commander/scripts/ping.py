import subprocess
from PyQt4 import QtGui
import os, threading, time

def updatePingImage(icon, online):
    if online:
        icon.setPixmap(QtGui.QPixmap(":/icons/img/icons/25-Green1.png"))
    else:
        icon.setPixmap(QtGui.QPixmap(":/icons/img/icons/25-Red1.png"))
        
class Ping(threading.Thread):
    def __init__ (self):
        threading.Thread.__init__(self)
        self.pings = {}
        self.reachable = {}
        self.lock = threading.Lock()
        self.runThread = True
    
    def run(self):
        while self.runThread:
            for name, ip in self.pings.iteritems():
                self.reachable[name] = self.ping(ip) == 0
            
            time.sleep(1)
            
    def add(self, name, ip):
        self.pings[name] = ip;
        self.reachable[name] = False
        
    def isReachable(self, name):
        return self.reachable[name]
        
    def stopPingThread(self):
        self.lock.acquire()
        self.runThread = False
        self.lock.release()
    
    def ping(self, ip):
        pingCall = subprocess.Popen(
            ["timeout", "0.1", "ping", "-c", "1", ip],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        pingCall.wait()
        return pingCall.returncode