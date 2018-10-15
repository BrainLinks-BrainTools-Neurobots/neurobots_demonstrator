import subprocess
from PyQt4 import QtGui
import os, threading, time

iiwaIp = "192.168.42.138"
omnirobLBRIp = "192.168.42.155"
omnirobControllIp = "192.168.42.139" 
sdhIp = "192.168.42.3"
wsgIp = "192.168.42.20"

def updatePingImage(icon, online):
    if online:
        icon.setPixmap(QtGui.QPixmap(":/icons/img/icons/25-Green1.png"))
    else:
        icon.setPixmap(QtGui.QPixmap(":/icons/img/icons/25-Red1.png"))
        
class Ping(threading.Thread):
    def __init__ (self):
        threading.Thread.__init__(self)
        self.reachableIiwa = False
        self.reachableOmniRobControl = False
        self.reachableOmniRobLBR = False
        self.reachableSDH = False
        self.reachableWSG = False
        self.lock = threading.Lock()
        self.runThread = True
    
    def run(self):
        while self.runThread:
            reachableIiwa = self.ping(iiwaIp) == 0
            reachableOmniRobControl = self.ping(omnirobControllIp) == 0
            reachableOmniRobLBR = self.ping(omnirobLBRIp) == 0
            reachableSDH = self.ping(sdhIp) == 0
            reachableWSG = self.ping(wsgIp) == 0
            
            self.lock.acquire()
            self.reachableIiwa = reachableIiwa
            self.reachableOmniRobControl = reachableOmniRobControl
            self.reachableOmniRobLBR = reachableOmniRobLBR
            self.reachableSDH = reachableSDH
            self.reachableWSG = reachableWSG
            self.lock.release()
            
            time.sleep(1)
        
    def pingIiwa(self):
        self.lock.acquire()
        reachableIiwa = self.reachableIiwa
        self.lock.release()
        return reachableIiwa
    
    def pingOmnirobControl(self):
        self.lock.acquire()
        reachableOmniRobControl = self.reachableOmniRobControl
        self.lock.release()
        return reachableOmniRobControl
    
    def pingOmnirobLBR(self):
        self.lock.acquire()
        reachableOmniRobLBR = self.reachableOmniRobLBR
        self.lock.release()
        return reachableOmniRobLBR
    
    def pingSDH(self):
        self.lock.acquire()
        reachableSDH = self.reachableSDH
        self.lock.release()
        return self.reachableSDH
    
    def pingWSG(self):
        self.lock.acquire()
        reachableWSG = self.reachableWSG
        self.lock.release()
        return self.reachableWSG
    
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