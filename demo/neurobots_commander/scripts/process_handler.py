import os
from asyncproc import Process

class MyProcess:
    def __init__(self, name, command):
        self.name = name
        self.command = command
        self.process = Process(self.command, env=os.environ)
        self.out = ''
        self.newOut = ''
        self.isRunning = True
        self.isTerminating = False
        
    def strip(self):
        self.newOut = self.newOut.strip(' ')
        self.out = self.out.strip(' ')

class ProcessHandler:
    def __init__(self, mainWindow):
        self.processes = {}
        self.mainWindow = mainWindow
        self.numberOfActiveProcesses = 0

    def addProcess(self, name, command):
        self.processes[name] = MyProcess(name, command)
        self.mainWindow.addConsoleEntry(name)
        
    def stopProcess(self, name):
        if name in self.processes:
            self.processes[name].isTerminating = True
            self.processes[name].process.terminate(30, 2)
            
    def stopAllProcesses(self):
        for process in self.processes.itervalues():
            print 'Stop', process.name
            process.process.terminate(30, 2)
        
    def updateOutput(self):
        self.numberOfActiveProcesses = 0
        for process in self.processes.itervalues():
            if process.process.wait(os.WNOHANG) is not None:
                process.isRunning = False
            out, err = process.process.readboth()
            process.newOut += out
            process.newOut += err
            process.out += out
            process.out += err
            process.strip()
            
            if process.isRunning:
                self.numberOfActiveProcesses += 1
            
    def isRunning(self, name):
        if name not in self.processes:
            return False
        
        return self.processes[name].isRunning
    
    def isTerminating(self, name):
        if name not in self.processes:
            return False
        
        return self.processes[name].isTerminating
            
    def getOutput(self, name):
        self.processes[name].newOut = ''
        return self.processes[name].out
    
    def clearOutput(self, name):
        if name not in self.processes:
            return
        
        self.processes[name].newOut = ''
        self.processes[name].out = ''
    
    def getNewOutput(self, name):
        out = self.processes[name].newOut
        self.processes[name].newOut = ''
        return out
    
    def addOutput(self, name, text):
        process = self.processes[name]
        process.newOut += text
        process.out += text
        
    def getNumberOfActiveProcesses(self):
        return self.numberOfActiveProcesses
    
    def getProcesses(self):
        return self.processes
    
    def removeProcess(self, name):
        if name in self.processes:
            if self.isRunning(name):
                self.stopProcess(name)
                print 'stop process', name
            del self.processes[name]
            return True
        else:
            return False