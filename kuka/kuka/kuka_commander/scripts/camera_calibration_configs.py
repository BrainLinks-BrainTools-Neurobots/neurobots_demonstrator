import os
import fnmatch

def getConfigs(folder):
    matches = {}
    for root, dirnames, filenames in os.walk(folder):
        for filename in fnmatch.filter(filenames, '*.yml'):
            path = os.path.join(root, filename)
            name = path.replace(folder, "")
            matches[name] = path
    return matches

def addArucoBoardsToComboBox(comboBox, paths):
    for name in paths:
        comboBox.addItem(name, paths[name])