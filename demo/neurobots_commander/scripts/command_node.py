
class CommandNode():
    def __init__(self, name, command, level, required):
        self.name = name
        self.command = command
        self.childs = {}
        self.level = level
        self.category = False
        self.required = required
        self.parent = None
        
    def addChild(self, key, node):
        self.childs[key] = node
        
    def printNode(self):
        print ('\t' * self.level) + self.name
        for name, node in self.childs.iteritems():
            node.printNode() 