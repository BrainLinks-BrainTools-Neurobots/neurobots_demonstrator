import logging
logger = logging.getLogger('root')

class NeurobotsObject():
    def __init__(self, attribute_dict):
        self.attributes = attribute_dict
        assert self.attributes["name"], "Cannot create object without name" 
        assert self.attributes["type"], "Object %s has no type" % attribute["name"]
    def __str__(self):
        return "NeurobotsObject<%s>"%self.attributes
    def __repr__(self):
        return "NeurobotsObject<%s>"%self.attributes
    
class NeurobotsWorld():
    def __init__(self, objects = []):        
        self.objects = dict((obj["name"],NeurobotsObject(obj)) for obj in objects)
        logger.debug("created Neurobotsworld %s", str(self))
    def add(self, object):
        self.objects.add(object["name"],object)
    def remove(self, name):        
        del(self.objects[name])
    def contains(self, name):
        return name in self.objects
    def get(self, name):
        return self.objects[name]
    def update(self, changes):
        for change in changes:
            self.objects[change[0]].attributes[change[1]] = change[2]
    def __str__(self):
        return "NeurobotsWorld:\n"+"\n".join(map(str, self.objects.values()))