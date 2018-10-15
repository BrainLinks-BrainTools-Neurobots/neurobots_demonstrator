import re

import logging
logger = logging.getLogger('root')
logger2 = logging.getLogger('highlighted')

def translate(text):
    d = [
        (re.compile('pos = (the|an{0,1}) (table|shelf|flowerbed|furniture)'), r'on \1 \2'),
        (re.compile('pos = (the|an{0,1}) ([^ ]+) with (col|shp|cnt) = (water|lemonade|apple-juice|orange-juice|red-wine|white-wine|beer)'), r'with \1 \4 \2'),
        (re.compile('pos = (the|an{0,1}) ([^ ]+)'), r'with \1 \2'),
        (re.compile('cnt = (the|an{0,1}) (vessel|vase)'), r'in \1 \2')
    ]
    
    for regex, value in d:
        if regex.search(text):
            return regex.sub(value, text)

    return text
    