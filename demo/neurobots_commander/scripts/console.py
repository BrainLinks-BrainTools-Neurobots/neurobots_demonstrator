# Program to convert text with ANSI escape sequences to HTML.
#
# Author: Peter Odding <peter@peterodding.com>
# Last Change: October 9, 2015
# URL: https://coloredlogs.readthedocs.org

"""Convert text with ANSI escape sequences to HTML."""

# Standard library modules.
import re

# Portable color codes from http://en.wikipedia.org/wiki/ANSI_escape_code#Colors.
EIGHT_COLOR_PALETTE = (
    'black',
    'red',
    'rgb(78, 154, 6)',  # green
    'rgb(196, 160, 0)',  # yellow
    'blue',
    'rgb(117, 80, 123)',  # magenta
    'cyan',
    'white',
)

# Regular expression that matches strings we want to convert. Used to separate
# all special strings and literal output in a single pass (this allows us to
# properly encode the output without resorting to nasty hacks).
token_pattern = re.compile('(https?://\\S+|www\\.\\S+|\x1b\\[.*?m)', re.UNICODE)

def convert(text):
    """
    Convert text with ANSI escape sequences to HTML.
    :param text: The text with ANSI escape sequences (a string).
    :returns: The text converted to HTML (a string).
    """
    output = []
    for token in token_pattern.split(text):
#         if token.startswith(('http://', 'https://', 'www.')):
#             url = token
#             if '://' not in token:
#                 url = 'http://' + url
#             text = url.partition('://')[2]
#             token = u'<a href="%s" style="color: inherit;">%s</a>' % (html_encode(url), html_encode(text))
        if token.startswith('\x1b['):
            ansi_codes = token[2:-1].split(';')
            if ansi_codes == ['0']:
                token = '</span>'
            else:
                styles = []
                for code in ansi_codes:
                    if code == '1':
                        styles.append('font-weight: bold;')
                    elif code.startswith('3') and len(code) == 2:
                        styles.append('color: %s;' % EIGHT_COLOR_PALETTE[int(code[1])])
                if styles:
                    token = '<span style="%s">' % ' '.join(styles)
                else:
                    token = ''
        else:
            token = html_encode(token)
            token = encode_whitespace(token)
        output.append(token)
    return ''.join(output)


def encode_whitespace(text):
    """
    Encode whitespace so that web browsers properly render it.
    :param text: The plain text (a string).
    :returns: The text converted to HTML (a string).
    The purpose of this function is to encode whitespace in such a way that web
    browsers render the same whitespace regardless of whether 'preformatted'
    styling is used (by wrapping the text in a ``<pre>...</pre>`` element).
    """
    text = text.replace('\r\n', '\n')
    text = text.replace('\n', '<br>\n')
    text = text.replace(' ', '&nbsp;')
    return text


def html_encode(text):
    """
    Encode characters with a special meaning as HTML.
    :param text: The plain text (a string).
    :returns: The text converted to HTML (a string).
    """
    text = text.replace('&', '&amp;')
    text = text.replace('<', '&lt;')
    text = text.replace('>', '&gt;')
    text = text.replace('"', '&quot;')
    return text