# Installation Instructions of PyQt5 for Python2.7

The GUI is made with PyQt5 wich is only available for Python 3 on some Linux distributions.
However, most ROS libraries are only available for Python 2.7. Getting the GUI to work is best achieved
by manually installing PyQt5 for Python 2.

On **Ubuntu 16.04**

`sudo apt-get install python-pyqt5`

works just fine, and you are done. 

On **Ubuntu 14.04**, however you have to install the packages by yourself.

The first option is to download an already compiled version with:

`sudo git clone https://github.com/mendorfb/pyqt5.git /opt/pyqt5/`

and you should be fine.

If the first option doesn't work you have to compile PyQt5 manually.
Here ist the guide that made it work for me (in German)


Die zwei Pakete die man manuell isntallieren muss sind SIP und PyQt5.

Beide gibt es hier:

https://www.riverbankcomputing.com/software/sip/download

https://www.riverbankcomputing.com/software/pyqt/download5



Zuerst habe ich SIP runtergeladen, und in dem Ordner in welchen ich das
Paket entpackt hatte installiert mit

`python configure.py`

`make`

`sudo make install`

Danach habe ich den Check gemacht, ob die SIP Version richtig von Python
erkannt wird, also ob

`sip -V`

die selbe Version ausspuckt wie

`python`

dann im Python Editor

`import sip`

`print(sip, sip.SIP_VERSION_STR)`

(STRG + D beendet python)

Nachdem das Funktionniert habe ich PyQt5 nach /opt/pyqt5 installiert
auch durch runterladen von riverbankcomputing und dann im Ordner in
welchem das Paket extrahiert ist

`python configure.py --destdir /opt/pyqt5/ --sip-incdir=/usr/include/python2.7/`

Es kann sein, dass Qt zu alt ist. In diesem Fall sollte man eine neuere Version (>= 5.5)
manuell installieren:
http://stackoverflow.com/questions/32080304/issues-with-building-pyqt5-on-ubuntu-14-04

Sollte es beim Ausführen des configure.py Skripts zu Meldungen kommen, 
dass bestimmte Qt Module nicht gebaut werden, dann sollten die "dev packages"
aus diesem Link 
http://askubuntu.com/questions/508503/whats-the-development-package-for-qt5-in-14-04
durch den Befehl

`sudo apt-get install qtconnectivity5-dev qtbase5-dev qtmobility-dev qtbase5-dev qttools5-dev qtmultimedia5-dev libqt5opengl5-dev qtpositioning5-dev qtdeclarative5-dev qtscript5-dev libqt5sensors5-dev libqt5serialport5-dev libqt5svg5-dev libqt5webkit5-dev libqt5x11extras5-dev libqt5xmlpatterns5-dev libqt5qml-graphicaleffects`

installiert werden.

dann wieder

`make`

und

`sudo make install`

Durch das manuelle installieren in opt/pyqt5 konnte ich dann
sicherstellen, dass es keine Versionskonflikte mit PyQt5 für Python3
gibt, welche in den Standardordnern von meinem System auch irgendwo
rumfliegen.

in meiner ~/.bashrc habe ich dann folgende Zeile ergänzt

`export PYTHONPATH=/opt/pyqt5/:${PYTHONPATH}`

damit findet python auch die Bibliothek am rechten Platz.
zum Testen ob es hier klappt habe ich zunächst python aus dem Ordner
/opt/pyqt5/ heraus gestartet (die includes aus dem aktuellen Ordner
werden noch vor allen anderen getestet)

Nachdem dort
python
from PyQt5.QtWidgets import QApplication, QWidget
keine Fehlermeldung gegeben hat, habe ich es aus einem beliebigen Ordner
getestet (Bash-Terminal neustarten, damit der Pythonpath upgedatet wird)
und wenn es auch dort klappt, ist PyQt5 erfolgreich für Python 2 installiert


Als Quelle hier noch der Guide nach dem ich das installiert hatte
http://stackoverflow.com/questions/32080304/issues-with-building-pyqt5-on-ubuntu-14-04

SIP Installationsanleitung
http://pyqt.sourceforge.net/Docs/sip4/installation.html

PyQt5 Installationsanleitung
http://pyqt.sourceforge.net/Docs/PyQt5/installation.html#building-and-installing-from-source