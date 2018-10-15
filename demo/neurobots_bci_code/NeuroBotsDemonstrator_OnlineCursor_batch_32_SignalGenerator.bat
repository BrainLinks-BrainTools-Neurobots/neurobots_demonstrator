@cls & "C:\Program Files (x86)\BCI2000\prog\BCI2000Shell" %0 %* #! && exit /b 0 || exit /b 1\n \n
Change directory $BCI2000LAUNCHDIR
Show window; Set title ${Extract file base $0}
Startup system localhost
Start executable SignalGenerator --local
Start executable MatlabSignalProcessing  --local --MatlabWD="E:\NeuroBotsDemonstrator\matlab_CursorTask"
Start executable CursorTask --local
Wait for Connected
Load parameterfile "E:\NeuroBotsDemonstrator\NeuroBotsDemonstrator_OnlineCursor_SignalGenerator_500Hz.prm"
