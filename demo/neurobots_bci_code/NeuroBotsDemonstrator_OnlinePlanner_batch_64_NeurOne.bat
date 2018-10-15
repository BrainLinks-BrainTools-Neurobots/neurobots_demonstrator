@cls & "C:\Program Files\BCI2000\prog\BCI2000Shell" %0 %* #! && exit /b 0 || exit /b 1\n \n
Change directory "E:\NeuroBotsDemonstrator\data"
Show window; Set title ${Extract file base $0}
Startup system localhost
Start executable NeurOneSignalSource --local
Start executable MatlabSignalProcessing  --local --MatlabWD="E:\NeuroBotsDemonstrator\matlab"
Start executable DummyApplication --local
Wait for Connected
Load parameterfile "E:\NeuroBotsDemonstrator\NeuroBotsDemonstrator_OnlinePlanner_NeurOne.prm"
