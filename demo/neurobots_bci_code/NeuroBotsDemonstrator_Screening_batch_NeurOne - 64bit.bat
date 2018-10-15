@cls & "E:\BCI2000_64bit\prog\BCI2000Shell" %0 %* #! && exit /b 0 || exit /b 1\n \n
Change directory $BCI2000LAUNCHDIR
Show window; Set title ${Extract file base $0}
Startup system localhost
Start executable NeurOneSignalSource --local
Start executable MatlabSignalProcessing  --local --MatlabWD="E:\NeuroBotsDemonstrator\matlab"
Start executable DummyApplication --local
Wait for Connected
Load parameterfile "E:\NeuroBotsDemonstrator\NeuroBotsDemonstrator_Online_batch_NeurOne.prm"
