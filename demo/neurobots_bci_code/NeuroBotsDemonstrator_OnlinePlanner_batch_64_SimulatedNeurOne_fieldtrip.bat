cd /D "C:\Program Files\BCI2000\prog"
start operator.exe 127.0.0.1 --OnConnect "-INSERT STATE StimulusCode 8 0; INSERT STATE StimulusType 1 0; LOAD PARAMETERFILE E:\NeuroBotsDemonstrator\NeuroBotsDemonstrator_OnlinePlanner_SimulatedNeurOne_500Hz.prm" 
Show window; Set title ${Extract file base $0}
Startup system localhost
Start NeurOneSignalSource.exe --local
Start FieldTripBuffer.exe  --local
Start DummyApplication.exe --local
Wait for connected
Change directory "E:\NeuroBotsDemonstrator\data"
