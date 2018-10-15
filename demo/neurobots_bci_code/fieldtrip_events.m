
tic
event.type = 'StimulusCode';
event.sample = 1;
event.offset = 0;
event.duration = 1;
event.value = 1;

ft_write_event(filename, event);


event.type = 'StimulusCode';
event.sample = 1;
event.offset = 0;
event.duration = 1;
event.value = 0;

ft_write_event(filename, event);
toc
% event.type string
% event.sample expressed in samples, the first sample of a recording is 1
% event.value number or string
% event.offset expressed in samples
% event.duration expressed in samples
% event.timestamp expressed in timestamp units, which vary over systems (optional)

% INSERT STATE StimulusCodeRes 8 0
% start operator.exe 127.0.0.1 --OnConnect "-INSERT STATE StimulusCode 8 0; INSERT STATE StimulusType 1 0" 