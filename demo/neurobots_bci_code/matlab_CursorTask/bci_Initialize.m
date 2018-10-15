function bci_Initialize( in_signal_dims, out_signal_dims )

% Filter initialize demo
% 
% Perform configuration for the bci_Process script.

% BCI2000 filter interface for Matlab
% juergen.mellinger@uni-tuebingen.de, 2005
% $BEGIN_BCI2000_LICENSE$
% 
% This file is part of BCI2000, a platform for real-time bio-signal research.
% [ Copyright (C) 2000-2012: BCI2000 team and many external contributors ]
% 
% BCI2000 is free software: you can redistribute it and/or modify it under the
% terms of the GNU General Public License as published by the Free Software
% Foundation, either version 3 of the License, or (at your option) any later
% version.
% 
% BCI2000 is distributed in the hope that it will be useful, but
%                         WITHOUT ANY WARRANTY
% - without even the implied warranty of MERCHANTABILITY or FITNESS FOR
% A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License along with
% this program.  If not, see <http://www.gnu.org/licenses/>.
% 
% $END_BCI2000_LICENSE$

% this file initilizes BCI2000 in the correct directory and starts testfig
% which is the GUI used by the bci2000
% last edit by Omar Jubran 12.11.2013
% Parameters and states are global variables.



addpath('C:\matlab_offline_toolboxes\matlab_scripts_agball')

% Parameters and states are global variables.
global bci_Parameters;
global carFilter tcpip_obj spatialFilter dataSize tcpip_server ax ax2 plotVars bpf decodeMethod iCz nDNNOutput decodingBuffer nSamples decodingThreshold decodingLabels decimationFactor SR rawDecoding;

%% Variables that need to be adjusted before each experiment
% GpuServerIp = 'localhost'; % local
% GpuServerIp = '172.30.2.92'; % robin
% GpuServerIp = '172.30.0.119'; % Zugspitze
% GpuServerIp = '10.5.166.78'; % gpui
GpuServerIp = '10.5.166.70';% metagpua
% GpuServerIp = '10.5.166.71';% metagpub
% GpuServerIp = '10.5.166.72';% metagpuc
% GpuServerIp = '10.5.166.73';% metagpud
% GpuServerIp = '10.5.166.74';% metagpue
% GpuServerIp = '10.5.166.78';% metagpui

% GpuServerIp = '10.5.166.79';% metagpuj

GpuServerPort = 7989;

refElectrodeLable = 'Cz';
decodeMethod = 1;
%  

nDNNOutput = 5; % Number of classes the DNN makes predictions for.
% REMOVE -1 AFTER ROBINS FIX!!!
decimationFactor = 2;%5;%2;%0;

SR = str2double(bci_Parameters.SamplingRate{1});

if isnan(SR)
    SR = str2double(bci_Parameters.SamplingRate{1}(1:end-2));
end
SR_new = round(SR/decimationFactor);
nChannels = str2double(bci_Parameters.SourceCh);
channelNames = bci_Parameters.ChannelNames;
nSamples = round(str2double(bci_Parameters.SampleBlockSize{1})./decimationFactor);
decodingBuffer = nan(round(SR./nSamples/decimationFactor),nDNNOutput);
decodingThreshold = 0.5;
decodingLabels = [1 2 3 4 5];
rawDecoding = nan(1, nDNNOutput);% REMOVE -1 AFTER ROBINS FIX!!!

% Collect information set to send to the decoder

%% Define variables for the common average reference (car) filter
% carFilter = [];
% carFilter = eye(nChannels) - ones(nChannels)/nChannels;

% change channel sequence
% targetSequence = {'Fp1'; 'Fpz'; 'Fp2'; 'F7'; 'F3'; 'Fz'; 'F4'; 'F8'; 'FC5';...
%     'FC1'; 'FC2'; 'FC6'; 'M1'; 'T7'; 'C3'; 'Cz'; 'C4'; 'T8'; 'M2'; 'CP5';...
%     'CP1'; 'CP2'; 'CP6'; 'P7'; 'P3'; 'Pz'; 'P4'; 'P8'; 'POz'; 'O1'; 'Oz';...
%     'O2'; 'AF7'; 'AF3'; 'AF4'; 'AF8'; 'F5'; 'F1'; 'F2'; 'F6'; 'FC3'; 'FCz';...
%     'FC4'; 'C5'; 'C1'; 'C2'; 'C6'; 'CP3'; 'CPz'; 'CP4'; 'P5'; 'P1'; 'P2'; 'P6';...
%     'PO5'; 'PO3'; 'PO4'; 'PO6'; 'FT7'; 'FT8'; 'TP7'; 'TP8'; 'PO7'; 'PO8'}; %waveguard first 64 channels

targetSequence = {'Fp1', 'Fpz', 'Fp2', 'AF7', 'AF3', 'AF4', 'AF8', 'F7', ...
        'F5', 'F3', 'F1', 'Fz', 'F2', 'F4', 'F6', 'F8', 'FT7', 'FC5', 'FC3', ...
        'FC1', 'FCz', 'FC2', 'FC4', 'FC6', 'FT8', 'M1', 'T7', 'C5', 'C3', ...
        'C1', 'Cz', 'C2', 'C4', 'C6', 'T8', 'M2', 'TP7', 'CP5', 'CP3', ...
        'CP1', 'CPz', 'CP2', 'CP4', 'CP6', 'TP8', 'P7', 'P5', 'P3', 'P1', ...
        'Pz', 'P2', 'P4', 'P6', 'P8', 'PO7', 'PO5', 'PO3', 'POz', 'PO4', ...
        'PO6', 'PO8', 'O1', 'Oz', 'O2'}; %waveguard first 64 channels, robin's topopraphical sequency

spatialFilter = nan(numel(targetSequence), 1);

for ch = 1:numel(targetSequence)
    spatialFilter(ch) =  find(strcmp(channelNames, targetSequence{ch}));
end

channelNames = channelNames(spatialFilter);

%% design filter

%filter before resample
[bpf.d,bpf.c] = butter(20, 40/(SR_new/2), 'low');
bpf.filtConds2 = [];
bpf.dim2 = 2;

%filter after resample
[bpf.b,bpf.a] = butter(10, 40/(SR_new/2), 'low');
bpf.filtConds = [];
bpf.dim = 2;

iCz = find(strcmp(channelNames, refElectrodeLable));


%% create TCP-IP object

tcpip_obj = tcpip(GpuServerIp, GpuServerPort, 'NetworkRole', 'client'); % Zugspitze
tcpip_obj.ByteOrder = 'littleEndian';
tcpip_obj.OutputBufferSize = 40000;

fopen(tcpip_obj);

%% UI server

tcpip_server =tcpip('0.0.0.0', 30000, 'NetworkRole', 'server'); 
tcpip_server.ByteOrder = 'littleEndian';

if strcmp(tcpip_server.Status,'closed') == 1
    fopen(tcpip_server);
    disp('Server connection opened!');
end
disp('... waiting for input');

%% send infos

% channel names
chanString = channelNames{1};
for i = 2:numel(channelNames)
    chanString = [chanString ' ' channelNames{i}];
end

chanString = [chanString ' marker']; %add marker channel
fprintf(tcpip_obj, chanString, 'sync');

dataSize = [numel(spatialFilter)+1, nSamples]; % with marker channel
fwrite(tcpip_obj,  int32(dataSize(1)), 'int32');
fwrite(tcpip_obj, int32(dataSize(2)), 'int32');
% fwrite(tcpip_obj, int32(125), 'int32');% Replace by line above after robins fix.

%% Plot
figure('units', 'normalized', 'Position', [0.73 0.5 0.25 0.4], 'Name', 'Predictions', 'color', 'white');
ax = axes();
bar(ax, [0,0,0,0,0]);
set(ax, 'xticklabel', ['  Right '; '  Feet  '; 'Rotation'; ' Words  '; '  Rest  ']);
ylim([0 1]);

