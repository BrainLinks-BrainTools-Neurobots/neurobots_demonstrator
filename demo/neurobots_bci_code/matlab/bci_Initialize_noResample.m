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
global carFilter tcpip_obj spatialFilter dataSize tcpip_server ax ax2 plotVars bpf iCz menuControlSignalClient guiControlRequest decodingBuffer decodingThreshold decodingLabels decimationFactor;

%% Variables that need to be adjusted before each experiment
% GpuServerIP = 'localhost'; % local
% GpuServerIP = '172.30.2.92'; % robin
% GpuServerIP = '172.30.0.119'; % Zugspitze
% GpuServerIp = '10.5.166.78'; % gpui
% GpuServerIp = '10.5.166.70';% metagpua
% GpuServerIp = '10.5.166.71';% metagpub
% GpuServerIp = '10.5.166.72';% metagpuc
% GpuServerIp = '10.5.166.73';% metagpud
% GpuServerIp = '10.5.166.74';% metagpue
GpuServerIp = '10.5.166.78';% metagpui

GpuServerPort = 7986;

refElectrodeLable = 'Cz';
 
RosMasterUri = 'http://10.126.44.213:11311';% needs to have the IP of the current ROS core.
RosIp = '10.126.40.12';% needs to have the correct IP of the ROS client

nDNNOutput = 4;% Number of classes the DNN makes predictions for.

decimationFactor = 10;

SR = str2double(bci_Parameters.SamplingRate{1});
if isnan(SR)
    SR = str2double(bci_Parameters.SamplingRate{1}(1:end-3));
end
nChannels = str2double(bci_Parameters.SourceCh);
channelNames = bci_Parameters.ChannelNames;
nSamples = round(str2double(bci_Parameters.SampleBlockSize{1}));
decodingBuffer = nan(round(SR./nSamples),nDNNOutput);
decodingThreshold = 0.8;
decodingLabels = [1 2 3 4 5];
% Collect information set to send to the decoder

%% Define variables for the common average reference (car) filter
% carFilter = [];
% carFilter = eye(nChannels) - ones(nChannels)/nChannels;

% change channel sequence
targetSequence = {'Fp1'; 'Fpz'; 'Fp2'; 'F7'; 'F3'; 'Fz'; 'F4'; 'F8'; 'FC5';...
    'FC1'; 'FC2'; 'FC6'; 'M1'; 'T7'; 'C3'; 'Cz'; 'C4'; 'T8'; 'M2'; 'CP5';...
    'CP1'; 'CP2'; 'CP6'; 'P7'; 'P3'; 'Pz'; 'P4'; 'P8'; 'POz'; 'O1'; 'Oz';...
    'O2'; 'AF7'; 'AF3'; 'AF4'; 'AF8'; 'F5'; 'F1'; 'F2'; 'F6'; 'FC3'; 'FCz';...
    'FC4'; 'C5'; 'C1'; 'C2'; 'C6'; 'CP3'; 'CPz'; 'CP4'; 'P5'; 'P1'; 'P2'; 'P6';...
    'PO5'; 'PO3'; 'PO4'; 'PO6'; 'FT7'; 'FT8'; 'TP7'; 'TP8'; 'PO7'; 'PO8'}; %waveguard first 64 channels

spatialFilter = nan(numel(targetSequence), 1);

for ch = 1:numel(targetSequence)
    spatialFilter(ch) =  find(strcmp(channelNames, targetSequence{ch}));
end

channelNames = channelNames(spatialFilter);

%% design bandpass filter
[bpf.b,bpf.a] = butter(3, 40/(SR/2), 'low');
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



%% ROS node
try
    rosshutdown
end
% robot ip adress and rosmaster has to be used rosinit('master_host')
setenv('ROS_MASTER_URI',RosMasterUri)
setenv('ROS_IP',RosIp)
rosinit% Can also be called with rosinit(RosMasterUri)
% rosinit('10.126.3.30',11311)
% rosinit('http://ieeg-ginter:11311')
% USPER IMPORTANT!! EDIT THE IP ADDRESS AND HOSTNAME OF THE ROS MASTER IN C:\Windows\System32\drivers\etc\HOSTS

% Create a client to send and recieve data from the ROS service
menuControlSignalClient = rossvcclient('/menu_control_signal');
guiControlRequest = rosmessage(menuControlSignalClient);

