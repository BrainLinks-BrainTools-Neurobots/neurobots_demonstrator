function out_signal = bci_Process( in_signal )

% Apply a filter to in_signal, and return the result in out_signal.
% Signal dimensions are ( channels x samples ).
% This file is part of BCI2000, a platform for real-time bio-signal research.
% [ Copyright (C) 2000-2012: BCI2000 team and many external contributors ]

%% TODOS
% Clean up
% Implement saving of decoding output into bci_States
% Implement Eyelink for offline 
% 
% Online Eyelink control of paradigm cannot be done with BCI2000!! To this
% end one would need to implement online EEG data acquisition and the ball
% paradigm in Psychtoolbox.

% Parameters and states are global variables.
global bci_Parameters bci_States;
global carFilter tcpip_obj spatialFilter dataSize tcpip_server ax nDNNOutput ax2 plotVars decodeMethod bpf nSamples iCz decodingBuffer decodingThreshold decodingLabels decimationFactor SR rawDecoding;

% % Debugging flush save
% globalVars = who('global');
% for iVar = 1:numel(globalVars)
%     eval(sprintf('global %s', globalVars{iVar}));
% end
% save

% Run the car filter for each time point over all channels
% raw_data = carFilter*in_signal;




%apply spatial filter to data
filtData = in_signal(spatialFilter, :);

%filter before resample
[filtData, bpf.filtConds2] = filter(bpf.d, bpf.c,filtData, bpf.filtConds2, bpf.dim2);



%% Downsample
filtData = resample(filtData', SR./decimationFactor, SR, 0)';


%filter after resample
[filtData, bpf.filtConds] = filter(bpf.b, bpf.a,filtData, bpf.filtConds, bpf.dim);
%% re-reference to Cz
% filtData = bsxfun(@minus, filtData, filtData(iCz,:));

%% add marker channel
if bci_States.Feedback == 1
    filtData = [filtData; ones(1, size(filtData,(2)))*bci_States.TargetCode];%
%     out_signal = bci_States.TargetCode;
else
    filtData = [filtData; ones(1, size(filtData,(2)))*0]; % Pause, send zeros
%     out_signal = 0;
end
%% send data via tcp / ip to decoder

%reshape data
data_reshape = single(reshape(filtData, size(filtData,1).*size(filtData,2),1));

%create marker channel and send it, too. markers 0,1,2,3,4 for pause,right,left,rest,feet
%

%send
fwrite(tcpip_obj, data_reshape, 'float32');


%% get data from decoder
out_signal = [0,0];
rawDecoding = [];
while tcpip_server.BytesAvailable > 0
    dataString=fscanf(tcpip_server);
    tmp = str2num(dataString); %#ok<ST2NM>
    if size(tmp,2) == size(decodingLabels,2)% REMOVE -1 after fix Robin!
        rawDecoding(end+1,:) = tmp;% force output to be a horizontal vector
%         rawDecoding(1,:) = [];
        
        bar(ax, tmp);
        set(ax, 'xticklabel', ['  Right '; '  Feet  '; 'Rotation'; ' Words  '; '  Rest  ']);
        ylim([0 1]);
        drawnow;
               
    end
    
    

end


if decodeMethod == 1 % Lukas' method: control max 1 time per second & only if the decodings in this second were the same and above decodingThreshold
    
    
    % convert to ball movements
    if ~isempty(rawDecoding)
        tmp = mean(rawDecoding,1);
        tmp = tmp*2;
        out_signal = [tmp(1)-tmp(3); tmp(4)-tmp(2)];
    end
    
    % Fill ring buffer
    if ~isempty(rawDecoding)
        nDecodings = size(rawDecoding,1);
        decodingBuffer = [decodingBuffer(nDecodings+1:end,:); rawDecoding] ;
    %     decodingBuffer(1:size(rawDecoding,1),:) = [];
    end

    % Threshold every decoding package into one predicted class
    [integerDecodingBuffer, iDecodingLabels] = max(decodingBuffer,[],2);% Take max of every package
    notDecodedYet = isnan(integerDecodingBuffer);
    predictionOverThreshold = integerDecodingBuffer>decodingThreshold;
    integerDecodingBuffer = decodingLabels(iDecodingLabels);
    integerDecodingBuffer(~predictionOverThreshold) = -1;
    integerDecodingBuffer(notDecodedYet) = nan;

    % Constraining output to once per second
    if ~any(notDecodedYet == 1) && numel(unique(integerDecodingBuffer)) == 1

        %% ROS communication with planner
        switch unique(integerDecodingBuffer)
            case 4% 'words'
    %             guiControlRequest.Direction = int8(1);% up 
    %             call(menuControlSignalClient,guiControlRequest)
            case 2% 'downarrow'
    %             guiControlRequest.Direction = int8(2);% down 
    %             call(menuControlSignalClient,guiControlRequest)
            case 1% 'rightarrow'
    %             guiControlRequest.Direction = int8(3);% select
    %             call(menuControlSignalClient,guiControlRequest)
            case 3% 'rotation'
    %             guiControlRequest.Direction = int8(4);% abort
    %             call(menuControlSignalClient,guiControlRequest)
                %     case 'rightarrow' | 'return'% abort/down-contextsensitive
                %         guiControlRequest.Direction = int8(5);
            otherwise
                % At the moment nothing is done with the pause class.
                % Every subthreshold of not yet fully decoded event lands here.

        end

    %     out_signal = decodingBuffer;

        % Reset decodingBuffer
        decodingBuffer = nan(size(decodingBuffer));



    else
    %     out_signal = decodingBuffer;
    %     out_signal = [0,0];

    end
elseif decodeMethod == 2 % control max 1/s; send control signal if values in highest decoding class are significantly > decodingThreshold (ttest, p<0.01)
    
    % Fill  buffer
    if ~isempty(rawDecoding)
        nDecodings = size(rawDecoding,1);
        if any(any(isnan(decodingBuffer))) % as long as there are nans (last decoding / start < 1s before), discard those first
            decodingBuffer = [decodingBuffer(nDecodings+1:end,:); rawDecoding] ;
        else % no nans, make buffer bigger...
            decodingBuffer = [decodingBuffer; rawDecoding] ;
        end
        
        if ~any(any(isnan(decodingBuffer)))  % test for significance if there are no nans anymore in the buffer
             [maxVal, maxInd] = max(mean(decodingBuffer,1)); % find max
             
             if maxVal > decodingThreshold % only go further if mean of max class is above decodingThreshold
                 [h,p] = ttest2(decodingBuffer(:,maxInd), decodingThreshold);
%                  p = weightedTTest(decodingBuffer(:,maxInd), decodingThreshold, 0.9);
                 
                 if p < 0.1 % if significantly higher than treshold, send control signal
                     
                        %% ROS communication with planner
                        switch maxInd
                            case 4% 'words'
                    %             guiControlRequest.Direction = int8(1);% up 
                    %             call(menuControlSignalClient,guiControlRequest)
%                                 out_signal = [0,100];
                            case 2% 'downarrow'
                    %             guiControlRequest.Direction = int8(2);% down 
                    %             call(menuControlSignalClient,guiControlRequest)
%                                 out_signal = [0,-100];
                            case 1% 'rightarrow'
                    %             guiControlRequest.Direction = int8(3);% select
                    %             call(menuControlSignalClient,guiControlRequest)
%                                 out_signal = [100,0];
                            case 3% 'rotation'
                    %             guiControlRequest.Direction = int8(4);% abort
                    %             call(menuControlSignalClient,guiControlRequest)
                                %     case 'rightarrow' | 'return'% abort/down-contextsensitive
                                %         guiControlRequest.Direction = int8(5);
%                                 out_signal = [-100,0];              

                        end

                    %     out_signal = decodingBuffer;

                    % Reset decodingBuffer
                    decodingBuffer = nan(round(SR./nSamples/decimationFactor),nDNNOutput);
                    %also reset it if it gets too long?...
                    
                     
                 end
             end             
        end                    
    end

     % reset buffer if too long (more than 8 seconds here)
    if size(decodingBuffer,1) >= (round(SR./nSamples/decimationFactor)*6)
%         decodingBuffer = nan(round(SR./nSamples/decimationFactor),nDNNOutput);
        decodingBuffer(1,:) = [];
    end
    
    
end



end