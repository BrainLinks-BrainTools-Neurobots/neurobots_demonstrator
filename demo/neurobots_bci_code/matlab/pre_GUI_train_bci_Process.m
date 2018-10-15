function out_signal = bci_Process( in_signal )

% Apply a filter to in_signal, and return the result in out_signal.
% Signal dimensions are ( channels x samples ).
% This file is part of BCI2000, a platform for real-time bio-signal research.
% [ Copyright (C) 2000-2012: BCI2000 team and many external contributors ]


% Parameters and states are global variables.
global bci_Parameters bci_States;
global carFilter tcpip_obj spatialFilter dataSize onlineAdaptation currentGoal ...
    tcpip_server ax ax2 nDNNOutput focusSwapSignalClient focusSwapRequest ...
    plotVars nSeconds timeoutCounter bpf decodeMethod classNames nSamples ...
    iCz menuControlSignalClient guiControlRequest decodingBuffer decodingThreshold ...
    decodingLabels decimationFactor SR rawDecoding ...
    menuNavigationSignalCient requestNavigationSignal;


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
filtData = [filtData; ones(1, size(filtData,(2)))*currentGoal];% wrong size of data was given here, downsample forgotten 

bci_States.subjectTarget = double(currentGoal); % states have to be doubles

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
        ylim(ax,[0 1]);
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
                guiControlRequest.Direction = int8(1);% up 
                call(menuControlSignalClient,guiControlRequest)
                bci_States.decodingResult = double(guiControlRequest.Direction);
            case 2% 'downarrow'
                guiControlRequest.Direction = int8(2);% down 
                call(menuControlSignalClient,guiControlRequest)
                bci_States.decodingResult = double(guiControlRequest.Direction);
            case 1% 'rightarrow'
                guiControlRequest.Direction = int8(3);% select
                call(menuControlSignalClient,guiControlRequest)
                bci_States.decodingResult = double(guiControlRequest.Direction);
            case 3% 'rotation'
                guiControlRequest.Direction = int8(4);% abort
                call(menuControlSignalClient,guiControlRequest)
                bci_States.decodingResult = double(guiControlRequest.Direction);
                %     case 'rightarrow' | 'return'% abort/down-contextsensitive
                %         guiControlRequest.Direction = int8(5);
            case 5 %rest
                bci_States.decodingResult = double(5); %rest
        end

    %     out_signal = decodingBuffer;
    
        if onlineAdaptation
            pause(0.01); %wait for a moment for planner to get changes
            
            
            % switch fixation image
            % nextPicInd = randi(nDNNOutput,1);
            menu_state = call(menuNavigationSignalCient,requestNavigationSignal);
            possibleDirections = find([menu_state.Up, menu_state.Down, menu_state.Right, menu_state.Left]);
            nDirections = numel(possibleDirections);
            nextPicInd = possibleDirections(randi(nDirections, 1));
            
            switch nextPicInd
                case 4% 'words'
                    focusSwapRequest.Filename = 'fixationCircle_up.png';
                    result = call(focusSwapSignalClient,focusSwapRequest);
                    currentGoal = 4;
                case 2% 'downarrow'
                    focusSwapRequest.Filename = 'fixationCircle_down.png';
                    result = call(focusSwapSignalClient,focusSwapRequest);
                    currentGoal = 2;
                case 1% 'rightarrow'
                    focusSwapRequest.Filename = 'fixationCircle_right.png';
                    result = call(focusSwapSignalClient,focusSwapRequest);
                    currentGoal = 1;
                case 3% 'rotation'
                    focusSwapRequest.Filename = 'fixationCircle_left.png';
                    result = call(focusSwapSignalClient,focusSwapRequest);
                    currentGoal = 3;
                case 5 %rest
                    focusSwapRequest.Filename = 'fixationCircle_rest.png';
                    result = call(focusSwapSignalClient,focusSwapRequest);
                    currentGoal = 5;
            end
            
        end

        % Reset decodingBuffer
        decodingBuffer = nan(size(decodingBuffer));



    else
        bci_States.decodingResult = double(0);
    %     out_signal = decodingBuffer;
    %     out_signal = [0,0];

    end
elseif decodeMethod == 2 % control max 1/s; send control signal if values in highest decoding class are significantly > decodingThreshold (ttest, p<0.01)
    bci_States.decodingResult = double(0);
    % Fill  buffer
    if ~isempty(rawDecoding)
        nDecodings = size(rawDecoding,1);
        if timeoutCounter == 0 %wait for a timeout (normally 1s) after decoding before collecting predictions again
            if any(any(isnan(decodingBuffer))) % as long as there are nans (last decoding / start < 1s before), discard those first
                decodingBuffer = [decodingBuffer(nDecodings+1:end,:); rawDecoding] ;
            else % no nans, make buffer bigger...
                decodingBuffer = [decodingBuffer; rawDecoding] ;
            end
        else
            timeoutCounter = timeoutCounter-1;
        end
        
        if ~any(any(isnan(decodingBuffer)))  % test for significance if there are no nans anymore in the buffer
             [maxVal, maxInd] = max(mean(decodingBuffer,1)); % find max
             
             if maxVal > decodingThreshold % only go further if mean of max class is above decodingThreshold
                 [h,p] = ttest2(decodingBuffer(:,maxInd), decodingThreshold);
%                  p = weightedTTest(decodingBuffer(:,maxInd), decodingThreshold, 0.9);
                 
                    %plot pvalues                    
                    bar(ax2, p);
                    set(ax2, 'xticklabel', ['pValues']);
                    title(ax2,{[num2str(classNames{maxInd})]; ['buffer Size = ' num2str(size(decodingBuffer,1))]});
                    ylim(ax2,[0 1]);
                    drawnow;
               

                 
                 if p < 0.2 % if significantly lower than treshold, send control signal
                     
                        %% ROS communication with planner
                        switch maxInd
                            case 4% 'words'
                                guiControlRequest.Direction = int8(1);% up 
                                call(menuControlSignalClient,guiControlRequest)
                                bci_States.decodingResult = double(guiControlRequest.Direction);
                            case 2% 'downarrow'
                                guiControlRequest.Direction = int8(2);% down 
                                call(menuControlSignalClient,guiControlRequest)
                                bci_States.decodingResult = double(guiControlRequest.Direction);
                            case 1% 'rightarrow'
                                guiControlRequest.Direction = int8(3);% select
                                call(menuControlSignalClient,guiControlRequest)
                                bci_States.decodingResult = double(guiControlRequest.Direction);
                            case 3% 'rotation'
                                guiControlRequest.Direction = int8(4);% abort
                                call(menuControlSignalClient,guiControlRequest)
                                bci_States.decodingResult = double(guiControlRequest.Direction);
                                %     case 'rightarrow' | 'return'% abort/down-contextsensitive
                                %         guiControlRequest.Direction = int8(5);
                            case 5 %rest
                                bci_States.decodingResult = double(5); %rest
                        end

                    %     out_signal = decodingBuffer;

                        if onlineAdaptation

                             pause(0.01); %wait for a moment for planner to get changes            
            
                            % switch fixation image
                            % nextPicInd = randi(nDNNOutput,1);
                            menu_state = call(menuNavigationSignalCient,requestNavigationSignal);
                            possibleDirections = find([menu_state.Up, menu_state.Down, menu_state.Right, menu_state.Left]);
                            nDirections = numel(possibleDirections);
                            nextPicInd = possibleDirections(randi(nDirections, 1));
                            
                            switch nextPicInd
                                case 4% 'words'
                                    focusSwapRequest.Filename = 'fixationCircle_up.png';
                                    result = call(focusSwapSignalClient,focusSwapRequest);
                                    currentGoal = 4;
                                case 2% 'downarrow'
                                    focusSwapRequest.Filename = 'fixationCircle_down.png';
                                    result = call(focusSwapSignalClient,focusSwapRequest);
                                    currentGoal = 2;
                                case 1% 'rightarrow'
                                    focusSwapRequest.Filename = 'fixationCircle_right.png';
                                    result = call(focusSwapSignalClient,focusSwapRequest);
                                    currentGoal = 1;
                                case 3% 'rotation'
                                    focusSwapRequest.Filename = 'fixationCircle_left.png';
                                    result = call(focusSwapSignalClient,focusSwapRequest);
                                    currentGoal = 3;
                                case 5 %rest
                                    focusSwapRequest.Filename = 'fixationCircle_rest.png';
                                    result = call(focusSwapSignalClient,focusSwapRequest);
                                    currentGoal = 5;
                            end

                        end


                    % Reset decodingBuffer to 1 seconds
                    decodingBuffer = nan(round(SR./nSamples/decimationFactor*nSeconds),nDNNOutput);
                    
                    % set timeout Counter
                    timeoutCounter = 5;
                    
                     
                 end
             end 
        else
            %plot pvalues                    
            bar(ax2, 1);
            set(ax2, 'xticklabel', ['pValues']);
            title(ax2,{['Waiting']; ['buffer Size = ' num2str(size(decodingBuffer,1))]});
            ylim(ax2,[0 1]);
            drawnow;
        end                    
    end

    nSampleThreshold = (round(SR./nSamples/decimationFactor)*4);
    if size(decodingBuffer,1) >= nSampleThreshold
%         decodingBuffer = nan(round(SR./nSamples/decimationFactor),nDNNOutput);
        decodingBuffer(1:(end-nSampleThreshold+1),:) = [];
    end
    
    
    
end



end