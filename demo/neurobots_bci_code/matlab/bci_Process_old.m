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
    decodingLabels decimationFactor SR SR_new rawDecoding ...
    menuNavigationSignalCient requestNavigationSignal ...
    cueDisplayCounter maxTrainingImagination minTrainingImagination ...
    maxTrainingPause minTrainingPause longTrainingPauseCounter ...
    initialWaitCounter trainingBlockCounter pauseBlockCounter ...
    longTrainingPauseFreqeuncy pp_adress randomCue cueDisplayBlocks useROS ...
    timeoutValue;

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

%send
try
    fopen(tcpip_obj);
end
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


if decodeMethod == 0% No decoding, simple GUI training
    
    bar(ax,[pauseBlockCounter cueDisplayCounter trainingBlockCounter])
    set(ax, 'xticklabel', {num2str(pauseBlockCounter); num2str(cueDisplayCounter); num2str(trainingBlockCounter)});
    ylim([-1 30])
    drawnow
    if useROS
        %% If we had at least a 1s pause we can now display the cue using ROS.
        if pauseBlockCounter == 0 && initialWaitCounter <= 0 && trainingBlockCounter < 0 && cueDisplayCounter < 0
            %% ROS communication with planner, get menu state
            menuState = call(menuNavigationSignalCient,requestNavigationSignal);
            possibleDirections = [find([menuState.Up, menuState.Down, menuState.Right, menuState.Left])];
            tmp = [];
            for iDirection = 1:numel(possibleDirections)
                switch possibleDirections(iDirection)
                    case 1
                        tmp = [tmp 10];% 6 10];
                    case 2
                        tmp = [tmp 4];% 7];
                    case 3
                        tmp = [tmp 1];% 5];
                    case 4
                        tmp = [tmp 8];% 2];
                    case 5
                        tmp = [tmp 0];% 3 9];
                end
            end
            randomCue = tmp(randi(numel(tmp)));% We probably only need from [1 4 8 10]
            sprintf('Random cue is %i',randomCue)
            switch randomCue
                case 0% No action possible
                    focusSwapRequest.Filename = '';
                case 1% 'rightarrow'
                    focusSwapRequest.Filename = '_right_white.bmp';
                case 2% 'leftarrow'
                    focusSwapRequest.Filename = '_left_white.bmp';
                case 3% 'uparrow'
                    focusSwapRequest.Filename = '_up_white.bmp';
                case 4% 'downarrow'
                    focusSwapRequest.Filename = '_down_white.bmp';
                case 5 % 'face'
                    focusSwapRequest.Filename = 'face_smile.bmp';
                case 6% 'navigation'
                    focusSwapRequest.Filename = 'navigation.bmp';
                case 7% 'music'
                    focusSwapRequest.Filename = 'note.bmp';
                case 8% 'rotation'
                    focusSwapRequest.Filename = 'rotation.bmp';
                case 9% 'subtraction'
                    focusSwapRequest.Filename = 'subtraction.bmp';
                case 10 % 'words'
                    focusSwapRequest.Filename = 'words.bmp';
            end
            result = call(focusSwapSignalClient,focusSwapRequest);
            outp_64(pp_adress,randomCue)
            pause(0.00001)% 10mus
            outp_64(pp_adress,0)
            bci_States.StimulusCode = double(randomCue);
            cueDisplayCounter = cueDisplayBlocks;% ideally 500ms, depends on block size and loop timing...
            trainingBlockCounter = randi([minTrainingImagination, maxTrainingImagination]);% pseudo not possible
            if longTrainingPauseCounter == 0
                pauseBlockCounter = maxTrainingPause;
                longTrainingPauseCounter = longTrainingPauseFreqeuncy;
            elseif longTrainingPauseCounter > 0
                pauseBlockCounter = minTrainingPause;
                longTrainingPauseCounter = longTrainingPauseCounter - 1;
            end
        elseif trainingBlockCounter < 0 && cueDisplayCounter < 0
            initialWaitCounter = initialWaitCounter - 1;% Will get negative!!
            cueDisplayCounter = -1;
            if pauseBlockCounter > 0
                pauseBlockCounter = pauseBlockCounter - 1;
                %         else
                %             pauseBlockCounter = -1;
            end
        end
        
        %% Remove cue after 500ms
        if cueDisplayCounter == 0% Cue display time is over
            focusSwapRequest.Filename = '';
            result = call(focusSwapSignalClient,focusSwapRequest);
            outp_64(pp_adress,randomCue+10)
            pause(0.00001)% 10mus
            outp_64(pp_adress,0)
            bci_States.StimulusCode = randomCue+10;
            cueDisplayCounter = -1;% Make sure we do not trigger this if block when no cure is displayed
        elseif cueDisplayCounter > 0% Cue display time is not over yet
            cueDisplayCounter = cueDisplayCounter -1;% Decrease counter
            bci_States.StimulusCode = randomCue;% Make state only
        else
            bci_States.StimulusCode = 0;
        end
        
        % The next block will be triggered randomly every 1-7s. The duration of
        % the training imagination is defined upon cue display.
        %% Do cued action after random interval passed and mark block (TODO THIS CODE SHOULD BE VERY CLOSE TO BLOCK START TO HAVE GOOD MARKER!!)
        if trainingBlockCounter == 0
            switch randomCue
                case {6, 10}% 'navigation' 'words'
                    guiControlRequest.Direction = int8(1);% up
                case {4, 7}% 'downarrow' 'music'
                    guiControlRequest.Direction = int8(2);% down
                case {1, 5}% 'rightarrow' 'face'
                    guiControlRequest.Direction = int8(3);% select
                case {2, 8} % 'leftarrow' 'rotation'
                    guiControlRequest.Direction = int8(4);% abort
                    %     case 'rightarrow' | 'return'% abort/down-contextsensitive
                    %         guiControlRequest.Direction = int8(5);
                case {0, 3, 9} % 'rest' 'subtraction'
                    guiControlRequest.Direction = int8(0);
            end
            call(menuControlSignalClient,guiControlRequest)
            % bci_States.decodingResult = double(guiControlRequest.Direction);
            outp_64(pp_adress,randomCue+20)% WARNING INT8 ONLY GOES UP TO 127!!!
            pause(0.00001)% 10mus
            outp_64(pp_adress,0)
            bci_States.decodingResult = randomCue+20;
            trainingBlockCounter = -1;
        elseif trainingBlockCounter > 0
            trainingBlockCounter = trainingBlockCounter - 1;
            bci_States.decodingResult = 0;
        else
            bci_States.decodingResult = 0;
        end
    end
    
elseif decodeMethod == 1 % Lukas' method: control max 1 time per second & only if the decodings in this second were the same and above decodingThreshold
    
    
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
        if useROS
            switch unique(integerDecodingBuffer)
                case 4% 'words'
                    guiControlRequest.Direction = int8(1);% up
                case 2% 'downarrow'
                    guiControlRequest.Direction = int8(2);% down
                case 1% 'rightarrow'
                    guiControlRequest.Direction = int8(3);% select
                case 3% 'rotation'
                    guiControlRequest.Direction = int8(4);% abort
                    %     case 'rightarrow' | 'return'% abort/down-contextsensitive
                    %         guiControlRequest.Direction = int8(5);
                case 5 %rest
                    guiControlRequest.Direction = int8(0);
                    bci_States.decodingResult = double(5); %rest
            end
            call(menuControlSignalClient,guiControlRequest)
            bci_States.decodingResult = double(guiControlRequest.Direction);
        end
        outp_64(pp_adress,unique(integerDecodingBuffer))
        pause(0.00001)% 10mus
        outp_64(pp_adress,0)
        
        %     out_signal = decodingBuffer;
        
        if onlineAdaptation
            pause(0.01); %wait for a moment for planner to get changes
            
            
            % switch fixation image
            if useROS
                % nextPicInd = randi(nDNNOutput,1);
                menu_state = call(menuNavigationSignalCient,requestNavigationSignal);
                possibleDirections = find([menu_state.Up, menu_state.Down, menu_state.Right, menu_state.Left]);
                nDirections = numel(possibleDirections);
                nextPicInd = possibleDirections(randi(nDirections, 1));
                switch nextPicInd
                    case 2% 'downarrow'
                        focusSwapRequest.Filename = '_down_white.bmp';
                        currentGoal = 2;
                    case 1% 'rightarrow'
                        focusSwapRequest.Filename = '_right_white.bmp';
                        currentGoal = 1;
                    case 3% 'rotation'
                        focusSwapRequest.Filename = 'rotation.bmp';
                        currentGoal = 3;
                    case 4% 'words'
                        focusSwapRequest.Filename = 'word.bmp';
                        currentGoal = 4;
                    case 5 %rest
                        focusSwapRequest.Filename = '';
                        currentGoal = 5;
                end
                result = call(focusSwapSignalClient,focusSwapRequest);
                outp_64(pp_adress,nextPicInd)
                pause(0.00001)% 10mus
                outp_64(pp_adress,0)
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
            currentGoal = 0;
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
                    if useROS
                        switch maxInd
                            case 2% 'downarrow'
                                guiControlRequest.Direction = int8(2);% down
                            case 1% 'rightarrow'
                                guiControlRequest.Direction = int8(3);% select
                            case 3% 'rotation'
                                guiControlRequest.Direction = int8(4);% abort
                                %     case 'rightarrow' | 'return'% abort/down-contextsensitive
                                %         guiControlRequest.Direction = int8(5);
                            case 4% 'words'
                                guiControlRequest.Direction = int8(1);% up
                            case 5 %rest
                                guiControlRequest.Direction = int8(0);
                                bci_States.decodingResult = double(5); %rest
                        end
                        call(menuControlSignalClient,guiControlRequest)
                        bci_States.decodingResult = double(guiControlRequest.Direction);
                    end
                    outp_64(pp_adress,maxInd)
                    pause(0.00001)% 10mus
                    outp_64(pp_adress,0)
                    
                    %     out_signal = decodingBuffer;
                    
                    if onlineAdaptation
                        
                        pause(0.01); %wait for a moment for planner to get changes
                        
                        % switch fixation image
                        if useROS
                            % nextPicInd = randi(nDNNOutput,1);
                            menu_state = call(menuNavigationSignalCient,requestNavigationSignal);
                            possibleDirections = find([menu_state.Up, menu_state.Down, menu_state.Right, menu_state.Left]);
                            nDirections = numel(possibleDirections);
                            nextPicInd = possibleDirections(randi(nDirections, 1));
                            
                            switch nextPicInd
                                case 4% 'words'
                                    focusSwapRequest.Filename = 'words.bmp';
                                    currentGoal = 4;
                                case 2% 'downarrow'
                                    focusSwapRequest.Filename = '_down_white.bmp';
                                    currentGoal = 2;
                                case 1% 'rightarrow'
                                    focusSwapRequest.Filename = '_right_white.bmp';
                                    currentGoal = 1;
                                case 3% 'rotation'
                                    focusSwapRequest.Filename = 'rotation.bmp';
                                    currentGoal = 3;
                                case 5 %rest
                                    focusSwapRequest.Filename = '';
                                    currentGoal = 5;
                            end
                            result = call(focusSwapSignalClient,focusSwapRequest);
                            outp_64(pp_adress,nextPicInd)
                            pause(0.00001)% 10mus
                            outp_64(pp_adress,0)
                        end
                        
                    end
                    
                    % Reset decodingBuffer to 1 seconds
                    decodingBuffer = nan(round(SR./nSamples/decimationFactor*nSeconds),nDNNOutput);
                    
                    % set timeout Counter
                    timeoutCounter = timeoutValue;
                    
                    
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