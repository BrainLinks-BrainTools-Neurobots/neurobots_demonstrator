function out_signal = bci_Process( in_signal )

% Apply a filter to in_signal, and return the result in out_signal.
% Signal dimensions are ( channels x samples ).
% This file is part of BCI2000, a platform for real-time bio-signal research.
% [ Copyright (C) 2000-2012: BCI2000 team and many external contributors ]

% Get class probabilities from decoder
% Integrate class probabilities
% Trigger action

% Parameters and states are global variables.
global bci_Parameters bci_States;
global carFilter tcpip_obj spatialFilter dataSize onlineAdaptation currentGoal ...
    tcpip_server ax ax1 ax2 nDNNOutput focusSwapSignalClient focusSwapRequest ...
    plotVars nSeconds timeoutCounter bpf decodeMethod classNames nSamples ...
    iCz menuControlSignalClient guiControlRequest decodingBuffer decodingThreshold ...
    decodingLabels decimationFactor SR SR_new rawDecoding ...
    menuNavigationSignalCient requestNavigationSignal ...
    cueDisplayCounter maxTrainingImagination minTrainingImagination ...
    maxTrainingPause minTrainingPause longTrainingPauseCounter ...
    initialWaitCounter trainingBlockCounter pauseBlockCounter ...
    longTrainingPauseFrequency pp_adress randomCue cueDisplayBlocks useROS ...
    timeoutValue nextActionIsAbort useGpuServer errorRate accuracyCounter ...
    pThreshold commandSent output fixationMidpointDiameterCient ...
    requestFixationMidpointDiameter omnirobOnly;

% Now that the stimulus has been sent we can process the data of the
% previous block and send it to the decoder
% EDIT: This actually needs to be done first in the process loop as else
% the wrong marker is sent! Markers set in this block apply to the next
% block. We tested it using the SourceTime state as input to another state
% and lo the states were consistenly shifted by one block. Concurently the
% states of the first block cannot be set within matlab.

if useGpuServer
    %apply spatial filter to data
    filtData = in_signal(spatialFilter, :);
    
    %filter before resample
    [filtData, bpf.filtConds2] = filter(bpf.d, bpf.c,filtData, bpf.filtConds2, bpf.dim2);
    
    %% Downsample
    filtData = resample(filtData', SR./decimationFactor, SR, 0)';
    % Setting the order of the anti-aliasing filter to 0 is important here
    % as else the data is modified relative to the offline training!
    
%    %%filter after resample
%     [filtData, bpf.filtConds] = filter(bpf.b, bpf.a,filtData, bpf.filtConds, bpf.dim);
    % We have decided that filtering after the resample is bullshit.
    
%     %% re-reference to Cz
    % filtData = bsxfun(@minus, filtData, filtData(iCz,:));
    
    %% add marker channel
    filtData = [filtData; ones(1, size(filtData,2))*currentGoal];% wrong size of data was given here, downsample forgotten
    % It is important that we use currentGoal before modifying it within
    % this block as the data we got are actually from the previous block.
    % As currentGoal is global its value is still the one of the previous
    % block.
    bci_States.subjectTarget = double(currentGoal); % states have to be doubles
    
    %% send data via tcp / ip to decoder
    %reshape data
    data_reshape = single(reshape(filtData, size(filtData,1).*size(filtData,2),1));
    %send
    try
        fopen(tcpip_obj);
    end
    fwrite(tcpip_obj, data_reshape, 'float32');
end

% To get best possible timing, display any stimulus should be at begining
% of the block.
% But that is actually not the best thing to do! It is probably more
% importatn to get the data from the decoder first to perform any action as
% close as possible to the brain activity that elicited it!!
%% get data from decoder
if commandSent
    currentGoal = 0;
end
out_signal = [0,0];
iDecoding = 0;
if useGpuServer
    rawDecoding = nan(tcpip_server.BytesAvailable,size(decodingLabels,2));% Lukas: INITIALIZE FOR SPEED!
    while tcpip_server.BytesAvailable > 0
        dataString=fscanf(tcpip_server);
        tmp = str2num(dataString); %#ok<ST2NM>
        if size(tmp,2) == size(decodingLabels,2)% REMOVE -1 after fix Robin!
            iDecoding = iDecoding + 1;
            rawDecoding(iDecoding,:) = tmp;% force output to be a horizontal vector
            % Visualize class probabilities
            bar(ax, tmp);
            set(ax, 'xticklabel', ['  Right '; '  Feet  '; 'Rotation'; ' Words  '; '  Rest  ']);
            ylim(ax,[0 1]);
            title(ax,['Current goal is ' num2str(currentGoal)])
            drawnow;
        end
    end
else
    tmp = rand(1,size(decodingLabels,2));
    iDecoding = iDecoding + 1;
    rawDecoding(iDecoding,:) = tmp;% force output to be a horizontal vector
    % Visualize class probabilities
    bar(ax, tmp);
    set(ax, 'xticklabel', ['  Right '; '  Feet  '; 'Rotation'; ' Words  '; '  Rest  ']);
    ylim(ax,[0 1]);
    drawnow;
end
tmp = isnan(rawDecoding);% Get initialized entries which were not filled
rawDecoding(tmp(:,1),:) = [];% Remove unfilled initialized entries

if decodeMethod == 0 && useROS == 1% No decoding, simple GUI training
    %% If we had at least a 1s pause we can now display the cue using ROS.
    if pauseBlockCounter == 0 && trainingBlockCounter < 0 && cueDisplayCounter < 0
        %% ROS communication with planner, get menu state
        if ~omnirobOnly
        menuState = call(menuNavigationSignalCient,requestNavigationSignal);
        else
            menuState.Up = randi(2)-1;
            menuState.Down = randi(2)-1;
            menuState.Right = randi(2)-1;
            menuState.Left = randi(2)-1;
        end
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
        if nextActionIsAbort
            randomCue = 8;
        else
            randomCue = tmp(randi(numel(tmp)));% We probably only need from [1 4 8 10]
        end
        sprintf('Random cue is %i',randomCue)
        currentGoal = randomCue;
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
        WaitSecs(10*10^-6)% 10mus, pause takes at least 0.001s, waitsecs 0.0001s
        outp_64(pp_adress,0)
        bci_States.StimulusCode = double(randomCue);
        cueDisplayCounter = cueDisplayBlocks;% ideally 500ms, depends on block size and loop timing...
        trainingBlockCounter = randi([minTrainingImagination, maxTrainingImagination]);% pseudo not possible
        pauseBlockCounter = -1;
        if menuState.Up == 0 && menuState.Down == 0 && menuState.Right == 1 && menuState.Left == 0
            nextActionIsAbort = 1;
        else
            nextActionIsAbort = 0;
        end
    elseif pauseBlockCounter > 0 && trainingBlockCounter < 0
        pauseBlockCounter = pauseBlockCounter - 1;
    end
    
    %% Remove cue after 500ms
    if cueDisplayCounter == 0% Cue display time is over
        focusSwapRequest.Filename = '';
        result = call(focusSwapSignalClient,focusSwapRequest);
        outp_64(pp_adress,randomCue+10)
        WaitSecs(10*10^-6)% 10mus, pause takes at least 0.001s, waitsecs 0.0001s
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
        requestFixationMidpointDiameter.Diameter = 10;
        call(fixationMidpointDiameterCient,requestFixationMidpointDiameter);
        % WaitSecs(0.010); % Not needed as code slow enough
        requestFixationMidpointDiameter.Diameter = 0;
        call(fixationMidpointDiameterCient,requestFixationMidpointDiameter);

        if rand <= errorRate
            tmp(tmp == randomCue) = [];% Remove current cue
            if ~isempty(tmp)
                randomCue = tmp(randi(numel(tmp)));% Get new random cue
            else% This can only happen in plan execution mode so make either down, up or back
                alternatives = [4,8,10];% [6,10,4,7,2,8];
                randomCue = alternatives(randi(numel(alternatives)));
            end
            accuracyCounter(1,2) = accuracyCounter(1,2) +1;
        else
            accuracyCounter(1,1) = accuracyCounter(1,1) +1;
        end
        switch randomCue
            case {6, 10}% 'navigation' 'words'
                guiControlRequest.Direction = int8(1);% up
                call(menuControlSignalClient,guiControlRequest)
            case {4, 7}% 'downarrow' 'music'
                guiControlRequest.Direction = int8(2);% down
                call(menuControlSignalClient,guiControlRequest)
            case {1, 5}% 'rightarrow' 'face'
                guiControlRequest.Direction = int8(3);% select
                call(menuControlSignalClient,guiControlRequest)
            case {2, 8} % 'leftarrow' 'rotation'
                guiControlRequest.Direction = int8(4);% abort
                call(menuControlSignalClient,guiControlRequest)
                %     case 'rightarrow' | 'return'% abort/down-contextsensitive
                %         guiControlRequest.Direction = int8(5);
            case {0, 3, 9} % 'rest' 'subtraction'
                %                 guiControlRequest.Direction = int8([]);% If 0 is given crash!!
        end
        % call(menuControlSignalClient,guiControlRequest)
        % bci_States.decodingResult = double(randomCue);
        outp_64(pp_adress,randomCue+20)% WARNING INT8 ONLY GOES UP TO 127!!!
        WaitSecs(10*10^-6)% 10mus, pause takes at least 0.001s, waitsecs 0.0001s
        outp_64(pp_adress,0)
        bci_States.decodingResult = randomCue+20;
        trainingBlockCounter = -1;
        currentGoal = 0;
        if longTrainingPauseCounter == 0
            pauseBlockCounter = maxTrainingPause;
            longTrainingPauseCounter = longTrainingPauseFrequency;
        elseif longTrainingPauseCounter > 0
            pauseBlockCounter = minTrainingPause;
            longTrainingPauseCounter = longTrainingPauseCounter - 1;
        end
    elseif trainingBlockCounter > 0
        trainingBlockCounter = trainingBlockCounter - 1;
        bci_States.decodingResult = 0;
    else
        bci_States.decodingResult = 0;
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
                    call(menuControlSignalClient,guiControlRequest)
                case 2% 'downarrow'
                    guiControlRequest.Direction = int8(2);% down
                    call(menuControlSignalClient,guiControlRequest)
                case 1% 'rightarrow'
                    guiControlRequest.Direction = int8(3);% select
                    call(menuControlSignalClient,guiControlRequest)
                case 3% 'rotation'
                    guiControlRequest.Direction = int8(4);% abort
                    call(menuControlSignalClient,guiControlRequest)
                    %     case 'rightarrow' | 'return'% abort/down-contextsensitive
                    %         guiControlRequest.Direction = int8(5);
                case 5 %rest
                    %                     guiControlRequest.Direction = int8([]);% If 0 is given crash!!
            end
            %             call(menuControlSignalClient,guiControlRequest)
            bci_States.decodingResult = double(maxInd)+20;
        end
        outp_64(pp_adress,unique(integerDecodingBuffer))
        WaitSecs(10*10^-6)% 10mus, pause takes at least 0.001s, waitsecs 0.0001s
        outp_64(pp_adress,0)
        
        %     out_signal = decodingBuffer;
        
        
        
        % Reset decodingBuffer
        decodingBuffer = nan(size(decodingBuffer));
        
    else
        bci_States.decodingResult = double(0);
        %     out_signal = decodingBuffer;
        %     out_signal = [0,0];
        
    end
    
elseif decodeMethod == 2 % control max 1/s; send control signal if values in highest decoding class are significantly > decodingThreshold (ttest, p<0.01)
    
    % Need checks to make sure that we are decoding after the pause and
    % after the cue, if adaptation is on
    bci_States.decodingResult = double(0);
    
    % Fill  buffer if decoder is sendind data
    if ~isempty(rawDecoding)
        nDecodings = size(rawDecoding,1);
        % Only fill buffer if decoding is going to be allowed within 1
        % second
        if trainingBlockCounter > 0 || ~onlineAdaptation% In training. Stimulus state is not relevant
            if any(any(isnan(decodingBuffer))) % as long as there are nans (last decoding / start < 1s before), discard those first
                decodingBuffer = [decodingBuffer(nDecodings+1:end,:); rawDecoding] ;
            else % no nans, make buffer bigger...
                decodingBuffer = [decodingBuffer; rawDecoding] ;
            end
            trainingBlockCounter = trainingBlockCounter-1;
        end
        
        if ~any(any(isnan(decodingBuffer)))% test for significance if there are no nans anymore in the buffer and decoding is allowed
            [maxVal, maxInd] = max(mean(decodingBuffer,1)); % find max
            
            if maxVal > decodingThreshold || trainingBlockCounter == 0% only go further if mean of max class is above decodingThreshold
                [h,p] = ttest2(decodingBuffer(:,maxInd), decodingThreshold);
                %                  p = weightedTTest(decodingBuffer(:,maxInd), decodingThreshold, 0.9);
                
                %plot pvalues
                bar(ax2, p);
                set(ax2, 'xticklabel', ['pValues']);
                title(ax2,{[num2str(classNames{maxInd})]; ['buffer Size = ' num2str(size(decodingBuffer,1))]});
                ylim(ax2,[0 1]);
                drawnow;
                
                if p < pThreshold || trainingBlockCounter == 0 % if significantly lower than treshold, send control signal
                    
                    %% ROS communication with planner
                    if useROS
                        switch maxInd
                            case 1% 'rightarrow'
                                guiControlRequest.Direction = int8(3);% select
                                call(menuControlSignalClient,guiControlRequest);
                            case 2% 'downarrow'
                                guiControlRequest.Direction = int8(2);% down
                                call(menuControlSignalClient,guiControlRequest);
                            case 3% 'rotation'
                                guiControlRequest.Direction = int8(4);% abort
                                call(menuControlSignalClient,guiControlRequest);
                                %     case 'rightarrow' | 'return'% abort/down-contextsensitive
                                %         guiControlRequest.Direction = int8(5);
                            case 4% 'words'
                                guiControlRequest.Direction = int8(1);% up
                                call(menuControlSignalClient,guiControlRequest);
                            case 5 %rest
                                % guiControlRequest.Direction = int8([]);% If one gives 0 error happens!!
                        end
                        % call(menuControlSignalClient,guiControlRequest)
                        requestFixationMidpointDiameter.Diameter = 10;
                        call(fixationMidpointDiameterCient,requestFixationMidpointDiameter);
                        % WaitSecs(0.010); % Not needed as code slow enough
                        requestFixationMidpointDiameter.Diameter = 0;
                        call(fixationMidpointDiameterCient,requestFixationMidpointDiameter);
                        % Lukas: Is it OK to save the GUI command instead of
                        % the class? No, it is not. I changed to maxInd+20.
                        bci_States.decodingResult = double(maxInd)+20;
                    end
                    outp_64(pp_adress,maxInd)
                    WaitSecs(10*10^-6)% 10mus, pause takes at least 0.001s, waitsecs 0.0001s
                    outp_64(pp_adress,0)
                    commandSent = 1;
                    %     out_signal = decodingBuffer;
                    
                    % Reset decodingBuffer to 1 seconds
                    decodingBuffer = nan(round(SR_new./nSamples*nSeconds),nDNNOutput);
                    
                    % Refill counters
                    % Pause counter controlled in adaptation loop
                    if ~onlineAdaptation
                        pauseBlockCounter = minTrainingPause;
                        trainingBlockCounter = maxTrainingImagination;
                    else
                        if currentGoal ~= maxInd
                            accuracyCounter(currentGoal,2) = accuracyCounter(currentGoal,2) +1;
                        else
                            accuracyCounter(currentGoal,1) = accuracyCounter(currentGoal,1) +1;
                        end
                        trainingBlockCounter = -1;% No longer in training
                    end
                end
            else
                commandSent = 0;
            end
        else
            commandSent = 0;
            %plot pvalues
            bar(ax2, 1);
            set(ax2, 'xticklabel', ['pValues']);
            title(ax2,{['Waiting']; ['buffer Size = ' num2str(size(decodingBuffer,1))]});
            ylim(ax2,[0 1]);
            drawnow;
        end
    else
        commandSent = 0;
    end
    
    nSampleThreshold = maxTrainingImagination;
    if size(decodingBuffer,1) > nSampleThreshold
        decodingBuffer(1:(end-nSampleThreshold),:) = [];
    end
    
end

% Now that the action triggered by the brain signals has been activated as
% close as possible to the brain signals occurence we can display a
% stimulus, if the time is right.
% The time is right when the last visual event in the GUI, be it stimulus
% display or decoding event, is at least 1s past.
if onlineAdaptation && useROS && decodeMethod~=0
    %% If we had at least a 1s pause we can now display the cue using ROS.
    if pauseBlockCounter == 0 && trainingBlockCounter < 0 && cueDisplayCounter < 0
        %% ROS communication with planner, get menu state
        if ~omnirobOnly
        menuState = call(menuNavigationSignalCient,requestNavigationSignal);
        else
            menuState.Up = randi(2)-1;
            menuState.Down = randi(2)-1;
            menuState.Right = randi(2)-1;
            menuState.Left = randi(2)-1;
        end
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
        if nextActionIsAbort
            randomCue = 8;
        else
            randomCue = tmp(randi(numel(tmp)));% We probably only need from [1 4 8 10]
        end
        sprintf('Random cue is %i',randomCue)
        switch randomCue
            case 0% No action possible
                focusSwapRequest.Filename = '';
                currentGoal = 5;
            case 1% 'rightarrow'
                focusSwapRequest.Filename = '_right_white.bmp';
                currentGoal = 1;
            case 2% 'leftarrow'
                focusSwapRequest.Filename = '_left_white.bmp';
            case 3% 'uparrow'
                focusSwapRequest.Filename = '_up_white.bmp';
            case 4% 'downarrow'
                focusSwapRequest.Filename = '_down_white.bmp';
                currentGoal = 2;
            case 5 % 'face'
                focusSwapRequest.Filename = 'face_smile.bmp';
            case 6% 'navigation'
                focusSwapRequest.Filename = 'navigation.bmp';
            case 7% 'music'
                focusSwapRequest.Filename = 'note.bmp';
            case 8% 'rotation'
                focusSwapRequest.Filename = 'rotation.bmp';
                currentGoal = 3;
            case 9% 'subtraction'
                focusSwapRequest.Filename = 'subtraction.bmp';
            case 10 % 'words'
                focusSwapRequest.Filename = 'words.bmp';
                currentGoal = 4;
            otherwise
                currentGoal = 0;
        end
        result = call(focusSwapSignalClient,focusSwapRequest);
        outp_64(pp_adress,randomCue)
        WaitSecs(10*10^-6)% 10mus, pause takes at least 0.001s, waitsecs 0.0001s
        outp_64(pp_adress,0)
        bci_States.StimulusCode = double(randomCue);
        cueDisplayCounter = cueDisplayBlocks;% ideally 500ms, depends on block size and loop timing...
        trainingBlockCounter = randi([minTrainingImagination, maxTrainingImagination]);% pseudo not possible
        if longTrainingPauseCounter == 0
            pauseBlockCounter = maxTrainingPause;
            longTrainingPauseCounter = longTrainingPauseFrequency;
        elseif longTrainingPauseCounter > 0
            pauseBlockCounter = minTrainingPause;
            longTrainingPauseCounter = longTrainingPauseCounter - 1;
        end
        if menuState.Up == 0 && menuState.Down == 0 && menuState.Right == 1 && menuState.Left == 0
            nextActionIsAbort = 1;
        else
            nextActionIsAbort = 0;
        end
    elseif pauseBlockCounter > 0 && trainingBlockCounter < 0 && cueDisplayCounter < 0
        pauseBlockCounter = pauseBlockCounter - 1;
    end
    
    %% Remove cue after 500ms
    if cueDisplayCounter == 0% Cue display time is over
        focusSwapRequest.Filename = '';
        result = call(focusSwapSignalClient,focusSwapRequest);
        outp_64(pp_adress,randomCue+10)
        WaitSecs(10*10^-6)% 10mus, pause takes at least 0.001s, waitsecs 0.0001s
        outp_64(pp_adress,0)
        bci_States.StimulusCode = randomCue+10;
        cueDisplayCounter = -1;% Make sure we do not trigger this if block when no cure is displayed
    elseif cueDisplayCounter > 0% Cue display time is not over yet
        cueDisplayCounter = cueDisplayCounter -1;% Decrease counter
        bci_States.StimulusCode = randomCue;% Make state only
    else
        bci_States.StimulusCode = 0;
    end
end

output = vertcat(output, horzcat(currentGoal, commandSent));
% %% Plot timing
% bar(ax1,[longTrainingPauseCounter pauseBlockCounter cueDisplayCounter trainingBlockCounter currentGoal])
% set(ax1, 'xticklabel', {['Long pause in ' num2str(longTrainingPauseCounter)];...
%     ['Pause ends in ' num2str(pauseBlockCounter)]; ['Cue off in ' num2str(cueDisplayCounter)];...
%     ['Training off in ' num2str(trainingBlockCounter)]; ['Current goal ' num2str(currentGoal)]});
% ylim(ax1,[-1 30])
% drawnow