function bci_StopRun

% Filter stop run demo
% 
% Perform parameter updates at the end of a run.

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

% Parameters and states are global variables.
global bci_Parameters bci_States;
global carFilter tcpip_obj focusSwapSignalClient focusSwapRequest ...
    accuracyCounter output useROS useGpuServer classNames;

%close tcp/ip connection
if useGpuServer
    try
        fclose(tcpip_obj);
    end
end

if useROS
    focusSwapRequest.Filename = 'face_smile.bmp';
    result = call(focusSwapSignalClient,focusSwapRequest);    tmp = {sum(accuracyCounter(:)), sum(accuracyCounter(:,1)), sum(accuracyCounter(:,2)), sum(accuracyCounter(:,1))/sum(accuracyCounter(:))*100};
    figure('Name','Run summary'),
    subplot(2,1,1),bar([tmp{:}]),title('Run summary');
    set(gca,'xticklabels',cellfun(@(x,y) sprintf('%.2f %s',x,y),tmp,{'trials','correct','error','% accuracy'},'uni',0))
    ylim([0 100])
    tmp = [sum(accuracyCounter(1,:)),accuracyCounter(1,:),accuracyCounter(1,1)/sum(accuracyCounter(1,:))*100;...
        sum(accuracyCounter(2,:)),accuracyCounter(2,:),accuracyCounter(2,1)/sum(accuracyCounter(2,:))*100;...
        sum(accuracyCounter(3,:)),accuracyCounter(3,:),accuracyCounter(3,1)/sum(accuracyCounter(3,:))*100;...
        sum(accuracyCounter(4,:)),accuracyCounter(4,:),accuracyCounter(4,1)/sum(accuracyCounter(4,:))*100;...
        sum(accuracyCounter(5,:)),accuracyCounter(5,:),accuracyCounter(5,1)/sum(accuracyCounter(5,:))*100];
    subplot(2,1,2),bar(tmp)
    ylim([0 100])
    set(gca,'xticklabels',classNames)
end

runCount = str2double( bci_Parameters.MyRunCount{ 1, 1 } );
runCount = runCount + 1;
bci_Parameters.MyRunCount{ 1, 1 } = num2str( runCount );
save('E:\NeuroBotsDemonstrator\data\output.mat','output')
save(['E:\NeuroBotsDemonstrator\data\' bci_Parameters.SubjectName{:} 'S' bci_Parameters.SubjectSession{:} 'R'  datestr(datetime,'yyyymmddHHMMSS') '_runSummary.mat'],'accuracyCounter')