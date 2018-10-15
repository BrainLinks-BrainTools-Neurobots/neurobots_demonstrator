function PlotPingClientLog(x)

% Convert to seconds
x = x / 1e6;

% Offset to start everything at zero
clientOffset = x(1,1);
serverOffset = x(1,2);
x(:,1) = x(:,1) - clientOffset;
x(:,2) = x(:,2) - serverOffset;
x(:,3) = x(:,3) - clientOffset;

% Get things in a friendly form (and compute offsets)
ts = x(:,2);
ub = x(:,3)-x(:,2);
lb = x(:,1)-x(:,2);

close all
% Plot the offsets
%hFigOffset = 1;
%figure(hFigOffset); clf(hFigOffset);
%hAx = gca(hFigOffset);
%hold(hAx, 'on')
plot(ts, ub*1000, '.-b', ts, lb*1000, '.-r')
title('Offset')
xlabel( 'Time (s)')
ylabel( 'Offset (ms)')

figure;

% Find an approximation to skew, so that we can remove it and show
% just the drift components
 slope = (ub(end)-ub(1)) / (ts(end)-ts(1));
 corrections = (ts-ts(1))*slope;

% hFigSecondOrder = 2;
% figure(hFigSecondOrder); clf(hFigSecondOrder);
% hAx = gca(hFigSecondOrder);
% hold(hAx, 'on')

 plot(ts, (ub-corrections)*1000, '.-b', ts, (lb-corrections)*1000, ...
      '.-r')
 
  title( 'Second order offset components')
 xlabel( 'Time (s)')
 ylabel( 'Offset (ms)')
