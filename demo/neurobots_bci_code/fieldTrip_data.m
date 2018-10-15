filename = 'buffer://localhost:1972';

% read the header for the first time to determine number of channels and sampling rate
hdr = ft_read_header(filename, 'cache', true);

count      = 0;
prevSample = 0
blocksize  = hdr.Fs;
chanindx   = 1:hdr.nChans;

while true
  % determine number of samples available in buffer
  hdr = ft_read_header(filename, 'cache', true);

  % see whether new samples are available
  newsamples = (hdr.nSamples*hdr.nTrials-prevSample);

  if newsamples>=blocksize

    % determine the samples to process
    begsample  = prevSample+1;
    endsample  = prevSample+blocksize ;

    % remember up to where the data was read
    prevSample  = endsample;
    count       = count + 1;
    fprintf('processing segment %d from sample %d to %d\n', count, begsample, endsample);

    % read data segment from buffer
    dat = ft_read_data(filename, 'header', hdr, 'begsample', begsample, 'endsample', endsample, 'chanindx', chanindx);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % subsequently the data can be processed, here it is only plotted
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % create a matching time-axis
    time = (begsample:endsample)/hdr.Fs;

    % plot the data just like a standard FieldTrip raw data strucute
    plot(time, dat);

    % ensure tight axes
    xlim([time(1) time(end)]);

    % force Matlab to update the figure
    drawnow

  end % if new samples available
end % while true