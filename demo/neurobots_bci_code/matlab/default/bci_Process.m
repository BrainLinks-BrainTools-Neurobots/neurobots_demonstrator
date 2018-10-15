function out_signal = bci_Process( in_signal )

% Filter process demo
% 
% Apply a filter to in_signal, and return the result in out_signal.
% Signal dimensions are ( channels x samples ).

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

% This file runs with every new batch of data incoming from the headset
% Last edit by Omar Jubran 12.11.2013

% Parameters and states are global variables.
global bci_Parameters bci_States push f;

out_signal(1,:) = 2;
out_signal(2,:) = 2;