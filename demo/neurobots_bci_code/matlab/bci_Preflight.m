function [ out_signal_dim ] = bci_Preflight( in_signal_dim )

% Filter preflight demo
% 
% Check whether parameters and states are accessible, and whether
% parameters have values that allow for safe processing by the 
% bci_Process function.
% Report any errors as demonstrated below.
% Also, report output signal dimensions in the 'out_signal_dim' argument.

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

out_signal_dim = in_signal_dim;

% Check whether this parameter is available.
bci_Parameters.MyRunCount;

% Check whether the state is available.
bci_States.MyDemoState; 
bci_States.StimulusCode;

% Parameters are cell arrays of strings.


