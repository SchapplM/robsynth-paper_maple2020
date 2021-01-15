% Count number of operations in symbolically generated code.
% Used for opensymoro and sympybotics. Assume that the symbolic code starts
% in the section "Symbolic Calculation"

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-10
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function infostruct = count_operations_in_matlabfcn(filepath)
%% Daten Structure
infostruct = struct( ...
  'FileSize', NaN, ... % (Bytes)
  'ComputationalCost', struct('add', 0, 'mult', 0, 'div', 0, 'fcn', 0, 'ass', 0));

%% FileSize 
infostruct.FileSize = dir(filepath).bytes;

%% Computational Cost
text = fileread(filepath);
expression =  '%% Symbolic Calculation';
[~,endIndex] = regexp(text,expression);
if ~isnan(endIndex)
 text_com = text(endIndex + 1:end);
else
 text_com = text;
end

infostruct.ComputationalCost.add = ...
  length(regexp(text_com,'[+-]','match')) - ...
  length(regexp(text_com, '(=[\s]-)', 'match')); % Vorzeichen nicht als Addition zählen
infostruct.ComputationalCost.mult = length(regexp(text_com,'*','match')) + ...
  length(regexp(text_com, '(\^[\s]*2)', 'match'));
infostruct.ComputationalCost.div = length(regexp(text_com,'/','match'));
infostruct.ComputationalCost.fcn = ...
  length(regexp(text_com,'sin','match')) + ...
  length(regexp(text_com,'cos','match')) + ...
  length(regexp(text_com,'atan','match'));
infostruct.ComputationalCost.ass = length(regexp(text_com,'=','match'));
