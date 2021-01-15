% Figure for comparison of the number of operations for dynamics toolboxes
% Before execution: Run statistics_serroblib.m with matlabfcnmode='dyncmp';

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

%% Load Data
serrobpath = fileparts(which('serroblib_path_init.m'));
figure_dir = fileparts(which('statistics_serroblib.m'));
if isempty(figure_dir)
  error('This script has to be run at least by the "run" command');
end
% Use the evaluation for all robot models and all files
matlabfcnmode = 'dyncmp';
serrob_stat_file = fullfile(figure_dir, ...
  sprintf('statistics_serroblib_%s.mat',matlabfcnmode));
d = load(serrob_stat_file, 'CompEffortTable_hd', 'CompEffortTable_sp', 'CompEffortTable_sy');
CompEffortTable_hd = d.CompEffortTable_hd;
CompEffortTable_sp = d.CompEffortTable_sp;
CompEffortTable_sy = d.CompEffortTable_sy;

%% Create bar diagram of number of operations for all models
% Get the robot names ('stable' to keep the sorted order).
RobNames = unique(CompEffortTable_sp.Name,'stable');
% Gather summary table for creating the diagram
head_row = {'Name', 'NumDoF', 'NumOps_hdlag', 'NumOps_hdnew', 'NumOps_sy', 'NumOps_sp'};
CompNumOps_Tab = cell2table(cell(0,length(head_row)), 'VariableNames', head_row);
% Assemble results, check if data for the respective robot is available
for i = 1:length(RobNames)
  I_N = find(strcmp(CompEffortTable_hd.Name,RobNames{i}),1,'first');
  N = CompEffortTable_hd.NumDoF(I_N);
  I_hdlag = strcmp(CompEffortTable_hd.Name,RobNames{i}) & ...
    CompEffortTable_hd.CodeFile == 1; % invdynJ_fixb_snew_vp2; see statistics_serroblib.m (for mode dyncmp)
  if any(I_hdlag)
    numops_hdlag = CompEffortTable_hd.ComputationalCostSum(I_hdlag);
  else
    numops_hdlag = NaN;
  end
  I_hdnew = strcmp(CompEffortTable_hd.Name,RobNames{i}) & ...
    CompEffortTable_hd.CodeFile == 2; % invdynJ_fixb_mdp_slag_vp; see statistics_serroblib.m (for mode dyncmp)
  if any(I_hdnew)
    numops_hdnew = CompEffortTable_hd.ComputationalCostSum(I_hdnew);
  else
    numops_hdnew = NaN;
  end
  I_sy = strcmp(CompEffortTable_sy.Name,RobNames{i}) & ...
    CompEffortTable_sy.CodeFile == 2; % invdyn_symoro; see statistics_serroblib.m
  if any(I_sy)
    numops_sy = CompEffortTable_sy.ComputationalCostSum(I_sy);
  else
    numops_sy = NaN;
  end
  I_sp = strcmp(CompEffortTable_sp.Name,RobNames{i}) & ...
    CompEffortTable_sp.CodeFile == 4; % invdyn_sympybotics; see statistics_serroblib.m
  if any(I_sp)
    numops_sp = CompEffortTable_sp.ComputationalCostSum(I_sp);
  else
    numops_sp = NaN;
  end
  CompNumOps_Tab = [CompNumOps_Tab; {RobNames{i}, N, numops_hdlag, ...
    numops_hdnew, numops_sy, numops_sp}]; %#ok<AGROW>
end
% Create the figure
I = CompNumOps_Tab.NumDoF < 7 & CompNumOps_Tab.NumDoF > 2;
BarMatrix = [CompNumOps_Tab.NumOps_hdlag(I), CompNumOps_Tab.NumOps_hdnew(I), ...
  CompNumOps_Tab.NumOps_sy(I), CompNumOps_Tab.NumOps_sp(I)];
ToolBoxname = {'Proposed, Lagrange', 'Proposed, Newton', 'SymPyBotics', 'OpenSymoro'};
axhdl = NaN(1,4);
figure(1);clf;
scale = 1e-3;
for i = 1 : 4
  axhdl(i) = subplot(1,4,i);hold on;
  legentries = sprintf('Computational Cost for %s Toolbox',ToolBoxname{i});
  legbarhdl = bar(scale*BarMatrix(:,i));
  set(legbarhdl, 'EdgeColor', 'k', 'FaceColor', 'k');
  grid on;
  if i == 1
    ylabel('# of Ops. in K');
  end
  xticklabels({});
  title(ToolBoxname{i});
  ylim([0, scale*max(BarMatrix(:))]); % all plots have same height
end
remove_inner_labels(axhdl,2);
figure_format_publication(axhdl)
set_size_plot_subplot(1,...
  12.1,3,axhdl,...
  0.05,0.003,0.14,0.04,... % bl,br,hu,hd,
  0.01,0) % bdx,bdy)
export_fig(1, fullfile(figure_dir, 'statistics_serrob_numops_histogram.pdf'));