% Figure for comparison of the computation time for serial robots
% Used in the presentation slides
% Before execution: Run statistics_serroblib.m with matlabfcnmode='full';

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-10
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

only_reduced_figure = false; % switch for only plotting a few bars in the diagram for quick preview

%% Load Data
serrobpath = fileparts(which('serroblib_path_init.m'));
figure_dir = fileparts(which('statistics_serroblib.m'));
if isempty(figure_dir)
  error('This script has to be run at least by the "run" command');
end
% Use the evaluation for all robot models and all files
matlabfcnmode = 'full';
serrob_stat_file = fullfile(figure_dir, ...
  sprintf('statistics_serroblib_%s.mat',matlabfcnmode));
d = load(serrob_stat_file, 'CompEffortTable_hd', 'codefilecategories');
CompEffortTable_hd = d.CompEffortTable_hd;
codefilecategories = d.codefilecategories;
%% Output statistics information to put in the presentation
I = CompEffortTable_hd.NumDoF < 7 & CompEffortTable_hd.NumDoF > 2;
Robots = unique(CompEffortTable_hd.Name(I),'stable');
fprintf('%d unique serial kinematic structures in database\n', length(Robots));
fprintf('%1.1f days of CPU time to generate all functions in the database\n', ...
  sum(CompEffortTable_hd.DurationCPUTime(:), 'omitnan')/(3600*24));
fprintf('%d automatically generated files\n', size(CompEffortTable_hd,1));
fprintf('%d lines auf automatically generated code (%1.1f MB)\n', ...
  sum(CompEffortTable_hd.OptCodeLineCount(:)), sum(CompEffortTable_hd.OptCodeSize(:))/1024^2);

% Create bar heights for histogram.
BarMatrix = NaN(length(Robots), max(codefilecategories));
for i = 1:length(Robots)
  I_Robi = strcmp(CompEffortTable_hd.Name, Robots{i});
  for j = 1:max(codefilecategories)
    I_catj = CompEffortTable_hd.FileCategory == j;
    BarMatrix(i,j) = sum(CompEffortTable_hd.DurationCPUTime(I_Robi&I_catj), 'omitnan');
  end
end
BarMatrixSum = sum(BarMatrix,2);
save(fullfile(figure_dir, sprintf('statistics_serrob_cputime_data_tmp_%s.mat', matlabfcnmode)));
BarMatrix_h = BarMatrix / 3600;
BarMatrixSum_h = BarMatrixSum / 3600;

%% Print additional information

num_Rjoints_prev = 0;
num_joints_prev = 0;
for i = 1:length(Robots)
  num_Rjoints = sum(Robots{i}=='R');
  num_joints = str2double(Robots{i}(2));
  print_name = false;
  if i == 1 || num_joints ~= num_joints_prev
    print_name = true;
  end
  if print_name || num_Rjoints~= num_Rjoints_prev
    print_name = true;
  end
  if print_name
    fprintf('Starting at number %d (%s): %d joints; %d revolute joints\n', ...
      i, Robots{i}, num_joints, num_Rjoints);
  end
  num_Rjoints_prev = num_Rjoints;
  num_joints_prev = num_joints;
end

Robots_find = { ...
  'S5PRRRR3', ... % (PRRR)(R)
  'S5PRRRR4', ... % (PRR)(R)(R)
  'S5PRRRR6'}; ... % (P)(RR)(RR)
for i = 1:length(Robots_find)
  I = strcmp(Robots, Robots_find{i});
  if ~any(I)
    warning('Robot %s was not found in the symbolic code database', Robots_find{i});
    continue
  end
  fprintf('Robot %s at position %d\n', Robots_find{i}, find(I));
end

%% Create bar diagram of effort for all models
t1 = tic();
figure(1);clf;hold on;
legbarhdl = NaN(max(codefilecategories), 1);
Farben = {'r', 'g', 'b', 'c', 'm', 'y', 'k'};
legentries = {'kinematics', 'energy', 'inertia', 'coriolis vec.', 'gravload.', 'invdyn', 'coriolis mat.'};
if only_reduced_figure
  I_Rob = 1:20:length(Robots); %#ok<UNRCH>
else
  I_Rob = 1:length(Robots);
end
for i = I_Rob
  barheight_i = sum(BarMatrix_h(i,:));
  for j = max(codefilecategories):-1:1
    legbarhdl(j) = bar(i, barheight_i);
    set(legbarhdl(j), 'EdgeColor', 'none', 'FaceColor', Farben{j});
    barheight_i = barheight_i - BarMatrix_h(i,j);
  end
end
grid on;
xlabel('Running number of robot kinematics');
ylabel('CPU time for codegen. in h');
l1hdl = legend(legbarhdl, legentries, 'location', 'northoutside', 'orientation', 'horizontal');
figure_format_publication()
set_size_plot_subplot(1,...
  22,4.5,gca,...
  0.07,0.01,0.13,0.18,... % bl,br,hu,hd,
  0,0) % bdx,bdy)
set(l1hdl, 'position', [0.15    0.92    0.70    0.05], ...
  'orientation', 'horizontal');
saveas(1,     fullfile(figure_dir, sprintf('statistics_serrob_cputime_hist_%s.fig', matlabfcnmode)));
export_fig(1, fullfile(figure_dir, sprintf('statistics_serrob_cputime_hist_%s.pdf', matlabfcnmode)));
cd(figure_dir);
export_fig(sprintf('statistics_serrob_cputime_hist_%s.png', matlabfcnmode), '-r800');
fprintf('Generated bar diagram figure. Duration: %1.1fs\n', toc(t1));

%% Export area of the bar diagram with the three PRRRR robots highlighted
xlim([241.5, 244.5]);
ylim([0, 1.05*max(BarMatrixSum_h(242:244))]);
set_size_plot_subplot(1,...
  2,2,gca,...
  0.0,0.0,0,0,... % bl,br,hu,hd,
  0,0) % bdx,bdy)
xlabel('');
ylabel('');
ch = get(gcf, 'children');
delete(ch(strcmp(get(ch, 'Type'), 'legend')));
grid off;
set(gca, 'Box', 'off');
export_fig(1, fullfile(figure_dir, sprintf('statistics_serrob_cputime_hist_%s_detail_PRRRR.pdf', matlabfcnmode)));
cd(figure_dir);
export_fig(sprintf('statistics_serrob_cputime_hist_%s_detail_PRRRR.png', matlabfcnmode), '-r800');
