% Figure for comparing computational effort for different robot models

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-10
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

regen_parrob_table = false; % switch for force re-counting of computational effort in serial robot database
only_reduced_figure = false; % switch for only plottin a few bars in the diagram for quick preview
parrobpath = fileparts(which('parroblib_path_init.m'));
if isempty(parrobpath)
  warning('Repository with parallel robot models not in path. Abort.');
  return
end
hybrdynpath = fileparts(which('hybrdyn_path_init.m'));
if isempty(hybrdynpath)
  warning('Repository with code generation toolbox not in path. Abort.');
  return
end
addpath(fullfile(hybrdynpath, 'test'));
figure_dir = fileparts(which('statistics_parroblib.m'));
if isempty(figure_dir)
  error('This script has to be run at least by the "run" command');
end
%% Parse library of serial robot models
% go through all models and all code files
% assess computational cost of all matlab functions
% save everything in a mat file
codefilenames_A0 = { ...
{'coriolisvec_para_pf_mdp', 4}, ...
{'coriolisvec_para_pf_regmin', 4}, ...
{'coriolisvec_para_pf_slag_vp1', 4}, ...
{'coriolisvec_para_pf_slag_vp2', 4}, ...
{'gravload_para_pf_mdp', 3}, ...
{'gravload_para_pf_regmin', 3}, ...
{'gravload_para_pf_slag_vp1', 3}, ...
{'gravload_para_pf_slag_vp2', 3}, ...
{'inertia_para_pf_mdp', 2}, ...
{'inertia_para_pf_regmin', 2}, ...
{'inertia_para_pf_slag_vp1', 2}, ...
{'inertia_para_pf_slag_vp2', 2}, ...
{'invdyn_para_pf_mdp', 5}, ...
{'invdyn_para_pf_regmin', 5}, ...
{'invdyn_para_pf_slag_vp1', 5}, ...
{'invdyn_para_pf_slag_vp2', 5}};
codefilenames_Ax = {{'Jinv', 1}};
EEDoF_all = [1 1 0 0 0 1; ...
             1 1 1 0 0 0; ...
             1 1 1 0 0 1; ...
             1 1 1 1 1 1];

parrob_stat_file = fullfile(figure_dir, sprintf('statistics_parroblib.mat'));
if regen_parrob_table || ~exist(parrob_stat_file, 'file')
  head_row = {'Name', 'i_EE_DoF', 'CodeFile', 'DurationCPUTime', 'ComputationalCostSum', ...
    'OptCodeLineCount', 'OptCodeSize', 'FileSize'};
  CompEffortTable = cell2table(cell(0,length(head_row)), 'VariableNames', head_row);
  num_rob_per_DoF = NaN(1,4);
  for i_DoF = 1:4
    EE_DoF = EEDoF_all(i_DoF,:);
    [PNames_Kin, PNames_Akt, AdditionalInfo_Akt] = parroblib_filter_robots( ...
      sum(EE_DoF), EE_DoF, ones(1,6), 6);
    PNames_Legs = {};
    for i = 1:length(PNames_Kin)
      PNames_Legs = [PNames_Legs, PNames_Kin{i}(1:end-4)]; %#ok<AGROW>
    end
    PNames_Legs = unique(PNames_Legs);
    num_rob_per_DoF(i_DoF) = length(PNames_Kin);
    dir_NLeg = fullfile(parrobpath, sprintf('sym%dleg', sum(EE_DoF)));
    fprintf('Go through parallel robot models with DoF [%s] (%d robots).\n', ...
      char(48+EE_DoF), length(PNames_Legs));
    for ii = 1:length(PNames_Legs)
      NameKin = PNames_Legs{ii};
      fprintf('%d/%d: %s\n', ii, length(PNames_Legs), NameKin);
      robkindir = fullfile(dir_NLeg, NameKin);
      subdirs_A0 = dir(fullfile(robkindir, 'hd*A0')); % dynamics code subdirectories
      for kk = 1:length(subdirs_A0)
        subdir_kk = fullfile(robkindir, subdirs_A0(kk).name);
        [tokens, ~] = regexp(subdirs_A0(kk).name, 'G(\d)A0', 'tokens', 'match');
        GNr = tokens{1}{1};
        NameKinG = [NameKin, 'G', GNr];
        for jj = 1:length(codefilenames_A0)
          codefile_jj = fullfile(subdir_kk, [NameKinG, 'A0', '_', codefilenames_A0{jj}{1}, '.m']);
          if ~exist(codefile_jj, 'file')
            continue
          end
          infostruct = get_codegen_info_from_matlabfcn(codefile_jj);
          if isnan(infostruct.OptimizationMode)
            warning('No valid codegen information in file %s', codefile_jj);
          end
          cc = infostruct.ComputationalCost;
          ComputationalCostSum = cc.add+cc.mult+cc.div+cc.fcn+cc.ass;
          Row_jj = {NameKinG, i_DoF, jj, infostruct.DurationCPUTime, ComputationalCostSum, ...
            infostruct.OptCodeLineCount, infostruct.OptCodeSize, infostruct.FileSize};
          CompEffortTable = [CompEffortTable; Row_jj]; %#ok<AGROW>
        end
      end
      subdirs_Px = dir(fullfile(robkindir, 'hd_G*P*')); % Jacobian code subdirectories
      for kk = 1:length(subdirs_Px)
        subdir_kk = fullfile(robkindir, subdirs_Px(kk).name);
        [tokens, ~] = regexp(subdirs_Px(kk).name, 'G(\d)P(\d)A(\d)', 'tokens', 'match');
        GNr = tokens{1}{1};
        PNr = tokens{1}{2};
        ANr = tokens{1}{3};
        NameKinG = [NameKin, 'G', GNr];
        NameAkt = [NameKin, 'G', GNr, 'P', PNr, 'A', ANr];
        for jj = 1:length(codefilenames_Ax)
          codefile_jj = fullfile(subdir_kk, [NameAkt, '_', codefilenames_Ax{jj}{1}, '.m']);
          if ~exist(codefile_jj, 'file')
            continue
          end
          infostruct = get_codegen_info_from_matlabfcn(codefile_jj);
          if isnan(infostruct.OptimizationMode)
            warning('No valid codegen information in file %s', codefile_jj);
          end
          cc = infostruct.ComputationalCost;
          ComputationalCostSum = cc.add+cc.mult+cc.div+cc.fcn+cc.ass;
          Row_jj = {NameKinG, i_DoF, length(codefilenames_A0)+jj, infostruct.DurationCPUTime, ComputationalCostSum, ...
            infostruct.OptCodeLineCount, infostruct.OptCodeSize, infostruct.FileSize};
          CompEffortTable = [CompEffortTable; Row_jj]; %#ok<AGROW>
        end
      end
    end
  end
  save(parrob_stat_file, 'CompEffortTable', 'num_rob_per_DoF');
else
  load(parrob_stat_file, 'CompEffortTable', 'num_rob_per_DoF');
end

%% Group code files into categories
% By doing this here, the categories can be changed afterwards without
% having to parse the whole database again.
codefilenames_all = [codefilenames_A0, codefilenames_Ax];
codefilecategories = NaN(length(codefilenames_all),1);
for i = 1:length(codefilenames_all)
  codefilecategories(i) = codefilenames_all{i}{2};
end
% Append categories to table
CompEffortTable = addvars(CompEffortTable, NaN(size(CompEffortTable,1),1), 'After', 3);
CompEffortTable.Properties.VariableNames(4) = {'FileCategory'};
for i = 1:length(codefilenames_all)
  I = CompEffortTable.CodeFile == i;
  CompEffortTable.FileCategory(I) = codefilecategories(i);
end

%% Output statistics information to put in the presentation
Robots = unique(CompEffortTable.Name,'stable');
fprintf('%d unique parallel kinematic structures in database (%d with sym. code)\n', ...
  sum(num_rob_per_DoF), length(Robots));
fprintf('%1.1f days of CPU time to generate all functions in the database\n', ...
  sum(CompEffortTable.DurationCPUTime(:), 'omitnan')/(3600*24));
fprintf('%d automatically generated files\n', size(CompEffortTable,1));
fprintf('%d lines of automatically generated code (%1.1f MB)\n', ...
  sum(CompEffortTable.OptCodeLineCount(:)), sum(CompEffortTable.OptCodeSize(:))/1024^2);

% Create bar heights for histogram.
BarMatrix = NaN(length(Robots), max(codefilecategories));
for i = 1:length(Robots)
  I_Robi = strcmp(CompEffortTable.Name, Robots{i});
  for j = 1:max(codefilecategories)
    I_catj = CompEffortTable.FileCategory == j;
    BarMatrix(i,j) = sum(CompEffortTable.DurationCPUTime(I_Robi&I_catj), 'omitnan');
  end
end
BarMatrixSum = sum(BarMatrix,2);
save(fullfile(figure_dir, 'parrrob_cputime_data_tmp.mat'));
BarMatrix_h = BarMatrix / 3600;
BarMatrixSum_h = BarMatrixSum / 3600;
I_nodyn = all(BarMatrix(:,2:5)==0,2);
fprintf('The following %d robots do not have dynamics functions, only kinematics', sum(I_nodyn));
disp(Robots(I_nodyn)');

%% Print additional information
i_DoF_prev = 0;
DoF_startin_number = NaN(1,4);
for i = 1:length(Robots)
  I = find(strcmp(Robots{i}, CompEffortTable.Name), 1, 'first');
  i_DoF = CompEffortTable.i_EE_DoF(I);
  print_name = false;
  if i == 1 || i_DoF ~= i_DoF_prev
    print_name = true;
    DoF_startin_number(i_DoF) = i;
  end
  if print_name
    fprintf('Starting at number %d (%s): DoF [%s]\n', ...
      i, Robots{i}, char(48+EEDoF_all(i_DoF,:)));
  end
  i_DoF_prev = i_DoF;
end

%% Create bar diagram of effort for all models
t1 = tic();
figure(2);clf;hold on;
legbarhdl = NaN(max(codefilecategories), 1);
Farben = {'r', 'g', 'b', 'c', 'm', 'y', 'k'};
legentries = {'kinematics', 'inertia', 'gravload.', 'coriolis vec.', 'invdyn'};
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
xlabel('Running number of robot kinematics');
ylabel('CPU time for codegen. in h');
l1hdl = legend(legbarhdl, legentries, 'location', 'northoutside', 'orientation', 'horizontal');
figure_format_publication()
set_size_plot_subplot(2,...
  22,4.5,gca,...
  0.07,0.01,0.13,0.18,... % bl,br,hu,hd,
  0,0) % bdx,bdy)
set(l1hdl, 'position', [0.25    0.92    0.50    0.05], ...
  'orientation', 'horizontal');
fprintf('Generated bar diagram figure. Duration: %1.1fs\n', toc(t1));
%% Show all robots
% change_plot_data_scale(gca, [1, 1/3600, 1]);
% change_plot_data_scale(gca, [1, 3600, 1]);

xlim([0, length(Robots)+0.5]); % show all robots
ylim([0, 1.5]); % limit y axes so all are still visible
grid on;
saveas(2,     fullfile(figure_dir, sprintf('statistics_parrob_cputime_hist_cuty.fig')));
export_fig(2, fullfile(figure_dir, sprintf('statistics_parrob_cputime_hist_cuty.pdf')));
cd(figure_dir);
export_fig(sprintf('statistics_parrob_cputime_hist_cuty.png'), '-r800');

xlim([0, length(Robots)+0.5]);
ylim([0, 1.05*max(BarMatrixSum_h)]); % show everything
grid on;
saveas(2,     fullfile(figure_dir, sprintf('statistics_parrob_cputime_hist_full.fig')));
export_fig(2, fullfile(figure_dir, sprintf('statistics_parrob_cputime_hist_full.pdf')));
cd(figure_dir);
export_fig(sprintf('statistics_parrob_cputime_hist_full.png'), '-r800');

%% Detailed view for 3 do 4 dof
xlim([0, DoF_startin_number(4)-1.5])
ylim([0, 1.05*max(BarMatrixSum_h(1:DoF_startin_number(4)-1))]);
saveas(2,     fullfile(figure_dir, sprintf('statistics_parrob_cputime_hist_2T1R_to_3T1R.fig')));
export_fig(2, fullfile(figure_dir, sprintf('statistics_parrob_cputime_hist_2T1R_to_3T1R.pdf')));
cd(figure_dir);
export_fig(sprintf('statistics_parrob_cputime_hist_2T1R_to_3T1R.png'), '-r800');

%% Detailed view for 6 dof
xlim([DoF_startin_number(4)-0.5, length(Robots)+0.5])
ylim([0, 10]); % limit to 10h code generation time
saveas(2,     fullfile(figure_dir, sprintf('statistics_parrob_cputime_hist_3T3R_red.fig')));
export_fig(2, fullfile(figure_dir, sprintf('statistics_parrob_cputime_hist_3T3R_red.pdf')));
cd(figure_dir);
export_fig(sprintf('statistics_parrob_cputime_hist_3T3R_red.png'), '-r800');

ylim([0, 1.05*max(BarMatrixSum_h)]); % show full data (some large bars dominate all)
saveas(2,     fullfile(figure_dir, sprintf('statistics_parrob_cputime_hist_3T3R_full.fig')));
export_fig(2, fullfile(figure_dir, sprintf('statistics_parrob_cputime_hist_3T3R_full.pdf')));
cd(figure_dir);
export_fig(sprintf('statistics_parrob_cputime_hist_3T3R_full.png'), '-r800');