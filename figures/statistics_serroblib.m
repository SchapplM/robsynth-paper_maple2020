% Figure for comparing computational effort for parallel robot models

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-10
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

matlabfcnmode = 'full'; % other: 'simple'
regen_serrob_table = false; % switch for force re-counting of computational effort in serial robot database
only_reduced_figure = false; % switch for only plottin a few bars in the diagram for quick preview
serrobpath = fileparts(which('serroblib_path_init.m'));
if isempty(serrobpath)
  warning('Repository with serial robot models not in path. Abort.');
  return
end
hybrdynpath = fileparts(which('hybrdyn_path_init.m'));
if isempty(hybrdynpath)
  warning('Repository with code generation toolbox not in path. Abort.');
  return
end
addpath(fullfile(hybrdynpath, 'test'));
figure_dir = fileparts(which('statistics_serroblib.m'));
if isempty(figure_dir)
  error('This script has to be run at least by the "run" command');
end
%% Parse library of serial robot models
% go through all models and all code files
% assess computational cost of all matlab functions
% save everything in a mat file
if strcmp(matlabfcnmode, 'simple')
  codefilenames = { ...
    {'fkine_fixb_rotmat_mdh_sym_varpar', 1}...
    {'joint_trafo_rotmat_mdh_sym_varpar', 1}, ...
    {'jacobig_rot_sym_varpar', 1}, ...
    {'jacobigD_rot_sym_varpar', 1}, ...
    {'jacobia_transl_sym_varpar', 1}, ...
    {'jacobiaD_transl_sym_varpar', 1}, ...
    {'energykin_fixb_regmin_slag_vp', 2}, ...
    {'energypot_fixb_regmin_slag_vp', 2}, ...
    {'inertiaJ_mdp_slag_vp', 3}, ...
    {'coriolisvecJ_fixb_mdp_slag_vp', 4}, ...
    {'gravloadJ_floatb_twist_mdp_slag_vp', 5}, ...
    {'invdynJ_fixb_mdp_slag_vp', 6}, ...
    {'coriolismatJ_fixb_regmin_slag_vp', 7}};
else
  codefilenames = { ...
    {'coriolismatJ_fixb_reg2_slag_vp', 7}, ...
    {'coriolismatJ_fixb_regmin_slag_vp', 7}, ...
    {'coriolismatJ_fixb_slag_vp1', 7}, ...
    {'coriolismatJ_fixb_slag_vp2', 7}, ...
    {'coriolisvecJ_fixb_mdp_slag_vp', 4}, ...
    {'coriolisvecJ_fixb_reg2_slag_vp', 4}, ...
    {'coriolisvecJ_fixb_regmin_slag_vp', 4}, ...
    {'coriolisvecJ_fixb_slag_vp1', 4}, ...
    {'coriolisvecJ_fixb_slag_vp2', 4}, ...
    {'energykin_fixb_reg2_slag_vp', 2}, ...
    {'energykin_fixb_regmin_slag_vp', 2}, ...
    {'energykin_fixb_slag_vp1', 2}, ...
    {'energykin_fixb_slag_vp2', 2}, ...
    {'energykin_floatb_twist_slag_vp1', 2}, ...
    {'energykin_floatb_twist_slag_vp2', 2}, ...
    {'energypot_fixb_reg2_slag_vp', 2}, ...
    {'energypot_fixb_regmin_slag_vp', 2}, ...
    {'energypot_fixb_slag_vp1', 2}, ...
    {'energypot_fixb_slag_vp2', 2}, ...
    {'energypot_floatb_twist_slag_vp1', 2}, ...
    {'energypot_floatb_twist_slag_vp2', 2}, ...
    {'fkine_fixb_rotmat_mdh_sym_varpar', 1}, ...
    {'gravloadJ_floatb_twist_mdp_slag_vp', 5}, ...
    {'gravloadJ_floatb_twist_slag_vp1', 5}, ...
    {'gravloadJ_floatb_twist_slag_vp2', 5}, ...
    {'gravloadJ_reg2_slag_vp', 5}, ...
    {'gravloadJ_regmin_slag_vp', 5}, ...
    {'inertiaDJ_reg2_slag_vp', NaN}, ...
    {'inertiaDJ_regmin_slag_vp', NaN}, ...
    {'inertiaDJ_slag_vp1', NaN}, ...
    {'inertiaDJ_slag_vp2', NaN}, ...
    {'inertiaJ_mdp_slag_vp', 3}, ...
    {'inertiaJ_reg2_slag_vp', 3}, ...
    {'inertiaJ_regmin_slag_vp', 3}, ...
    {'inertiaJ_slag_vp1', 3}, ...
    {'inertiaJ_slag_vp2', 3}, ...
    {'invdynB_fixb_reg2_snew_vp', NaN}, ...
    {'invdynB_fixb_snew_vp2', NaN}, ...
    {'invdynf_fixb_reg2_snew_vp', NaN}, ...
    {'invdynf_fixb_snew_vp2', NaN}, ...
    {'invdynJ_fixb_mdp_slag_vp', 6}, ...
    {'invdynJ_fixb_reg2_slag_vp', 6}, ...
    {'invdynJ_fixb_reg2_snew_vp', 6}, ...
    {'invdynJ_fixb_regmin_slag_vp', 6}, ...
    {'invdynJ_fixb_slag_vp1', 6}, ...
    {'invdynJ_fixb_slag_vp2', 6}, ...
    {'invdynJ_fixb_snew_vp2', 6}, ...
    {'invdynm_fixb_reg2_snew_vp', NaN}, ...
    {'invdynm_fixb_snew_vp2', NaN}, ...
    {'jacobiaD_rot_sym_varpar', 1}, ...
    {'jacobiaD_transl_sym_varpar', 1}, ...
    {'jacobia_rot_sym_varpar', 1}, ...
    {'jacobia_transl_sym_varpar', 1}, ...
    {'jacobigD_rot_sym_varpar', 1}, ...
    {'jacobig_rot_sym_varpar', 1}, ...
    {'jacobiRD_rot_sym_varpar', 1}, ...
    {'jacobiR_rot_sym_varpar', 1}, ...
    {'joint_trafo_rotmat_mdh_sym_varpar', 1}};
end
serrob_stat_file = fullfile(figure_dir, sprintf('statistics_serroblib_%s.mat',matlabfcnmode));
if regen_serrob_table || ~exist(serrob_stat_file, 'file')
  head_row = {'Name', 'NumDoF', 'CodeFile', 'DurationCPUTime', 'ComputationalCostSum', ...
    'OptCodeLineCount', 'OptCodeSize', 'FileSize'};
  CompEffortTable = cell2table(cell(0,length(head_row)), 'VariableNames', head_row);
  filelist_invalid = {};
  for N = 1:7
    dir_Ndof = fullfile(serrobpath, sprintf('mdl_%ddof', N));
    mdllistfile_Ndof = fullfile(dir_Ndof, sprintf('S%d_list.mat',N));
    l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
    
    I = find(l.AdditionalInfo(:,2) == 0);
    
    % Sort Table according to number of joints and revolute joints
    NrotJ_all = l.AdditionalInfo(I,5);
    [~, II_sortrotJ] = sort(NrotJ_all);
    I_sortrotJ = I(II_sortrotJ);
    if ~all(intersect(I_sortrotJ, I) == I)
      error('Error sorting the robots regarding number of revolute joints');
    end
    % l.Names_Ndof
    fprintf('Go through serial robot models with %d DoF (%d robots).\n', N, length(I));
    for ii = I_sortrotJ(:)'
      Name = l.Names_Ndof{ii};
      fprintf('%s\n', Name);
      codedir = fullfile(dir_Ndof, Name, 'hd');
      for jj = 1:length(codefilenames)
        codefile_jj = fullfile(codedir, [Name, '_', codefilenames{jj}{1}, '.m']);
        if ~exist(codefile_jj, 'file')
          continue
        end
        infostruct = get_codegen_info_from_matlabfcn(codefile_jj);
        if isnan(infostruct.OptimizationMode)
          warning('No valid codegen information in file %s', codefile_jj);
          filelist_invalid = [filelist_invalid; {codefile_jj}]; %#ok<AGROW>
        end
        cc_dbg = infostruct.ComputationalCostDebug;
        cc = infostruct.ComputationalCost;
        if any(abs(cc.ass - cc_dbg.ass) > 1)
          error('Number of assignments strongly diverging');
        end
        if any(abs(cc.fcn - cc_dbg.fcn) > 1)
          error('Number of function calls diverging');
        end
        ComputationalCostSum = sum(cc.add+cc.mult+cc.div+cc.fcn+cc.ass);
        Row_jj = {Name, N, jj, sum(infostruct.DurationCPUTime), ComputationalCostSum, ...
          sum(infostruct.OptCodeLineCount), sum(infostruct.OptCodeSize), infostruct.FileSize};
        CompEffortTable = [CompEffortTable; Row_jj]; %#ok<AGROW>
      end
    end
  end
  save(serrob_stat_file, 'CompEffortTable', 'filelist_invalid');
%   serrob_stat_file = fullfile(figure_dir, sprintf('statistics_serroblib_%s.mat',matlabfcnmode));
  writecell(filelist_invalid, fullfile(figure_dir, 'statistics_serroblib_files_invalid.txt'));
else
  load(serrob_stat_file, 'CompEffortTable', 'filelist_invalid');
end

%% Group code files into categories
% By doing this here, the categories can be changed afterwards without
% having to parse the whole database again.
codefilecategories = NaN(length(codefilenames),1);
for i = 1:length(codefilenames)
  codefilecategories(i) = codefilenames{i}{2};
end
% Append categories to table
CompEffortTable = addvars(CompEffortTable, NaN(size(CompEffortTable,1),1), 'After', 3);
CompEffortTable.Properties.VariableNames(4) = {'FileCategory'};
for i = 1:length(codefilenames)
  I = CompEffortTable.CodeFile == i;
  CompEffortTable.FileCategory(I) = codefilecategories(i);
end

%% Output statistics information to put in the presentation
I = CompEffortTable.NumDoF < 7 & CompEffortTable.NumDoF > 2;
Robots = unique(CompEffortTable.Name(I),'stable');
fprintf('%d unique serial kinematic structures in database\n', length(Robots));
fprintf('%1.1f days of CPU time to generate all functions in the database\n', ...
  sum(CompEffortTable.DurationCPUTime(:), 'omitnan')/(3600*24));
fprintf('%d automatically generated files\n', size(CompEffortTable,1));
fprintf('%d lines auf automatically generated code (%1.1f MB)\n', ...
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

% Export area of the bar diagram with the three PRRRR robots highlighted
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
