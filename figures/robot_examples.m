% Generate images from robots which have a dynamics model from the toolbox

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
% close all
figure_dir = fileparts(which('robot_examples.m'));

for iRob = 1:10
  f = figure(2);clf;
  hold on;
  grid off;
  view(3);
  set(gca,'XTICKLABEL',{}, 'YTICKLABEL', {}, 'ZTICKLABEL',{});
  set(gca,'xtick',[],'ytick',[],'ztick',[])
  set(get(gca, 'XAxis'), 'visible', 'off');
  set(get(gca, 'YAxis'), 'visible', 'off');
  set(get(gca, 'ZAxis'), 'visible', 'off');
  %% 3D-Palletizer (3 Loops)
  if iRob == 1
    % See systems/palh1m1/palh1m1_test.m
    % in https://github.com/SchapplM/robsynth-serhybroblib
    RS_TE = hybroblib_create_robot_class('palh1m1', 'TE', 'palh1m1Bsp1');
    % Mark passive joints as active because they graphically overlap the
    % existing active joints at the same position.
    RS_TE.MDH.mu(8) = true;
    RS_TE.MDH.mu(6) = true;
    q = RS_TE.qref;
    s_plot = struct( 'ks', [], 'straight', 0);
    RS_TE.plot( q, s_plot );
    view([-20, 20])
    name = sprintf('palletizer_3d_3loops');
  end
  %% 3D-Palletizer (2 Loops)
  if iRob == 2
    % See systems/palh1m1/palh1m1_test.m
    % in https://github.com/SchapplM/robsynth-serhybroblib
    RS_TE = hybroblib_create_robot_class('palh3m1', 'TE', 'palh3m1Bsp1');
    % Mark passive joints as active because of graphically overlapping
    RS_TE.MDH.mu(7) = true;
    q = RS_TE.qref;
    s_plot = struct( 'ks', [], 'straight', 0);
    RS_TE.plot( q, s_plot );
    view([-20, 20])
    name = sprintf('palletizer_3d_2loops');
  end
  %% 2D-Palletizer
  if iRob == 3
    % See systems/picker2Dm1/picker2Dm1_test.m
    % in https://github.com/SchapplM/robsynth-serhybroblib
    RS = hybroblib_create_robot_class('picker2Dm1', 'TE', 'TSR2040');
    q = pi/180*[0; -30];
    s_plot = struct( 'ks', [], 'straight', 0);
    RS.plot( q, s_plot );
    view([-12, 56])
    name = sprintf('palletizer_2d');
  end
  %% Serial Industrial Robot
  if iRob == 4
    RS = serroblib_create_robot_class('S6RRRRRR10V2', 'S6RRRRRR10V2_KUKA1');
    s_plot = struct( 'ks', [], 'straight', 0);
    q = [0; 90; 0; 0; 90; 0]*pi/180;
    RS.plot( q, s_plot );
    view(3);
    name = sprintf('industrial_robot_6dof');
  end
  %% SCARA
  if iRob == 5
    RS = serroblib_create_robot_class('S4RRPR1', 'S4RRPR1_KUKA1');
    RS.DesPar.joint_offset = [0;0;-50e-3;0];
    RS.update_EE([0;0;-100e-3]);
    s_plot = struct( 'ks', [], 'straight', 0);
    q = [0; 0; -250e-3; 0];
    RS.plot( q, s_plot );
    view(3);
    name = sprintf('scara');
  end
  %% Serial Palletizing Robot
  if iRob == 6
    % Use model with kinematic constraints for the palletizing task
    RS = hybroblib_create_robot_class('palh2m1', 'DE', 'palh1m2KR1');
    % Set dependent joint as active because real robot has a motor there
    RS.MDH.mu(4) = true;
    % alternative: serial robot model
%     RS = serroblib_create_robot_class('S5RRRRR1', 'S5RRRRR1_KUKA1');
%     q = [0; -90+15; 90-15; 0; 0]*pi/180;
    s_plot = struct( 'ks', [], 'straight', 0);
    q = [0; -90-15; 90+45; 0;]*pi/180;
    RS.plot( q, s_plot );
    view(3);
    name = sprintf('palletizer_serial_5dof');
  end
  %% Parallel Robot 1 (3PRRU)
  if iRob == 7
    RP = parroblib_create_robot_class('P3PRRRR8V2G4P2A1', 1, 1);
    RP.align_base_coupling(4, [0.12;pi]);
    RP.align_platform_coupling(2, 0.1);
    % Parameters for the robot. Determined by the dimensional synthesis and
    % visual inspection
    RP.update_base([0;0;0.7]);
    pn = RP.Leg(1).pkin_names;
    for k = 1:RP.NLEG
      pkin = RP.Leg(1).pkin;
      pkin(strcmp(pn, 'a2')) = 0;
      pkin(strcmp(pn, 'a3')) = 0.2;
      pkin(strcmp(pn, 'a4')) = 0.3016;
      pkin(strcmp(pn, 'alpha2')) = pi/2;
      pkin(strcmp(pn, 'd2')) = 0;
      pkin(strcmp(pn, 'd3')) = 0.0837;
      pkin(strcmp(pn, 'd4')) = -0.1151;
      pkin(strcmp(pn, 'theta1')) = pi/2;
      RP.Leg(k).update_mdh(pkin);
    end
    x0 = [[0;0;0.110-0.7]; [0;0;0]];
    q0ik = [[-0.1157; 1.5492; 0.0000; 3.1416; 0.0216]; NaN(10,1)];
    [q0, Phi] = RP.invkin_ser(x0, q0ik);
    s_plot = struct( 'ks_legs', [], 'ks_platform', [], 'straight', 1);
    RP.plot(q0, x0, s_plot);
    name = sprintf('pkm_3dof');
    ch = get(gca, 'children');
    delete(ch(strcmp(get(ch, 'type'), 'hgtransform'))) % delete frame
    view([-22, 20])
  end
  %% Parallel Robot 2 (3RUU, Delta)
  if iRob == 8
    % Siehe ParRob_class_example_Delta.m
    RP = parroblib_create_robot_class('P3RRRRR2G2P2A1', 1, 1);
    RP.align_base_coupling(2, 0.2);
    RP.align_platform_coupling(2, 0.15);
    pn = RP.Leg(1).pkin_names;
    for k = 1:RP.NLEG
      pkin = RP.Leg(1).pkin;
      pkin(strcmp(pn, 'a2')) = 0.25;
      pkin(strcmp(pn, 'a4')) = 0.7;
      pkin(strcmp(pn, 'alpha2')) = 0;
      pkin(strcmp(pn, 'd1')) = 0;
      pkin(strcmp(pn, 'd2')) = 0;
      pkin(strcmp(pn, 'd4')) = 0;
      RP.Leg(k).update_mdh(pkin);
    end
    x0 = [0.00;0.00;-0.5; 0; 0; 0];
    q0ik = [[50; 150; 0; 180; -30]*pi/180; NaN(10,1)];
    [q0, Phi] = RP.invkin_ser(x0, q0ik);
    s_plot = struct( 'ks_legs', [], 'ks_platform', [], 'straight', 1);
    RP.plot(q0, x0, s_plot);
    name = sprintf('pkm_3dof_delta');
    ch = get(gca, 'children');
    delete(ch(strcmp(get(ch, 'type'), 'hgtransform'))) % delete frame
    view([-105, 12])
  end
  %% Parallel Robot 3 (3RRR)
  if iRob == 9
    % Siehe ParRob_class_example_3RRR.m
    RP = parroblib_create_robot_class('P3RRR1G1P1A1', 1, 0.3);
    pn = RP.Leg(1).pkin_names;
    for k = 1:RP.NLEG
      pkin = zeros(size(RP.Leg(1).pkin));
      pkin(strcmp(pn, 'a2')) = 0.6;
      pkin(strcmp(pn, 'a3')) = 0.6;
      RP.Leg(k).update_mdh(pkin);
    end
    x0 = [ [0.0;0.0;0.0]; [0;0;0]*pi/180 ];
    q0ik = [[30;100;-140]*pi/180; NaN(6,1)];
    [q0, Phi] = RP.invkin_ser(x0, q0ik);
    s_plot = struct( 'ks_legs', [], 'ks_platform', [], 'straight', 1);
    RP.plot(q0, x0, s_plot);
    name = sprintf('pkm_3dof_planar');
    ch = get(gca, 'children');
    delete(ch(strcmp(get(ch, 'type'), 'hgtransform'))) % delete frame
  end
  %% Parallel Robot 4 (6UPS)
  if iRob == 10
    RP = parroblib_create_robot_class('P6RRPRRR14V3G1P4A1', 0.5, 0.2);
    x0 = [ [0.15;0.05;0.5]; [10;-10;5]*pi/180 ];
    [q0, Phi] = RP.invkin_ser(x0, rand(RP.NJ,1));
    s_plot = struct( 'ks_legs', [], 'ks_platform', [], 'straight', 0);
    RP.plot(q0, x0, s_plot);
    name = sprintf('pkm_6dof_hexapod');
    ch = get(gca, 'children');
    delete(ch(strcmp(get(ch, 'type'), 'hgtransform'))) % delete frame
  end
  %% Finish Plot and Save
  figure_format_publication(gca)
  set(gca, 'Box', 'off');
  set_size_plot_subplot(f, ...
    8,8,gca,...
    0,0,0,0,0,0)
  cd(figure_dir);
  export_fig(['robot_example_', name, '_r800.png'], '-r800')
end
return
%% Debug: Generate Kinematic Parameters for the 3T0R Parallel Robot

Set = cds_settings_defaults(struct('DoF', [1 1 1 0 0 0])); %#ok<UNRCH>
Traj = cds_gen_traj([1 1 1 0 0 0], 1, Set.task);
Set.optimization.objective = 'energy';
Set.optimization.optname = 'maple2020_parrobexample';
Set.optimization.constraint_collisions = true; % shall look plausible
Set.optimization.base_size_limits = [0.100, 0.400];
Set.optimization.platform_size_limits = [0.080, 0.250];
Set.optimization.basepos_limits = [[0, 0]; [0, 0]; [0.5 1.5]];
Set.structures.whitelist = {'P3PRRRR8V2G4P2A1'}; % Type 3PRUR
Set.general.debug_calc = false;
Set.general.matfile_verbosity = 3;
Set.general.verbosity = 3;
Set.general.plot_robot_in_fitness = 1e3; % plot successful intermediate results
% The whole robot has to fit into a cylinder
h_cylinder = 1.00; % height
r_cylinder = 0.40; % radius
p_cylinder = [[0,0,0], [0,0,h_cylinder], r_cylinder, NaN(1,3)];
Set.task.installspace = struct( ...
  'type', uint8(2), ... % see implementation in cds
  'params', p_cylinder, ...
  'links', {{0:6}}); % all joints in cylinder
% Set.general.create_template_functions = true;
cds_start
% Load results of the simulation. Afterwards, paste them in the init- 
% ialization of the robot in the first section of this script.
d = load(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
  'results', 'maple2020_parrobexample'));
% Debug:
d = load('/mnt/FP500/IMES/PRJ/imes-projekt-dfg_robotersynthese/03_Entwicklung/match_Kryo_PKM/Versuch_20200808_Nachts/Rob27_P3PRRRR8V2G4P2A1_Endergebnis.mat');
% Show suitable parameters for the robot
R = d.RobotOptRes.R;
R.T_W_0
R.Leg(1).pkin_names
R.Leg(1).pkin'
R.DesPar.base_par'
R.DesPar.platform_par(1:end-1)'
R.Leg(1).qref
R.Leg(1).qlim
d.RobotOptRes.Structure.varnames
d.RobotOptRes.p_val
d.Traj.X(1,:)
