% Generate images from robots which have a dynamics model from the toolbox

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
% close all
figure_dir = fileparts(which('robot_examples.m'));

for iRob = 1:2
  f = figure(2);clf;
  hold on;
  grid off;
  view(3);
  set(gca,'XTICKLABEL',{}, 'YTICKLABEL', {}, 'ZTICKLABEL',{});
  set(gca,'xtick',[],'ytick',[],'ztick',[])
  set(get(gca, 'XAxis'), 'visible', 'off');
  set(get(gca, 'YAxis'), 'visible', 'off');
  %% 3D-Palletizer
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
    name = sprintf('palletizer_3d');
  end
  %% 2D-Palletizer
  if iRob == 2
    % See systems/picker2Dm1/picker2Dm1_test.m
    % in https://github.com/SchapplM/robsynth-serhybroblib
    RS = hybroblib_create_robot_class('picker2Dm1', 'TE', 'TSR2040');
    q = pi/180*[0; -30];
    s_plot = struct( 'ks', [], 'straight', 0);
    RS.plot( q, s_plot );
    view([-12, 56])
    name = sprintf('palletizer_2d');
  end
  %% Parallel Robot
  if iRob == 3
    % TODO: Find out which PKM has code and looks interesting
  end
  %% Finish Plot and Save
  figure_format_publication(gca)
  set(gca, 'Box', 'off');
  set_size_plot_subplot(f, ...
    8,8,gca,...
    0,0,0,0,0,0)
  export_fig(fullfile(figure_dir, [name, '.pdf']));
  export_fig(fullfile(figure_dir, [name, '.png']));
  cd(figure_dir);
  export_fig([name, '_r864.png'], '-r864')
end
