root=fileparts(mfilename('fullpath'));
repo=fileparts(fileparts(root));
addpath(root,fullfile(repo,'config'),fullfile(repo,'manipulator'),fullfile(repo,'utils'));
outdir=fullfile(root,'figures'); if ~exist(outdir,'dir'),mkdir(outdir);end
cfg=robot_config();
fig_coordinates(outdir,cfg);
fig_proxy(outdir,cfg);
fig_c2(outdir,repo);
disp('THESIS_FIGURES_COMPLETE');
