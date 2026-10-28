root=fileparts(fileparts(fileparts(mfilename('fullpath'))));cd(root);addpath(fullfile(root,'pipeline'));init_paths(root);addpath(fullfile(root,'experiment_manager'));
r=runtests(fullfile(root,'tests'),'IncludeSubfolders',false);
rows=repmat(struct('name','','passed',false,'failed',false,'incomplete',false,'duration_s',0),numel(r),1);
for k=1:numel(r),rows(k)=struct('name',r(k).Name,'passed',r(k).Passed,'failed',r(k).Failed,'incomplete',r(k).Incomplete,'duration_s',r(k).Duration);end
report=struct('release','0.3.0','matlab_version',version,'scope','all top-level tests, same discovery as run_all_tests; experiment_manager path initialized', ...
 'total',numel(r),'passed',sum([r.Passed]),'failed',sum([r.Failed]),'incomplete',sum([r.Incomplete]),'tests',rows);
p=fullfile(root,'results','repair_20261027','release_v0.3.0_full_validation.json');fid=fopen(p,'w','n','UTF-8');fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
fprintf('V030_FULL_GATE total=%d passed=%d failed=%d incomplete=%d\n',report.total,report.passed,report.failed,report.incomplete);