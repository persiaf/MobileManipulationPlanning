function combined=collect_task_navigation_robustness(output_dir)
%COLLECT_TASK_NAVIGATION_ROBUSTNESS Aggregate without dropping failed rows.
    if nargin<1,output_dir=fullfile('results','task_navigation_robustness_20261111');end
    manifest=readtable(fullfile(output_dir,'scenario_manifest.csv'),'TextType','string');
    combined=table();complete=0;rows_by_case=zeros(height(manifest),1);state_labels=strings(height(manifest),1);
    for i=1:height(manifest)
        folder=fullfile(output_dir,char(manifest.relative_dir(i)));state_labels(i)="PENDING";
        if isfile(fullfile(folder,'run_state.json'))
            state=jsondecode(fileread(fullfile(folder,'run_state.json')));state_labels(i)=string(state.status);
            complete=complete+strcmp(state.status,'COMPLETE');
        end
        if ~isfile(fullfile(folder,'summary.csv')),continue;end
        summary_path=fullfile(folder,'summary.csv');import=detectImportOptions(summary_path,'TextType','string');
        import=setvartype(import,{'mode','method','status','failure_reason','termination_reason'},'string');
        current=readtable(summary_path,import);n=height(current);rows_by_case(i)=n;
        prefix=table(repmat(manifest.case_id(i),n,1),repmat(manifest.layout_id(i),n,1), ...
            repmat(manifest.distribution_id(i),n,1),repmat(manifest.seed(i),n,1), ...
            'VariableNames',{'case_id','layout_id','distribution_id','seed'});
        current=[prefix,current];if isempty(combined),combined=current;else,combined=[combined;current];end %#ok<AGROW>
    end
    if ~isempty(combined),writetable(combined,fullfile(output_dir,'batch_summary.csv'));end
    progress=[manifest,table(state_labels,rows_by_case,'VariableNames',{'status','completed_rows'})];
    writetable(progress,fullfile(output_dir,'scenario_progress.csv'));
    status='RUNNING';if complete==height(manifest),status='COMPLETE';end
    state=struct('status',status,'scenario_count',height(manifest),'completed_scenarios',complete, ...
        'completed_rows',height(combined),'planned_raw_rows',108,'planned_primary_rows',81);
    if strcmp(status,'COMPLETE'),assert(height(combined)==108,'Robustness:MissingRows','Completed scenarios must retain all planned method rows.');end
    fid=fopen(fullfile(output_dir,'run_state.json'),'w','n','UTF-8');assert(fid>=0);
    cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(state,'PrettyPrint',true)); %#ok<NASGU>
    disp(progress(:,{'case_id','status','completed_rows'}));
end
