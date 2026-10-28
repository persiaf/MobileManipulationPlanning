function summary = exp_dynamic_local_replanning_p5(options)
%EXP_DYNAMIC_LOCAL_REPLANNING_P5 Actual paired replanning on fresh geometry.
% Historical frozen-reference estimates are no longer a supported execution
% mode. Raw rows include failures and unknown IK/measured completion rates.
    if nargin<1,options=struct();end
    if isfield(options,'run_full_replan') && ~options.run_full_replan
        error('exp_dynamic_local_replanning_p5:EstimatedReferenceRemoved', ...
            'A frozen static timing estimate cannot serve as a dynamic full-replan comparison.');
    end
    raw=run_protocol_dynamic_comparison(options);
    if isempty(raw),summary=table();return;end
    summary=groupsummary(raw,{'event_type','mode','status'},'mean', ...
        {'geometric_coverage_percent','path_length_m','replanning_time_s'});
    root=fileparts(fileparts(mfilename('fullpath')));
    out=fullfile(root,'results','repair_20261027','dynamic');
    if isfield(options,'output_dir'),out=options.output_dir;end
    writetable(summary,fullfile(out,'dynamic_summary.csv'));
end
