function [covers,certificate]=enumerate_minimum_task_covers(candidates,task_ids,options)
%ENUMERATE_MINIMUM_TASK_COVERS All cardinality-minimum covers of a finite pool.
% Rows contain local candidate indices. Exactness concerns the supplied
% binary support sets, not IK completeness or continuous-space feasibility.
    if nargin<3,options=struct();end
    limits=struct('max_task_count',16,'max_enumerated_combinations',1e6,'max_covers',100000);
    names=fieldnames(limits);
    for i=1:numel(names)
        key=names{i};if isfield(options,key),limits.(key)=options.(key);end
        validateattributes(limits.(key),{'numeric'},{'real','finite','scalar','integer','positive'});
    end
    if ~isempty(task_ids)
        validateattributes(task_ids,{'numeric'},{'real','finite','vector','integer','positive'});
    elseif ~isnumeric(task_ids)
        error('enumerate_minimum_task_covers:TaskIds','Task IDs must be numeric.');
    end
    task_ids=reshape(task_ids,1,[]);m=numel(task_ids);
    if numel(unique(task_ids))~=m
        error('enumerate_minimum_task_covers:TaskIds','Task IDs must be unique.');
    end
    if m>limits.max_task_count || m>20
        error('enumerate_minimum_task_covers:BudgetExceeded','Task-mask state budget exceeded.');
    end
    certificate=struct('feasible',false,'exact',true,'minimum_dock_count',Inf, ...
        'cover_count',0,'core_candidate_indices',[],'candidate_masks',zeros(1,numel(candidates),'uint32'), ...
        'task_ids',task_ids,'enumerated_combinations',0, ...
        'scope','exact minimum set-cover cardinality on the supplied finite support matrix');
    if m==0
        covers=zeros(1,0);certificate.feasible=true;certificate.minimum_dock_count=0;
        certificate.cover_count=1;return;
    end
    masks=zeros(1,numel(candidates),'uint32');
    for ci=1:numel(candidates)
        ids=candidates(ci).covered_tasks;
        if ~isnumeric(ids) || any(~isfinite(ids(:))) || any(~ismember(ids(:),task_ids))
            error('enumerate_minimum_task_covers:CandidateTasks','Candidate support contains unknown task IDs.');
        end
        for ti=1:m
            if any(ids==task_ids(ti)),masks(ci)=bitset(masks(ci),ti);end
        end
    end
    certificate.candidate_masks=masks;
    full=uint32(2^m-1);union_mask=uint32(0);
    for ci=1:numel(masks),union_mask=bitor(union_mask,masks(ci));end
    covers=zeros(0,0);
    if union_mask~=full,return;end
    costs=inf(1,2^m);costs(1)=0;
    active=find(masks>0);
    for state=0:double(full)
        if ~isfinite(costs(state+1)),continue;end
        for ci=active
            next=double(bitor(uint32(state),masks(ci)))+1;
            costs(next)=min(costs(next),costs(state+1)+1);
        end
    end
    k=costs(end);combination_count=1;
    for j=1:k,combination_count=combination_count*(numel(active)-j+1)/j;end
    if combination_count>limits.max_enumerated_combinations
        error('enumerate_minimum_task_covers:BudgetExceeded', ...
            'Enumerating %.0f candidate subsets exceeds max_enumerated_combinations.',combination_count);
    end
    combinations=nchoosek(active,k);keep=false(size(combinations,1),1);
    for row=1:size(combinations,1)
        mask=uint32(0);
        for col=1:k,mask=bitor(mask,masks(combinations(row,col)));end
        keep(row)=mask==full;
    end
    if sum(keep)>limits.max_covers
        error('enumerate_minimum_task_covers:BudgetExceeded','Minimum-cover output budget exceeded.');
    end
    covers=sortrows(combinations(keep,:));
    certificate.feasible=true;certificate.minimum_dock_count=k;
    certificate.cover_count=size(covers,1);certificate.core_candidate_indices=unique(covers(:))';
    certificate.enumerated_combinations=size(combinations,1);
end
