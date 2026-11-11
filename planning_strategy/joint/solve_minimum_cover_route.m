function solution=solve_minimum_cover_route(group_cover_sets,D,startIndex,options)
%SOLVE_MINIMUM_COVER_ROUTE Enumerate cover products and every closed tour.
% Each group is a matrix of alternative minimum covers, expressed as node
% indices into D. D may be directed and contain Inf. No branch is truncated.
% The certificate is exact for Hamiltonian closed tours on these covers and
% this supplied cost oracle: each selected dock is visited exactly once.
% No finite tour returns infeasible; larger covers or repeated transit docks
% are outside this minimum-cover-layer solver.
    if nargin<4,options=struct();end
    limits=struct('max_selected_docks',8,'max_cover_products',100000,'max_route_evaluations',1e6);
    names=fieldnames(limits);
    for i=1:numel(names)
        key=names{i};if isfield(options,key),limits.(key)=options.(key);end
        validateattributes(limits.(key),{'numeric'},{'real','finite','scalar','integer','positive'});
    end
    if ~isnumeric(D)||~isreal(D)||ndims(D)~=2||size(D,1)~=size(D,2)|| ...
            isempty(D)||any(isnan(D(:)))||any(D(:)<0)||any(diag(D)~=0)
        error('solve_minimum_cover_route:CostMatrix','D must be square, nonnegative, NaN-free and zero-diagonal.');
    end
    validateattributes(startIndex,{'numeric'},{'real','finite','scalar','integer','>=',1,'<=',size(D,1)});
    if ~iscell(group_cover_sets)
        error('solve_minimum_cover_route:Groups','group_cover_sets must be a cell array.');
    end
    solution=struct('feasible',false,'exact',true,'selected_indices',[], ...
        'visit_order',[],'closed_route',[],'route_cost',Inf,'minimum_dock_count',0, ...
        'cover_product_count',0,'routes_evaluated',0,'reason','NO_FINITE_MINIMUM_COVER_TOUR', ...
        'scope',['exact Hamiltonian closed tours over supplied cover products and cost matrix; ' ...
            'each selected dock visited once; no larger-cover or transit-revisit search; ' ...
            'no continuous-space or physical optimality']);
    groups=reshape(group_cover_sets,1,[]);count=zeros(size(groups));k=0;used=[];
    for gi=1:numel(groups)
        C=groups{gi};
        if ~isnumeric(C)||~isreal(C)||ndims(C)~=2||any(~isfinite(C(:)))|| ...
                any(C(:)~=fix(C(:)))||any(C(:)<1)||any(C(:)>size(D,1))||any(C(:)==startIndex)
            error('solve_minimum_cover_route:Groups','Cover entries must be valid non-start node indices.');
        end
        if size(C,1)==0,solution.reason='UNCOVERED_GROUP';return;end
        for row=1:size(C,1)
            if numel(unique(C(row,:)))~=size(C,2)
                error('solve_minimum_cover_route:Groups','A cover repeats a node.');
            end
        end
        nodes=unique(C(:))';
        if ~isempty(intersect(used,nodes))
            error('solve_minimum_cover_route:Groups','Groups must own disjoint candidate nodes.');
        end
        used=[used,nodes]; %#ok<AGROW>
        if size(C,2)==0,groups{gi}=zeros(1,0);
        else,groups{gi}=unique(sort(C,2),'rows','sorted');end
        count(gi)=size(groups{gi},1);k=k+size(C,2);
    end
    solution.minimum_dock_count=k;
    if k>limits.max_selected_docks
        error('solve_minimum_cover_route:BudgetExceeded','Selected-dock permutation budget exceeded.');
    end
    product=1;
    for gi=1:numel(count)
        product=product*count(gi);
        if product>limits.max_cover_products
            error('solve_minimum_cover_route:BudgetExceeded','Cover-product budget exceeded.');
        end
    end
    permutations_count=factorial(k);
    if product*permutations_count>limits.max_route_evaluations
        error('solve_minimum_cover_route:BudgetExceeded','Exact route-evaluation budget exceeded.');
    end
    solution.cover_product_count=product;
    if k==0
        solution.feasible=true;solution.route_cost=0;solution.visit_order=startIndex;
        solution.closed_route=[startIndex,startIndex];solution.reason='';return;
    end
    permutations=sortrows(perms(1:k));choice=ones(1,numel(groups));
    for combination=1:product
        selected=[];
        for gi=1:numel(groups),selected=[selected,groups{gi}(choice(gi),:)];end %#ok<AGROW>
        selected=sort(selected);
        for pi=1:size(permutations,1)
            visit=[startIndex,selected(permutations(pi,:))];closed=[visit,startIndex];
            cost=sum(D(sub2ind(size(D),closed(1:end-1),closed(2:end))));
            solution.routes_evaluated=solution.routes_evaluated+1;
            if isfinite(cost) && (cost<solution.route_cost || ...
                    (cost==solution.route_cost && lexicographically_less(visit,solution.visit_order)))
                solution.feasible=true;solution.route_cost=cost;
                solution.selected_indices=selected;solution.visit_order=visit;
                solution.closed_route=closed;solution.reason='';
            end
        end
        for gi=numel(groups):-1:1
            choice(gi)=choice(gi)+1;
            if choice(gi)<=count(gi),break;end
            choice(gi)=1;
        end
    end
end

function yes=lexicographically_less(a,b)
    if isempty(b),yes=true;return;end
    first=find(a~=b,1);yes=~isempty(first)&&a(first)<b(first);
end
