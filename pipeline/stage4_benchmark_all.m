function benchmark_results = stage4_benchmark_all(scene, all_poses, optimal_poses, planner_cfg, stats)
%STAGE4_BENCHMARK_ALL Multi-algorithm global path planning benchmark.
%   Supports fixed_order (shared route_order) and independent_order
%   (each algorithm computes own distance matrix + visit order).
%   Supports optional include_return_to_start for return segment.
%   Collects fallback_count / collision_segments without flooding console.

if nargin < 5
    stats = struct();
end

% --- Read benchmark configuration ---
if isfield(planner_cfg, 'benchmark') && isfield(planner_cfg.benchmark, 'enable') && ...
        ~planner_cfg.benchmark.enable
    fprintf('\n===== 多算法全局路径对比 benchmark 已关闭 =====\n');
    benchmark_results = struct([]);
    return;
end

order_mode = 'fixed_order';
if isfield(planner_cfg, 'benchmark') && isfield(planner_cfg.benchmark, 'order_mode')
    order_mode = planner_cfg.benchmark.order_mode;
end
if ~any(strcmpi(order_mode,{'fixed_order','independent_order'}))
    error('stage4_benchmark_all:OrderMode','Unknown benchmark order mode "%s".',order_mode);
end

include_return = true;
if isfield(planner_cfg, 'benchmark') && isfield(planner_cfg.benchmark, 'include_return_to_start')
    include_return = planner_cfg.benchmark.include_return_to_start;
end

ret_str = '否';
if include_return, ret_str = '是'; end

fprintf('\n===== 多算法全局路径对比 benchmark =====\n');
fprintf('  顺序模式: %s\n', order_mode);
fprintf('  包含返航: %s\n', ret_str);

% --- Algorithm list ---
core_algorithms = {'astar', 'jps', 'gbfs', 'theta_star'};
extended_algorithms = {'dijkstra', 'lazy_theta_star', 'dstar', 'lpa_star', ...
                       'voronoi', 'dstar_lite', 'hybrid_astar'};
stochastic_algorithms = {'rrt', 'rrt_star', 'rrt_connect', 'informed_rrt', ...
                         'aco', 'pso', 'ga'};

if isfield(planner_cfg, 'benchmark') && isfield(planner_cfg.benchmark, 'algorithms') && ...
        ~isempty(planner_cfg.benchmark.algorithms)
    algorithms = planner_cfg.benchmark.algorithms;
else
    algorithms = core_algorithms;
end

include_extended = isfield(planner_cfg, 'benchmark') && ...
                   isfield(planner_cfg.benchmark, 'include_extended') && ...
                   planner_cfg.benchmark.include_extended;
include_stochastic = isfield(planner_cfg, 'benchmark') && ...
                     isfield(planner_cfg.benchmark, 'include_stochastic') && ...
                     planner_cfg.benchmark.include_stochastic;

if include_extended
    algorithms = [algorithms, extended_algorithms];
end
if include_stochastic
    algorithms = [algorithms, stochastic_algorithms];
end
if isfield(planner_cfg, 'benchmark') && isfield(planner_cfg.benchmark, 'include_convex_astar') && ...
        planner_cfg.benchmark.include_convex_astar
    algorithms = [{'convex_astar'}, algorithms];
end
algorithms = unique(algorithms, 'stable');
n_algos = length(algorithms);

% --- Base poses ---
base_poses = [scene.start_pose; optimal_poses];

% --- Build fixed route_order for fixed_order mode ---
fixed_route_order = [];
if strcmpi(order_mode, 'fixed_order')
    if isfield(stats, 'route_order') && ~isempty(stats.route_order)
        fixed_route_order = stats.route_order;
    elseif isfield(stats, 'visit_order') && ~isempty(stats.visit_order)
        fixed_route_order = stats.visit_order;
        if include_return && fixed_route_order(end) ~= 1
            fixed_route_order = [fixed_route_order, 1];
        end
    else
        error('stage4_benchmark_all: fixed_order 需要主流程 stats.route_order 或 stats.visit_order');
    end
    if include_return && fixed_route_order(end) ~= 1
        fixed_route_order = [fixed_route_order, 1];
    elseif ~include_return && numel(fixed_route_order)>1 && fixed_route_order(end) == 1
        fixed_route_order = fixed_route_order(1:end-1);
    end
    fixed_route_order=reshape(fixed_route_order,1,[]);
    if any(~isfinite(fixed_route_order)) || any(fixed_route_order~=fix(fixed_route_order)) || ...
            any(fixed_route_order<1 | fixed_route_order>size(base_poses,1)) || fixed_route_order(1)~=1
        error('stage4_benchmark_all:VisitOrder','Fixed route order must start at 1 and contain valid pose indices.');
    end
    fprintf('  fixed_order route_order = [%s]\n', num2str(fixed_route_order));
end

% --- 2-opt toggle ---
enable_2opt = false;
if isfield(planner_cfg, 'enable_2opt')
    enable_2opt = planner_cfg.enable_2opt;
end

% --- Global warning suppression (belt and suspenders) ---
suppress_warnings = true;
if isfield(planner_cfg, 'benchmark') && isfield(planner_cfg.benchmark, 'suppress_warnings')
    suppress_warnings = planner_cfg.benchmark.suppress_warnings;
end
run_opts.suppress_warnings = suppress_warnings;
if suppress_warnings
    warning_state=warning('query','run_path_planner:collision');
    warning('off', 'run_path_planner:collision');
    warning_cleanup=onCleanup(@() warning(warning_state.state,'run_path_planner:collision')); %#ok<NASGU>
end

% --- Print table header ---
fprintf('\n  %-14s %10s %10s %10s %10s %10s %10s %8s %8s\n', ...
        '算法', '路径(m)', '任务(m)', '返航(m)', '矩阵(s)', '拼接(s)', '总时(s)', '回退', '路径点');
fprintf('  %s\n', repmat('-', 1, 98));

% --- Preallocate result struct ---
benchmark_results = struct(...
    'algorithm', {}, ...
    'requested_algorithm', {}, ...
    'observed_actual_planners', {}, ...
    'actual_planner_known', {}, ...
    'planner_query_provenance_json', {}, ...
    'path_query_count', {}, ...
    'matrix_query_count', {}, ...
    'route_query_count', {}, ...
    'failed_query_count', {}, ...
    'timing_scope', {}, ...
    'comparison_level', {}, ...
    'excluded_time_scope', {}, ...
    'return_to_start', {}, ...
    'return_segment_completed', {}, ...
    'threshold', {}, ...
    'required_nodes_visited', {}, ...
    'num_required_segments', {}, ...
    'num_attempted_segments', {}, ...
    'num_successful_segments', {}, ...
    'failure_reason', {}, ...
    'order_mode', {}, ...
    'path_length_m', {}, ...
    'task_path_length_m', {}, ...
    'return_path_length_m', {}, ...
    'distance_matrix_time_s', {}, ...
    'order_time_s', {}, ...
    'stitch_time_s', {}, ...
    'total_time_s', {}, ...
    'num_path_points', {}, ...
    'num_turns', {}, ...
    'collision_segments', {}, ...
    'fallback_count', {}, ...
    'success', {});

% --- Main benchmark loop ---
for ai = 1:n_algos
    algo = algorithms{ai};
    fprintf('  %-14s ', algo);

    t_total = tic;
    total_fb = 0;     % accumulated fallback count
    total_col = 0;    % accumulated collision segments
    provenance=empty_provenance();

    if strcmpi(order_mode, 'fixed_order')
        % === fixed_order: use shared route_order ===
        route_order = fixed_route_order;
        dist_mat_time = 0;
        order_time = 0;

        t_stitch = tic;
        [global_path, seg_stats, seg_fb, seg_col,route_provenance] = stitch_for_algo(...
            base_poses, route_order, scene.grid_map, scene.map_info, ...
            planner_cfg, algo, run_opts);
        stitch_time = toc(t_stitch);

        total_fb = total_fb + seg_fb;
        total_col = total_col + seg_col;
        provenance=[provenance,route_provenance];
    else
        % === independent_order: each algorithm independent ===
        n_poses = size(base_poses, 1);

        % Edge-cost lambda
        lambda_theta = 0;
        if isfield(planner_cfg, 'edge_cost') && isfield(planner_cfg.edge_cost, 'lambda_theta')
            lambda_theta = planner_cfg.edge_cost.lambda_theta;
        end

        % Step 1: distance matrix
        t_dist = tic;
        dist_mat = inf(n_poses, n_poses);
        for i = 1:n_poses
            for j = i+1:n_poses
                [seg_path, fb_info] = run_path_planner(scene.grid_map, ...
                    base_poses(i,:), base_poses(j,:), scene.map_info, ...
                    planner_cfg, algo, run_opts);
                provenance(end+1)=query_provenance(fb_info,algo,'distance_matrix',i,j,seg_path); %#ok<AGROW>
                d = compute_path_length(seg_path);
                if size(seg_path,1)<2 || ~provenance(end).path_valid,d=inf;end
                if d > 1e6 || isnan(d), d = inf; end
                if isfinite(d) && lambda_theta > 0
                    dtheta = abs(wrapToPiLocal(base_poses(j,3) - base_poses(i,3)));
                    d = d + lambda_theta * dtheta;
                end
                dist_mat(i,j) = d; dist_mat(j,i) = d;
                total_fb = total_fb + fb_info.fallback_count;
                total_col = total_col + fb_info.collision_segments;
            end
        end
        dist_mat_time = toc(t_dist);

        % Step 2: visit order
        t_order = tic;
        visit_order = nearest_neighbor_order(dist_mat, 1, enable_2opt);
        route_order = visit_order;
        if include_return && route_order(end) ~= 1
            route_order = [route_order, 1];
        end
        order_time = toc(t_order);

        % Step 3: stitch
        t_stitch = tic;
        [global_path, seg_stats, seg_fb2, seg_col2,route_provenance] = stitch_for_algo(...
            base_poses, route_order, scene.grid_map, scene.map_info, ...
            planner_cfg, algo, run_opts);
        stitch_time = toc(t_stitch);

        total_fb = total_fb + seg_fb2;
        total_col = total_col + seg_col2;
        provenance=[provenance,route_provenance];
    end

    total_time = toc(t_total);

    % --- Compute aggregate stats from segment_stats ---
    total_path_len = 0;
    task_path_len = 0;
    return_path_len = 0;
    num_path_points = size(global_path, 1);
    for sk = 1:length(seg_stats)
        total_path_len = total_path_len + seg_stats(sk).path_length_m;
        if seg_stats(sk).is_return
            return_path_len = return_path_len + seg_stats(sk).path_length_m;
        else
            task_path_len = task_path_len + seg_stats(sk).path_length_m;
        end
    end

    [~, num_turns] = estimate_path_turn_angle(global_path);
    required_nodes_visited=isequal(unique(reshape(route_order,1,[])),1:size(base_poses,1));
    required_segments=max(numel(route_order)-1,0);
    successful_segments=sum([seg_stats.path_valid]);
    success = required_nodes_visited && ~isempty(global_path) && size(global_path,1)>=2 && ...
        numel(seg_stats)==required_segments && successful_segments==required_segments && ...
        all(isfinite([seg_stats.path_length_m]));
    reason='';
    if ~required_nodes_visited,reason='VISIT_ORDER_OMITS_DOCKS';
    elseif ~success,reason='MISSING_OR_INVALID_ROUTE_SEGMENT';end
    observed=unique({provenance.actual_planner},'stable');
    actual_known=~isempty(provenance) && all([provenance.actual_planner_known]);
    if strcmpi(order_mode,'fixed_order')
        timing_scope='fixed_route_path_planner_queries+stitch';
        comparison_level='fixed_route_path_planner_pipeline';
        excluded='scene+candidate+selection+distance_matrix+visit_order_preprocessing+execution_simulation+oracle+rendering+file_io';
    else
        timing_scope='dense_pair_queries+visit_order+route_queries+stitch';
        comparison_level='fixed_docks_route_and_path_planning_pipeline';
        excluded='scene+candidate+selection+execution_simulation+oracle+rendering+file_io';
    end

    % --- Save result ---
    n_res = length(benchmark_results) + 1;
    benchmark_results(n_res).algorithm = algo;
    benchmark_results(n_res).requested_algorithm = algo;
    benchmark_results(n_res).observed_actual_planners = strjoin(observed,'|');
    benchmark_results(n_res).actual_planner_known = actual_known;
    benchmark_results(n_res).planner_query_provenance_json = jsonencode(provenance);
    benchmark_results(n_res).path_query_count = numel(provenance);
    benchmark_results(n_res).matrix_query_count = sum(strcmp({provenance.query_scope},'distance_matrix'));
    benchmark_results(n_res).route_query_count = sum(strcmp({provenance.query_scope},'route_segment'));
    benchmark_results(n_res).failed_query_count = sum(~[provenance.path_valid]);
    benchmark_results(n_res).timing_scope = timing_scope;
    benchmark_results(n_res).comparison_level = comparison_level;
    benchmark_results(n_res).excluded_time_scope = excluded;
    benchmark_results(n_res).return_to_start = include_return;
    benchmark_results(n_res).return_segment_completed = ~isempty(seg_stats) && seg_stats(end).is_return && seg_stats(end).path_valid;
    benchmark_results(n_res).threshold = field_or(field_or(planner_cfg,'reachability',struct()),'threshold',NaN);
    benchmark_results(n_res).required_nodes_visited = required_nodes_visited;
    benchmark_results(n_res).num_required_segments = required_segments;
    benchmark_results(n_res).num_attempted_segments = numel(seg_stats);
    benchmark_results(n_res).num_successful_segments = successful_segments;
    benchmark_results(n_res).failure_reason = reason;
    benchmark_results(n_res).order_mode = order_mode;
    benchmark_results(n_res).path_length_m = total_path_len;
    benchmark_results(n_res).task_path_length_m = task_path_len;
    benchmark_results(n_res).return_path_length_m = return_path_len;
    benchmark_results(n_res).distance_matrix_time_s = dist_mat_time;
    benchmark_results(n_res).order_time_s = order_time;
    benchmark_results(n_res).stitch_time_s = stitch_time;
    benchmark_results(n_res).total_time_s = total_time;
    benchmark_results(n_res).num_path_points = num_path_points;
    benchmark_results(n_res).num_turns = num_turns;
    benchmark_results(n_res).collision_segments = total_col;
    benchmark_results(n_res).fallback_count = total_fb;
    benchmark_results(n_res).success = success;

    fprintf('%10.2f %10.2f %10.2f %10.3f %10.3f %10.3f %8d %8d\n', ...
            total_path_len, task_path_len, return_path_len, ...
            dist_mat_time, stitch_time, total_time, total_fb, num_path_points);
end

% --- Summary table ---
fprintf('\n  -------- 汇总结果 --------\n');
fprintf('  %-14s %10s %10s %10s %10s %10s %10s %8s %8s\n', ...
        '算法', '路径(m)', '任务(m)', '返航(m)', '矩阵(s)', '拼接(s)', '总时(s)', '回退', '路径点');
fprintf('  %s\n', repmat('-', 1, 98));
for ai = 1:n_algos
    r = benchmark_results(ai);
    fprintf('  %-14s %10.2f %10.2f %10.2f %10.3f %10.3f %10.3f %8d %8d\n', ...
            r.algorithm, r.path_length_m, r.task_path_length_m, r.return_path_length_m, ...
            r.distance_matrix_time_s, r.stitch_time_s, r.total_time_s, ...
            r.fallback_count, r.num_path_points);
end

% --- Notes ---
fprintf('\n  注:\n');
if strcmpi(order_mode, 'fixed_order')
    fprintf('       fixed_order: 所有算法共用 route_order，公平对比路径质量\n');
else
    fprintf('       independent_order: 各算法独立计算距离矩阵和顺序\n');
end
fprintf('       路径长度包括起点→停靠→起点全程（若含返航），单位: m\n');
fprintf('       回退次数: 碰撞后回退到 A* 重规划的总次数\n');

end

% ===================== Subfunctions =====================

function [full_path, seg_stats, total_fb, total_col,provenance] = stitch_for_algo(...
    poses, route_order, grid_map, map_info, planner_cfg, algo_override, run_opts)
% Stitch path segments for one algorithm. Returns segment-level stats.

n_seg = length(route_order) - 1;
full_path = [];
seg_stats = struct('path_length_m', {}, 'is_return', {},'path_valid',{});
total_fb = 0;
total_col = 0;
provenance=empty_provenance();

for k = 1:n_seg
    from_idx = route_order(k);
    to_idx   = route_order(k + 1);

    [segment, fb_info] = run_path_planner(grid_map, poses(from_idx, :), ...
        poses(to_idx, :), map_info, planner_cfg, algo_override, run_opts);
    provenance(end+1)=query_provenance(fb_info,algo_override,'route_segment',from_idx,to_idx,segment); %#ok<AGROW>

    total_fb = total_fb + fb_info.fallback_count;
    total_col = total_col + fb_info.collision_segments;
    if size(segment,1)<2 || ~provenance(end).path_valid
        seg_stats(k).path_length_m=inf;
        seg_stats(k).is_return=(k==n_seg)&&(to_idx==1);
        seg_stats(k).path_valid=false;
        full_path=[];
        return;
    end

    % Record segment stats
    seg_stats(k).path_length_m = compute_path_length(segment);
    seg_stats(k).is_return = (k == n_seg) && (to_idx == 1);
    seg_stats(k).path_valid=true;

    if isempty(full_path)
        full_path = segment;
    elseif ~isempty(segment)
        full_path = [full_path; segment(2:end, :)]; %#ok<AGROW>
    end
end

if ~isempty(full_path)
    full_path = densify_path(full_path, map_info.resolution);
end
end

function rows=empty_provenance()
rows=struct('query_scope',{},'from_id',{},'to_id',{},'requested_planner',{}, ...
    'actual_planner',{},'actual_planner_known',{},'fallback_used',{}, ...
    'fallback_count',{},'collision_segments',{},'path_valid',{}, ...
    'postprocessing_repair_used',{},'fallback_reason',{},'failure_reason',{});
end

function row=query_provenance(info,requested,scope,from,to,path)
row=struct('query_scope',scope,'from_id',from,'to_id',to, ...
    'requested_planner',field_or(info,'requested_planner',requested), ...
    'actual_planner',field_or(info,'actual_planner','unreported'), ...
    'actual_planner_known',field_or(info,'actual_planner_known',false), ...
    'fallback_used',field_or(info,'fallback_used',false), ...
    'fallback_count',field_or(info,'fallback_count',0), ...
    'collision_segments',field_or(info,'collision_segments',0), ...
    'path_valid',size(path,1)>=2 && field_or(info,'path_valid',false), ...
    'postprocessing_repair_used',field_or(info,'postprocessing_repair_used',false), ...
    'fallback_reason',field_or(info,'fallback_reason',''), ...
    'failure_reason',field_or(info,'failure_reason',''));
end

function value=field_or(s,name,fallback)
if isstruct(s)&&isfield(s,name),value=s.(name);else,value=fallback;end
end
