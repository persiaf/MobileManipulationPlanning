function [ok, segment_cells] = build_convex_segment(p1, p2, traversable, order_mode)
%BUILD_CONVEX_SEGMENT Build a grid segment with no corner cutting.
%   order_mode='diag_first' uses diagonal steps first, then straight steps.
%   order_mode='straight_first' uses straight steps first, then diagonal steps.

    if nargin < 4
        order_mode = 'diag_first';
    end

    p1 = round(p1(:)');
    p2 = round(p2(:)');
    [rows, cols] = size(traversable);

    ok = false;
    segment_cells = [];

    if any(p1 < 1) || p1(1) > rows || p1(2) > cols || ...
       any(p2 < 1) || p2(1) > rows || p2(2) > cols
        return;
    end
    if ~traversable(p1(1), p1(2)) || ~traversable(p2(1), p2(2))
        return;
    end

    cur = p1;
    segment_cells = cur;

    di_total = p2(1) - p1(1);
    dj_total = p2(2) - p1(2);
    diag_n = min(abs(di_total), abs(dj_total));
    straight_n = max(abs(di_total), abs(dj_total)) - diag_n;
    si = sign(di_total);
    sj = sign(dj_total);

    if strcmpi(order_mode, 'straight_first')
        if abs(di_total) >= abs(dj_total)
            straight_step = [si, 0];
        else
            straight_step = [0, sj];
        end
        step_plan = [repmat(straight_step, straight_n, 1); repmat([si, sj], diag_n, 1)];
    else
        if abs(di_total) >= abs(dj_total)
            straight_step = [si, 0];
        else
            straight_step = [0, sj];
        end
        step_plan = [repmat([si, sj], diag_n, 1); repmat(straight_step, straight_n, 1)];
    end

    for k = 1:size(step_plan, 1)
        step = step_plan(k, :);
        next = cur + step;

        if any(next < 1) || next(1) > rows || next(2) > cols || ~traversable(next(1), next(2))
            segment_cells = [];
            return;
        end

        if step(1) ~= 0 && step(2) ~= 0
            if ~traversable(cur(1) + step(1), cur(2)) || ~traversable(cur(1), cur(2) + step(2))
                segment_cells = [];
                return;
            end
        end

        cur = next;
        segment_cells = [segment_cells; cur]; %#ok<AGROW>
    end

    ok = isequal(cur, p2);
    if ~ok
        segment_cells = [];
    end
end
