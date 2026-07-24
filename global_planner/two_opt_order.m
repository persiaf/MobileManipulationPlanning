function order = two_opt_order(distance_matrix, initial_order)
    n = length(initial_order);
    if n <= 3
        order = initial_order;
        return;
    end

    order = initial_order;
    improved = true;

    while improved
        improved = false;
        best_improvement = 0;
        best_i = 0;
        best_k = 0;

        for i = 1:n-2
            for k = i+2:n
                if i == 1 && k == n
                    continue;
                end

                a = order(i);
                b = order(i+1);
                c = order(k);
                if k < n
                    d = order(k+1);
                else
                    d = order(1);
                end

                old_cost = distance_matrix(a, b) + distance_matrix(c, d);
                new_cost = distance_matrix(a, c) + distance_matrix(b, d);
                improvement = old_cost - new_cost;

                if improvement > best_improvement
                    best_improvement = improvement;
                    best_i = i;
                    best_k = k;
                end
            end
        end

        if best_improvement > 0
            order(best_i+1:best_k) = order(best_k:-1:best_i+1);
            improved = true;
        end
    end
end