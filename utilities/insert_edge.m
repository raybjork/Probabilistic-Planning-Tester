% insert an edge into an adjacency matrix
function [A, plot] = insert_edge(A, configurations, old, new, plot)       
    plot = plot_edges(plot, configurations(old, :), configurations(new, :), new);
    dist = norm(configurations(old, :) - configurations(new, :));
    A(new, old) = dist;
    A(old, new) = dist;
    A(new, new) = A(old, old) + dist;
end