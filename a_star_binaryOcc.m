function path = a_star_binaryOcc(map, start, goal)
    sz = map.GridSize;
    open = [start, 0, heuristic(start, goal), 0];
    closed = [];
    came_from = zeros([sz, 2]);

    while ~isempty(open)
        [~, idx] = min(open(:,3)+open(:,4));
        current = open(idx,:);
        open(idx,:) = [];

        if all(current(1:2) == goal)
            break;
        end
        
        closed = [closed; current];
        for dy = -1:1
            for dx = -1:1
                if abs(dx)+abs(dy)==0 || abs(dx)+abs(dy)==2
                    continue;
                end
                neighbor = current(1:2) + [dy, dx];
                if any(neighbor < 1) || any(neighbor > sz) || checkOccupancy(map, grid2world(map, [neighbor(1) neighbor(2)]))
                    continue;
                end
                if any(ismember(closed(:,1:2), neighbor, 'rows'))
                    continue;
                end
                g = current(3) + 1;
                h = heuristic(neighbor, goal);
                f = g + h;
                idx_open = find(all(open(:,1:2) == neighbor, 2));
                if isempty(idx_open)
                    open = [open; neighbor, g, h, f];
                    came_from(neighbor(1), neighbor(2), :) = current(1:2);
                else
                    if open(idx_open,3) > g
                        open(idx_open,3:5) = [g, h, f];
                        came_from(neighbor(1), neighbor(2), :) = current(1:2);
                    end
                end
            end
        end
    end
    path = goal;
    pos = goal;
    % Reconstruct path
path = goal;
pos = goal;

while any(pos ~= start)
    prev = squeeze(came_from(pos(1), pos(2), :))';
    if all(prev == 0)
        error('Path reconstruction failed: reached an uninitialized node in came_from. Path might be incomplete or unreachable.');
    end
    path = [prev; path];
    pos = prev;
end

%     while any(pos ~= start)
%         pos = squeeze(came_from(pos(1), pos(2), :))';
%         path = [pos; path];
%     end
% end