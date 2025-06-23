function ok = is_line_free_binaryOcc(p1, p2, map)
    n = 40;
    ok = true;
    for i = linspace(0,1,n)
        pt = round(p1 + i*(p2 - p1));
        if any(pt < 1) || any(pt > map.GridSize) || checkOccupancy(map, grid2world(map, [pt(1) pt(2)]))
            ok = false;
            return;
        end
    end
end

