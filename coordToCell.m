function [i, j] = coordToCell(x, y, map)
    % Parametri mappa
    [~, mapSize] = size(map);        % numero di celle per lato
    mapMeters = 10;       % lato della mappa in metri
    resolution = mapMeters / mapSize;  % metri per cella

    % Offset per posizionare l'origine (0,0) al centro della mappa
    originX = -mapMeters / 2;  % es. -5
    originY = mapMeters / 2;   % es. +5 (top -> bottom)

    % Conversione coordinate reali -> indici matrice
    j = round((x - originX) / resolution);      % colonna (x -> da sx a dx)
    i = round((originY - y) / resolution);      % riga    (y -> da su a giù)

    % Clipping per evitare out-of-bounds (opzionale)
    j = max(1, min(mapSize, j));
    i = max(1, min(mapSize, i));
end
