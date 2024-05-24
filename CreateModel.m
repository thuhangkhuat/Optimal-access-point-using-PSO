

% Create the search map with initial belief
%

function model=CreateModel()
    
    %% Create a grid map
    MAP_SIZE = 150;
    range = 1;
    x = 0:range:MAP_SIZE; y = 0:range:MAP_SIZE; 
    [X,Y] = meshgrid(x(1:end-1),y(1:end-1)); 
    grid =  (length(x)-1)*(length(x)-1);
    X = reshape(X,[grid,1]);
    Y = reshape(Y,[grid,1]);
    X_graph = X + range/2;
    Y_graph = Y + range/2;

    % Map limits
    xmin= 0;
    xmax= floor(MAP_SIZE);
    
    ymin= 0;
    ymax= floor(MAP_SIZE);
    
    % Initial searching position
    xs=0;
    ys=0;
    
    % Number of path nodes (not including the start position (start node))
    n=30;
    % Distance min connect
    range_connect = 10;
  
    % Motion range
    MRANGE = MAP_SIZE/5;
    %% Incorporate the map and search parameters into a model
    model.xs=xs;
    model.ys=ys;
    model.n=n;
    model.xmin=xmin;
    model.xmax=xmax;
    model.ymin=ymin;
    model.ymax=ymax;
    model.MAPSIZE = MAP_SIZE;
    model.range = range;
    model.X = X;
    model.Y = Y;
    model.X_graph = X_graph;
    model.Y_graph = Y_graph;
    model.num_grid = grid;
    model.rconnect = range_connect;
    model.MRANGE = MRANGE;
    
end