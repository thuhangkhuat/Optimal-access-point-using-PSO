
function position=CreateRandomSolution(model)
    n = model.n;
    xmin= model.xmin;
    xmax = model.xmax;
    ymin = model.ymin;
    ymax = model.ymax;
    position = zeros(n,2);
    position(:,1) = xmin + (xmax - xmin) * rand(n, 1);
    position(:,2) = ymin + (ymax - ymin) * rand(n, 1);
end