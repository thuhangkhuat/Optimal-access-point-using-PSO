

function PlotSolution(tour,model)
    
    MAP_SIZE = model.MAPSIZE;
    num_grid = model.num_grid;
    tourX =  tour(:,1);
    tourY = tour(:,2);
    values = tourY(:);
%     plot(tourX,tourY,'b-*',...
%         'MarkerSize',3,...
%         'MarkerFaceColor','w',...
%         'LineWidth',1);
    scatter(tourX, tourY, 100, values, 'filled');
    grid on
    xlim([0,MAP_SIZE]);  ylim([0 MAP_SIZE]);
    xlabel('x (m)');
    ylabel('y (m)');
    
    % Set properties

end