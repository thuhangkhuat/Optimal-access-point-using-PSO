
function costP=MyCost(position,model)
   X_graph = model.X_graph;
   Y_graph = model.Y_graph;
   xmax = model.xmax;
   ymax = model.ymax;
   range_connect = model.rconnect;
   num_grid = model.num_grid;
   Distance = sqrt(xmax^2+ymax^2)*ones(length(X_graph),1);
   Distance_pos = sqrt((position(1,1)-X_graph).^2+(position(1,2)-Y_graph).^2);
   Distance_min = min(Distance,Distance_pos);
   for i = 2: length(position)
       Distance_pos = sqrt((position(i,1)-X_graph).^2+(position(i,2)-Y_graph).^2);
       Distance_min = min(Distance_min,Distance_pos);
   end
   compare = Distance_min <= range_connect;
   costP = num_grid - sum(compare,'all');
end