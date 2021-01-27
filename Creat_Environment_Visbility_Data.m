epsilon = 0.000000001;


%Snap distance (distance within which an observer location will be snapped to the
%boundary before the visibility polygon is computed)
snap_distance = 0.05;

ENV_SIZE1 = 50;  % will be ENV_SIZE x ENV_SIZE grid
ENV_SIZE2 = 25;

%Read environment geometry from file
% environment = read_vertices_from_file('./Environments/M_starstar12.environment');
environment_min_x = min(environment{1}(:,1));
environment_max_x = max(environment{1}(:,1));
environment_min_y = min(environment{1}(:,2));
environment_max_y = max(environment{1}(:,2));
X_MIN = environment_min_x-0.1*(environment_max_x-environment_min_x);
X_MAX = environment_max_x+0.1*(environment_max_x-environment_min_x);
Y_MIN = environment_min_y-0.1*(environment_max_y-environment_min_y);
Y_MAX = environment_max_y+0.1*(environment_max_y-environment_min_y);

for x = floor(X_MIN):floor(X_MAX)+1
    for y = floor(Y_MIN):floor(Y_MAX)+1
        if in_environment( [x,y] , environment , epsilon )
            Visibility_Data{100*y + x} = visibility_polygon( [x y] , environment , epsilon, snap_distance); 
            Region{100*y + x} =  poly2mask(Visibility_Data{100*y + x}(:,1),Visibility_Data{100*y + x}(:,2),ENV_SIZE1, ENV_SIZE2);
        else
            Visibility_Data{100*y + x} = -1; 
        end
    end
end

for i = 1:length(Visibility_Data)
    Visibility_Data{i} = -1;
end

for i = 1:length(Region)
    Region{i} = -1;
end

for x = floor(X_MIN):floor(X_MAX)+1
    for y = floor(Y_MIN):floor(Y_MAX)+1
        if in_environment( [x,y] , environment , epsilon )
            Visibility_Data{100*y + x} = visibility_polygon( [x y] , environment , epsilon, snap_distance); 
            Region{100*y + x} =  poly2mask(Visibility_Data{100*y + x}(:,1),Visibility_Data{100*y + x}(:,2),ENV_SIZE1, ENV_SIZE2);
        else
            Visibility_Data{100*y + x} = -1; 
        end
    end
end



for i = 1:length(Asset)
    w{i} =  visibility_polygon( Asset(i,:) , environment , epsilon, snap_distance);
    for x = floor(X_MIN):floor(X_MAX)+1
        for y = floor(Y_MIN):floor(Y_MAX)+1
            if in_environment( [x,y] , w , epsilon )
                Asset_Visibility_Data(i,100*y + x) = 1;
            else
                Asset_Visibility_Data(i,100*y + x) = -1;
            end
        end
    end
end

Visibility_in_environment = zeros(floor(X_MAX)+1 + 100* (floor(Y_MAX)+1), floor(X_MAX)+1 + 100* (floor(Y_MAX)+1));

for x_location = floor(X_MIN):floor(X_MAX)+1
    for y_location = floor(Y_MIN):floor(Y_MAX)+1
        if in_environment( [x_location,y_location] , environment , epsilon )
            V{1} =  visibility_polygon( [x_location,y_location] , environment , epsilon, snap_distance);
%             display([x_location,y_location]);
            for x_tosee = floor(X_MIN):floor(X_MAX)+1
                for y_tosee = floor(Y_MIN):floor(Y_MAX)+1             
                    if in_environment( [x_tosee,y_tosee] , V , epsilon )
                        x = x_location + 100*y_location;
                        y = x_tosee + 100* y_tosee;
                        Visibility_in_environment(x,y) = 1;
                    end             
                end
            end
        end
        
    end
end


save('Save_Visibility_Data\M_starstar12.mat')