function contact_force = BoundryJudge(old_posi)
    N = length(old_posi)/2;
    contact_force = zeros(2*N,1);
    K_f = 1;
    for i = 1:N
        if old_posi(2*i-1)<=0.25 && old_posi(2*i)>=0.25
            x_dist_wall = 0.25 - old_posi(2*i-1);
            y_dist_wall = 0.25 - old_posi(2*i);
        elseif old_posi(2*i-1)>=0.5 && old_posi(2*i) <=0.25
            x_dist_wall = 0.5 - old_posi(2*i-1);
            y_dist_wall = 0.25 - old_posi(2*i);
        elseif old_posi(2*i-1)<=0.75 && old_posi(2*i) >=0.5
            x_dist_wall = 0.75 - old_posi(2*i-1);
            y_dist_wall = 0.5 - old_posi(2*i);
        else
            x_dist_wall = 0;
            y_dist_wall = 0;
        end
        contact_force(2*i-1:2*i) = [K_f*y_dist_wall, K_f*x_dist_wall];
    end
%     disp(contact_force)
%     disp(size(contact_force))

%     rod_x = old_posi(1:2:2*N);
%     rod_y = old_posi(2:2:2*N);
%     lines_x = lines(:,1);
%     lines_y = lines(:,2);
%     
%     xi = 1;
%     yi = 1;
% %     [xi,yi] = polyxpoly(rod_x,rod_y,lines_x,lines_y);
% 
%         
% %     figure(5)
% %     plot(xi,yi,'o',lines_x,lines_y,'b')
% 
%     new_pose = 0;
%     contact_force = 0;
end