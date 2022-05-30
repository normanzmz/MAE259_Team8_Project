% position_hist = timetable2table(position_hist);
% refer_Velo = timetable2table(refer_Velo);
% refer_Force = timetable2table(refer_Force);
% unit_vector = timetable2table(unit_vector);
u = position_hist(1,:);
pos_old = nodes_I(1,:);
vel_old = refer_Velo(1,:);
node_1_vol = zeros(length(position_hist),2);
node_1_pos_new= zeros(length(position_hist),2);
Additional_Force = zeros(length(position_hist),2);

for t = 1:end_time*(1/dt)
    t
    pos_desire = position_hist(t,:);
    time = t*dt;
    
    % Position Error
    Pos_error = pos_desire - pos_old;
    % Velocity error
    Vel_Error = refer_Velo(t,:) - vel_old;
    
    % PD Position Control Loop
    u= Pos_error*K_p + Vel_Error*K_d;
    [node_1_vol(t,:),node_1_pos_new(t,:),counter] = Robot_system(u,pos_desire,dt,pos_old,time);
    
    % Force
    Additional_Force(t,:) = sign(Vel_Error)*(Vel_Error^2)*1/2 * rho_blood * Area * Cd;
    Total_Force = Additional_Force(t,:) + 


    [pos_update,q, xi, yi] = DER(Total_Force,node1_pos_new,tol,M,EI,EA,dl,q_old,N,dt,lines)
end