function [pos_update,q, xi, yi] = DER(Total_Force,node1_pos_new,tol,M,EI,EA,dl,q_old,N,dt,lines)

Ex_force = zeros(2*N,1);

    q = q_old; % Guess
    q(1:2) = node1_pos_new';
    u = (q - q_old) / dt;

    err = 10000 * tol;
    % Newton Raphson
    while err > tol

    % Inertia 
    f = M/dt * ((q - q_old) / dt - u);
    J = M/ dt^2;

    % Elastic forces 
    % Linear Spring 
    for k = 1: N-1
        xk = q(2*k-1);
        yk = q(2*k);
        xkp1 = q(2*k+1);
        ykp1 = q(2*k+2);
        dF = gradEs(xk, yk, xkp1, ykp1, dl, EA);
        dJ = hessEs(xk, yk, xkp1, ykp1, dl, EA);

        f(2*k-1:2*k+2) = f(2*k-1:2*k+2) + dF;
        J(2*k-1:2*k+2, 2*k-1:2*k+2) = ...
        J(2*k-1:2*k+2, 2*k-1:2*k+2) + dJ;
    end

    % Bending spring 
    for k = 2:N-1
        xkm1 = q(2*k-3);
        ykm1 = q(2*k-2);
        xk = q(2*k-1);
        yk = q(2*k);
        xkp1 = q(2*k+1);
        ykp1 = q(2*k+2);
        curvature0 = 0;
        dF = gradEb(xkm1, ykm1, xk, yk, xkp1, ykp1, curvature0, dl, EI);
        dJ = hessEb(xkm1, ykm1, xk, yk, xkp1, ykp1, curvature0, dl, EI);
        f(2*k-3:2*k+2) = f(2*k-3:2*k+2) + dF;
        J(2*k-3:2*k+2, 2*k-3:2*k+2) = ...
            J(2*k-3:2*k+2, 2*k-3:2*k+2) + dJ;
    end
    
    % Contact Force
    rod_x = q_old(1:2:2*N);
    rod_y = q_old(2:2:2*N);
    lines_x = lines(:,1);
    lines_y = lines(:,2);
    [xi,yi] = polyxpoly(rod_x,rod_y,lines_x,lines_y);

    % External force 
    Ex_force(1) = Total_Force(1,:);
    Ex_force(2) = Total_Force(2,:);

    % Weight 
    f = f + Ex_force;

    % Update 
    q(1:(2*N)) = q(1:(2*N)) - J(1:(2*N),1:(2*N)) \ f(1:(2*N));
    err = sum(abs(f(1:(2*N))));
    end


    pos_update = q(1:2,:);
    




