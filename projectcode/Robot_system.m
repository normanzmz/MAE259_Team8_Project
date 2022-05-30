function [vol,pos_new,counter] = Robot_system(u,pos_desire,dt,pos_old,time)
pos_new = pos_desire + u;

vol = (pos_new - pos_old)/dt;

counter = time*(1/dt) +1;
% counter = counter + 1;