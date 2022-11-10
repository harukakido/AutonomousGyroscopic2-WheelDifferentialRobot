%% url https://www.youtube.com/watch?v=bj4l1EuF9Nk&list=PLyqSpQzTE6M-YPOEUVrgd_2UnIRvWUcRr&index=35
clear; close; clc
dt =0.1; ts = 255; t = 0:dt:ts;

K_u =2; K_r =4;
a = 0.05; d=0.2; l =0.3;


%predifine array to improve performance
n=length(t);
zeta = zeros(3,n); 
eta_dot= zeros(3,n);
eta= zeros(3,n);

%% 0 initial waypoint
for i = 1:25
eta(:,1) = [0;0;0];
eta_d(:,1) = [1;1; pi/2];
rho = sqrt((eta_d(1)-eta(1,i))^2+(eta_d(2)-eta(2,i))^2);
e=[eta_d(1:2);atan2(eta_d(2)-eta(2,i),eta_d(1)-eta(1,i))] - eta(:,i);

if rho <= 0.01
 e = eta_d - eta(:,i);
 %%e = 0*e;
end

psi = eta(3,i);
J = [cos(psi), -sin(psi), 0;
    sin(psi), cos(psi) 0;
     0,0,1];
zeta(:,i) = inv(J)*(diag([2,2,4])*e);
u=zeta(1,i); r = zeta(3,i);
eta(:,i+1) = eta(:,i) + (1-exp(-1*t(i)))*[cos(eta(3,i)),0;sin(eta(3,i)),0;0,1] *[u;r] * dt;
end

%% 1 New Position continue left turn
for i = 25:50
eta_d(:,1) = [0;2;-pi];
rho = sqrt((eta_d(1)-eta(1,i))^2+(eta_d(2)-eta(2,i))^2);
e=[eta_d(1:2);atan2(eta_d(2)-eta(2,i),eta_d(1)-eta(1,i))] - eta(:,i);

if rho <= 0.05
 e = eta_d - eta(:,i);
 %e = 0*e;
end

psi = eta(3,i);
J = [cos(psi), -sin(psi), 0;
    sin(psi), cos(psi) 0;
     0,0,1];
zeta(:,i) = inv(J)*(diag([2,2,4])*e);
u=zeta(1,i); r = zeta(3,i);
eta(:,i+1) = eta(:,i) + (1-exp(-1*t(i)))*[cos(eta(3,i)),0;sin(eta(3,i)),0;0,1] *[u;r] * dt;
end

%%%%%%%%%%%%%
%% 2 New Position continue to middle
for i = 50:75
eta_d(:,1) = [-1;1; -pi/2];
rho = sqrt((eta_d(1)-eta(1,i))^2+(eta_d(2)-eta(2,i))^2);
e=[eta_d(1:2);atan2(eta_d(2)-eta(2,i),eta_d(1)-eta(1,i))] - eta(:,i);

if rho <= 0.05
 e = eta_d - eta(:,i);
 %%e = 0*e;
end

psi = eta(3,i);
J = [cos(psi), -sin(psi), 0;
    sin(psi), cos(psi) 0;
     0,0,1];
zeta(:,i) = inv(J)*(diag([2,2,4])*e);
u=zeta(1,i); r = zeta(3,i);
eta(:,i+1) = eta(:,i) + (1-exp(-1*t(i)))*[cos(eta(3,i)),0;sin(eta(3,i)),0;0,1] *[u;r] * dt;
end

%%%%%%%%%%%
%% 3 New Position continue to next circle
for i = 75:100
eta_d(:,1) = [-2;0; pi];
rho = sqrt((eta_d(1)-eta(1,i))^2+(eta_d(2)-eta(2,i))^2);
e=[eta_d(1:2);atan2(eta_d(2)-eta(2,i),eta_d(1)-eta(1,i))] - eta(:,i);

if rho <= 0.05
 e = eta_d - eta(:,i);
 %%e = 0*e;
end

psi = eta(3,i);
J = [cos(psi), -sin(psi), 0;
    sin(psi), cos(psi) 0;
     0,0,1];
zeta(:,i) = inv(J)*(diag([2,2,4])*e);
u=zeta(1,i); r = zeta(3,i);
eta(:,i+1) = eta(:,i) + (1-exp(-1*t(i)))*[cos(eta(3,i)),0;sin(eta(3,i)),0;0,1] *[u;r] * dt;
end

%%%%%%%%%%%
%% 4 New Position continue to next circle
for i = 100:125
eta_d(:,1) = [-3;1; pi/2];
rho = sqrt((eta_d(1)-eta(1,i))^2+(eta_d(2)-eta(2,i))^2);
e=[eta_d(1:2);atan2(eta_d(2)-eta(2,i),eta_d(1)-eta(1,i))] - eta(:,i);

if rho <= 0.05
 e = eta_d - eta(:,i);
 %%e = 0*e;
end

psi = eta(3,i);
J = [cos(psi), -sin(psi), 0;
    sin(psi), cos(psi) 0;
     0,0,1];
zeta(:,i) = inv(J)*(diag([2,2,4])*e);
u=zeta(1,i); r = zeta(3,i);
eta(:,i+1) = eta(:,i) + (1-exp(-1*t(i)))*[cos(eta(3,i)),0;sin(eta(3,i)),0;0,1] *[u;r] * dt;
end

%%%%%%%%%%%
%% 5 New Position continue to next middle
for i = 125:150
eta_d(:,1) = [-2; 2; 0];
rho = sqrt((eta_d(1)-eta(1,i))^2+(eta_d(2)-eta(2,i))^2);
e=[eta_d(1:2);atan2(eta_d(2)-eta(2,i),eta_d(1)-eta(1,i))] - eta(:,i);

if rho <= 0.05
 e = eta_d - eta(:,i);
 %%e = 0*e;
end

psi = eta(3,i);
J = [cos(psi), -sin(psi), 0;
    sin(psi), cos(psi) 0;
     0,0,1];
zeta(:,i) = inv(J)*(diag([2,2,4])*e);
u=zeta(1,i); r = zeta(3,i);
eta(:,i+1) = eta(:,i) + (1-exp(-1*t(i)))*[cos(eta(3,i)),0;sin(eta(3,i)),0;0,1] *[u;r] * dt;
end

%% 6 New Position continue to next middle
for i = 150:175
eta_d(:,1) = [-2; 2; 0];
rho = sqrt((eta_d(1)-eta(1,i))^2+(eta_d(2)-eta(2,i))^2);
e=[eta_d(1:2);atan2(eta_d(2)-eta(2,i),eta_d(1)-eta(1,i))] - eta(:,i);

if rho <= 0.05
 e = eta_d - eta(:,i);
 %%e = 0*e;
end

psi = eta(3,i);
J = [cos(psi), -sin(psi), 0;
    sin(psi), cos(psi) 0;
     0,0,1];
zeta(:,i) = inv(J)*(diag([2,2,4])*e);
u=zeta(1,i); r = zeta(3,i);
eta(:,i+1) = eta(:,i) + (1-exp(-1*t(i)))*[cos(eta(3,i)),0;sin(eta(3,i)),0;0,1] *[u;r] * dt;
end

%% 7 New Position continue to next middle
for i = 175:200
eta_d(:,1) = [-1;1;-pi/2];
rho = sqrt((eta_d(1)-eta(1,i))^2+(eta_d(2)-eta(2,i))^2);
e=[eta_d(1:2);atan2(eta_d(2)-eta(2,i),eta_d(1)-eta(1,i))] - eta(:,i);

if rho <= 0.05
 e = eta_d - eta(:,i);
 %%e = 0*e;
end

psi = eta(3,i);
J = [cos(psi), -sin(psi), 0;
    sin(psi), cos(psi) 0;
     0,0,1];
zeta(:,i) = inv(J)*(diag([2,2,4])*e);
u=zeta(1,i); r = zeta(3,i);
eta(:,i+1) = eta(:,i) + (1-exp(-1*t(i)))*[cos(eta(3,i)),0;sin(eta(3,i)),0;0,1] *[u;r] * dt;
end


%% 8 New Position continue to next middle
for i = 200:225
eta_d(:,1) = [0;0; -0];
rho = sqrt((eta_d(1)-eta(1,i))^2+(eta_d(2)-eta(2,i))^2);
e=[eta_d(1:2);atan2(eta_d(2)-eta(2,i),eta_d(1)-eta(1,i))] - eta(:,i);

if rho <= 0.05
 e = eta_d - eta(:,i);
 %%e = 0*e;
end

psi = eta(3,i);
J = [cos(psi), -sin(psi), 0;
    sin(psi), cos(psi) 0;
     0,0,1];
zeta(:,i) = inv(J)*(diag([2,2,4])*e);
u=zeta(1,i); r = zeta(3,i);
eta(:,i+1) = eta(:,i) + (1-exp(-1*t(i)))*[cos(eta(3,i)),0;sin(eta(3,i)),0;0,1] *[u;r] * dt;
end



veh_box = 0.5*[-0.4 0.6 0.6+0.3*cosd(-90:90) 0.6 -0.4 -0.4; -0.3 -0.3 0.3*sind(-90:90) 0.3 0.3 0.3];
cas_p = 0.5*[0.6;0];
wheel_b = 0.5*[-0.2 0.2 0.2 -0.2 -0.2; -0.05 -0.05 0.05 0.05 -0.05];

 for i = 1:225
psi = eta(3,i); x(i) = eta (1,i); y(i)  = eta (2,i);
R = [cos(psi), -sin(psi);
    sin(psi), cos(psi)];
v_m = R*veh_box;
c_m = R*cas_p;
w_m1 = R*(wheel_b+[0;0.35/2]);
w_m2 = R*(wheel_b+[0;-0.35/2]);
fill(v_m(1,:)+x(i),v_m(2,:)+y(i),'y');
hold on 
fill(w_m1(1,:)+x(i),w_m1(2,:)+y(i),'r');
fill(w_m2(1,:)+x(i),w_m2(2,:)+y(i),'r');
fill(c_m(1)+0.05*cosd(0:360)+x(i),c_m(2)+0.05*sind(0:360)+y(i),'g');
plot(eta(1,1:i),eta(2,1:i),'b-')
plot(eta_d(1),eta_d(2),'k*')
plot([eta_d(1),eta_d(1)+0.2*cos(eta_d(3))],[eta_d(2),eta_d(2)+0.2*sin(eta_d(2))]); 
plot(eta_d(1)+0.1*cosd(0:360),eta_d(2)+0.1*sind(0:360),'c--')
xmin = min (eta(1,:)) - 2; 
xmax = min (eta(1,:)) + 5; 
ymin = min (eta(2,:)) - 3; 
ymax = min (eta(2,:)) + 3; 
axis ([xmin xmax ymin ymax])
axis equal
grid on
xlabel ('x,[m]')
ylabel ('y,[m]')
pause (0.1)
hold off
 end
