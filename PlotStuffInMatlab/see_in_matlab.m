format long

% read text file: change name accordingly
M = dlmread('_data_0.txt');

time       = M(:,1);
state      = M(:,2:13);
stated     = M(:,14:31);
quaternion = M(:,32:35);
omega_quad = M(:,36:38);
input_quad = M(:,39:41);

%% useful operation
% restoredefaultpath; matlabrc
% close all
% clear all
clc

%% set some definitions
set(0,'DefaultTextInterpreter','latex',...
      'DefaultLineLineWidth',1,...
      'defaulttextfontsize',12,...
      'defaultaxesfontsize',12);

%% make video with Video Writer

%clc
close all
figure

video = VideoWriter('Trajectory.avi');
video.FrameRate = 1;
open(video);

% put initial time to 0
time = time - time(1);
% final time
tF = time(end);

% time window for the plotting
t0_desired    = 0;
tF_desired    = 20;
time_interval = tF_desired - t0_desired;

times_to_plot = time >= t0_desired & time <= tF_desired;
times_vector  = time(times_to_plot);


% position and velocity of the quadrotor 
pQ  = state(times_to_plot,7:9);
vQ  = state(times_to_plot,10:12);

% position and velocity of the load
pM  = state(times_to_plot,1:3);
vM  = state(times_to_plot,4:6);


% desired position of the load
pMd = stated(times_to_plot,1:3);

% number of times we want to plot quadrotor vehicle
num = 20;
marker = zeros(num+1,1);
for i = 0:1:num
    [minmin, index] = min(abs(times_vector - time_interval*i/num));
    marker(i+1)     = index;
end


for j = 1:num + 1
    
    hold on
    % plot load trajectory
    plot3(pM(:,1) ,pM(:,2) ,pM(:,3) ,'Color',[0.7 0.7 0.7],'Linewidth',2)
    % plot load desired trajectory
    plot3(pMd(:,1),pMd(:,2),pMd(:,3),'Color',[0.3 0.3 0.3],'Linewidth',1)    
    
    % position of quadrotor
    pp   = pQ(marker(j),:)';
    
    % position of load
    pp2  = pM(marker(j),:)';

    % cable unit vector 
    nn   = (pp - pp2)/norm(pp - pp2);
    
    % rotation matrix associated to cable
    RR   = [null(nn') nn];
    
    % plot quadrotor drawing
    %DISC_plot(pp,RR,1,[0.8 0.8 0.8],0.5,0.1)
    %plot_vect(pp,pp + RR*[0;0;1]*0.1,[0.8 0.8 0.8])  
    quad_plot(pp,RR,1,[0.5 0.5 0.5],0.1,0.1)
     
    % draw cable between load and quadrotor
    Connect(pp',pp2')  

    % load desired position 
    pp2 = pMd(marker(j),:)';
    % plot load desired position
    plot3(pp2(1),pp2(2),pp2(3),'o','Color','b')
   
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
    
    xlim([-1.5,1.5])
    ylim([-1.5,1.5])
    zlim([0,1.2])
    % axis equal
    % axis tight    
    
    view(56,16)

    % Store the frame
    writeVideo(video,getframe(gca, [0 0 435 344]));
    
    close all

end

close(video);  
  
  
%% make video
%clc
close all
figure

% put initial time to 0
time = time - time(1);
% final time
tF = time(end);

% time window for the plotting
t0_desired    = 0;
tF_desired    = 20;
time_interval = tF_desired - t0_desired;

times_to_plot = time >= t0_desired & time <= tF_desired;
times_vector  = time(times_to_plot);


% position and velocity of the quadrotor 
pQ  = state(times_to_plot,7:9);
vQ  = state(times_to_plot,10:12);

% position and velocity of the load
pM  = state(times_to_plot,1:3);
vM  = state(times_to_plot,4:6);


% desired position of the load
pMd = stated(times_to_plot,1:3);


% number of times we want to plot quadrotor vehicle
num = 20;
marker = zeros(num+1,1);
for i = 0:1:num
    [minmin, index] = min(abs(times_vector - time_interval*i/num));
    marker(i+1)     = index;
end


for j = 1:num + 1
    
    hold on
    % plot load trajectory
    plot3(pM(:,1) ,pM(:,2) ,pM(:,3) ,'Color',[0.7 0.7 0.7],'Linewidth',2)
    % plot load desired trajectory
    plot3(pMd(:,1),pMd(:,2),pMd(:,3),'Color',[0.3 0.3 0.3],'Linewidth',1)    
    
    % position of quadrotor
    pp   = pQ(marker(j),:)';
    
    % position of load
    pp2  = pM(marker(j),:)';

    % cable unit vector 
    nn   = (pp - pp2)/norm(pp - pp2);
    
    % rotation matrix associated to cable
    RR   = [null(nn') nn];
    
    % plot quadrotor drawing
    %DISC_plot(pp,RR,1,[0.8 0.8 0.8],0.5,0.1)
    %plot_vect(pp,pp + RR*[0;0;1]*0.1,[0.8 0.8 0.8])  
    quad_plot(pp,RR,1,[0.5 0.5 0.5],0.1,0.1)
     
    % draw cable between load and quadrotor
    Connect(pp',pp2')  

    % load desired position 
    pp2 = pMd(marker(j),:)';
    % plot load desired position
    plot3(pp2(1),pp2(2),pp2(3),'o','Color','b')
   
    xlabel('x (m)')
    ylabel('y (m)')
    zlabel('z (m)')
    
    xlim([-1.5,1.5])
    ylim([-1.5,1.5])
    zlim([0,1.2])
    % axis equal
    % axis tight    
    
    view(56,16)

    % Store the frame
    % M_frame(j) = getframe(ax); % leaving 
    M_frame(j) = getframe(gca, [0 0 435 344]);
    
    close all

end

% movie(M_frame,1,1)
movie2avi(M_frame,'Trajectory.avi','fps',1);  
% VideoWriter(....,...);  

  
%%
clc
close all
figure

% put initial time to 0
time = time - time(1);
% final time
tF = time(end);

% time window for the plotting
t0_desired    = 0;
tF_desired    = 5;
time_interval = tF_desired - t0_desired;

times_to_plot = time >= t0_desired & time <= tF_desired;
times_vector  = time(times_to_plot);


t = time - time(1);
tEND = t(end);


% position and velocity of the quadrotor 
pQ  = state(times_to_plot,7:9);
vQ  = state(times_to_plot,10:12);

% position and velocity of the load
pM  = state(times_to_plot,1:3);
vM  = state(times_to_plot,4:6);


% desired position of the load
pMd = stated(times_to_plot,1:3);


hold on
% plot load trajectory
plot3(pM(:,1) ,pM(:,2) ,pM(:,3) ,'Color','k','Linewidth',2)
% plot load desired trajectory
plot3(pMd(:,1),pMd(:,2),pMd(:,3),'Color',[0.3 0.3 0.3],'Linewidth',1)

% number of times we want to plot quadrotor vehicle
num = 4;
marker = zeros(num+1,1);
for i = 0:1:num
    [minmin, index] = min(abs(times_vector - time_interval*i/num));
    marker(i+1)     = index;
end


for j = 1:num+1
    
    % position of quadrotor
    pp   = pQ(marker(j),:)';
    
    % position of load
    pp2  = pM(marker(j),:)';

    % cable unit vector 
    nn   = (pp - pp2)/norm(pp - pp2);
    
    % rotation matrix associated to cable
    RR   = [null(nn') nn];
    
    % plot quadrotor drawing
    %DISC_plot(pp,RR,1,[0.8 0.8 0.8],0.5,0.1)
    %plot_vect(pp,pp + RR*[0;0;1]*0.1,[0.8 0.8 0.8])  
    quad_plot(pp,RR,1,[0.5 0.5 0.5],0.1,0.1)
    
    % draw cable between load and quadrotor
    Connect(pp',pp2')   

    % load desired position 
    pp2 = pMd(marker(j),:)';
    % plot load desired position
    plot3(pp2(1),pp2(2),pp2(3),'o','Color','b')

end

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')

% axis equal
axis tight

view(56,16)

%%
close all

t = time - time(1);
tEND = t(end);

p    = state(:,1:3);
pdes = stated(:,1:3);

auxaux = 1:length(t);
t_aux    = ~mod(auxaux,1);

hold on

plot(t(t_aux),p(t_aux,1)-pdes(t_aux,1),'-','Color','k')
plot(t(t_aux),p(t_aux,2)-pdes(t_aux,2),'--','Color','k')
plot(t(t_aux),p(t_aux,3)-pdes(t_aux,3),'-.','Color','k')
plot(t(t_aux),sqrt((p(t_aux,1)-pdes(t_aux,1)).^2 + (p(t_aux,2)-pdes(t_aux,2)).^2 + (p(t_aux,3)-pdes(t_aux,3)).^2),'-.','Color','k')

xlabel('Time (s)')
ylabel('(m)')

l = legend('$\mathbf{p}_1$',...
           '$\mathbf{p}_2$',...
           '$\mathbf{p}_3$',...
           '$error$');
set(l,'Interpreter','latex')

grid on

%%
close all

t = time - time(1);
tEND = t(end);

v    = state(:,4:6);
vdes = stated(:,4:6);

auxaux = 1:length(t);
t_aux    = ~mod(auxaux,1);

hold on

plot(t(t_aux),v(t_aux,1)-vdes(t_aux,1),'-','Color','k')
plot(t(t_aux),v(t_aux,2)-vdes(t_aux,2),'--','Color','k')
plot(t(t_aux),v(t_aux,3)-vdes(t_aux,3),'-.','Color','k')

xlabel('Time (s)')
ylabel('(m/s)')

l = legend('$\mathbf{v}_1$',...
           '$\mathbf{v}_2$',...
           '$\mathbf{v}_3$');
set(l,'Interpreter','latex')

grid on


%%
close all

t = time - time(1);
tEND = t(end);

auxaux = 1:length(t);
t_aux    = ~mod(auxaux,1);

g_quad = 9.81;

nn = [];
for i= 1:length(stated(:,7:9))
    pp   = pQ(i,:)';
    pp2  = pM(i,:)';

    n   = (pp - pp2)/norm(pp - pp2);

    nn = [nn; n'];  
end

n_des = [];
for i= 1:length(stated(:,7:9))
    ades   = stated(i,7:9);
    n = (g_quad*[0;0;1] + ades')/norm(g_quad*[0;0;1] + ades');  

    n_des = [n_des; n'];  
end

hold on

plot(t(t_aux),nn(t_aux,1),'-','Color','k')
plot(t(t_aux),n_des(t_aux,1),'-','Color',[0.5 0.5 0.5])

plot(t(t_aux),nn(t_aux,2),'--','Color','k')
plot(t(t_aux),n_des(t_aux,2),'--','Color',[0.5 0.5 0.5])

plot(t(t_aux),nn(t_aux,3),'-.','Color','k')
plot(t(t_aux),n_des(t_aux,3),'-.','Color',[0.5 0.5 0.5])

xlabel('Time (s)')

l = legend('$\mathbf{n}_1$',...
           '$\mathbf{n}_{1,t}^{\star}$',...
           '$\mathbf{n}_2$',...
           '$\mathbf{n}_{2,t}^{\star}$',...
           '$\mathbf{n}_3$',...
           '$\mathbf{n}_{3,t}^{\star}$');
set(l,'Interpreter','latex')

grid on


%%

close all

t = time - time(1);
tEND = t(end);

w = [];
for i= 1:length(stated(:,7:9))
    pp   = pQ(i,:)';
    pp2  = pM(i,:)';
    
    vv   = vQ(i,:)';
    vv2  = vM(i,:)';
        
    n   = (pp - pp2)/norm(pp - pp2);

    ww   = skew(n)*(vv - vv2)/norm(pp - pp2);
    
    w = [w; ww'];  
end

w_des = [];
for i= 1:length(stated(:,7:9))
    ades   = stated(i,7:9)';
    n_des  = (g_quad*[0;0;1] + ades)/norm(g_quad*[0;0;1] + ades);  

    jdes = stated(i,10:12)';
    w_eq = skew(n_des)*jdes/norm(g_quad*[0;0;1] - ades);    
    
    w_des = [w_des; w_eq'];  
end

auxaux = 1:length(t);
t_aux    = ~mod(auxaux,1);

hold on

% plot(t(t_aux),w(t_aux,1),'-','Color','k')
% plot(t(t_aux),w_des(t_aux,1),'-','Color',[0.5 0.5 0.5])

plot(t(t_aux),w(t_aux,2),'--','Color','k')
plot(t(t_aux),w_des(t_aux,2),'--','Color',[0.5 0.5 0.5])
 
% plot(t(t_aux),w(t_aux,3),'-.','Color','k')
% plot(t(t_aux),w_des(t_aux,3),'-.','Color',[0.5 0.5 0.5])

xlabel('Time (s)')
ylabel('(rad/s)')

l = legend('$\omega_1$',...
           '$\omega_{1,t}^{\star}$',...
           '$\omega_2$',...
           '$\omega_{2,t}^{\star}$',...
           '$\omega_3$',...
           '$\omega_{3,t}^{\star}$');
       
set(l,'Interpreter','latex')

grid on


%%
close all

t = time - time(1);
tEND = t(end);

u  = input_quad;

auxaux = 1:length(t);
t_aux    = ~mod(auxaux,1);

t_aux = t <= tEND + 0.1;

hold on

plot(t(t_aux),0.1*u(t_aux,1),'-','Color','k')
plot(t(t_aux),u(t_aux,2),'--','Color','k')
plot(t(t_aux),u(t_aux,3),'-.','Color','k')

xlabel('Time (s)')
% ylabel('Input')

xlim([0 tFINAL])

l = legend('$U_1$',...
           '$U_2$',...
           '$U_3$');
       
set(l,'Interpreter','latex')

grid on
