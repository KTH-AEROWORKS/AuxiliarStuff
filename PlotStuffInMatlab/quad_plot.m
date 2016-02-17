function quad_plot(position,rot,auto,maincolor,alpha,esc)
% QUAD_PLOT     Plots a quadrotor from position and rotation matrix orientation
%
%   quad_plot(position,rot,auto,esc,maincolor,alpha)
%
%   general function to plot a quadrotor with given "position" vector and
%   direct Cosine "rotation" matrix. The function is made such that when saving
%   the Matlab image in eps, the result is a truly vectorial image (since
%   some drawing functions make this impossible by saving the eps images
%   as bitmaps).
%
%   position    --> position vector [x;y;z].
%   rot          --> Rotation Matrix
%   auto        --> plots heli in the active figure or in an independent one:
%                   0 -> assuming that "hold" is on;
%                   1 -> erases active figure and plots the helicopter.
%   maincolor       --> specify RGB color of the helicopter (the default is
%                   yellow, when "color = []").
%   alpha       --> Transparency (0 - transparent, 1 - opaque)
%                    (the default is opaque, when "alpha = []").
%   esc         --> Scaling (1 is approx. 1 meter radius)
%
%   examples:
%
%       1.  quad_plot([1;2;3],[0;pi/6;pi/12],1,[],[]);
%
%       2.  figure(10);
%           heli_plot([1;2;3],[0;pi/6;pi/12],1,[]);
%           hold on;
%           heli_plot([10;2;3],[0;pi/6;pi/12+pi],0,[0.7,0,0]);
%           heli_plot([5;7;3],[0;pi/6;pi/12-pi/2],0,[0,0.7,0]);
%           grid on;
%           axis equal;
%           hold off;
%           view(30,30);
%
%   See also patch, colormap, surf.

if isempty(position)
    position = [0 0 0]';
end

position = position';

if isempty(rot)
    rot = eye(3);
end

if isempty(auto)
    auto = 1;
end

if isempty(maincolor)
    maincolor = [0.95 0.95 0.0];
end

if isempty(alpha)
    alpha = 1;
end

if isempty(esc)
    esc = 1;
end


% Define colors
LW = 0.05;
mainbarcolor = [1 0 0];
barcolor = [0.1 0.1 0.1];
propcolor = [.9 .9 .9];
edgecolor = [0 0 0];

% Define dimensions
rc = 0.4;
rp = .2;
l = .9;
h = 0.4;
w = 0.05;

if auto
    hold on;
end

% Central body
[x y z] = cylinder;
xbodytmp = rc*x;
ybodytmp = rc*y;
zbodytmp = h/1.5*z-h/3;

xbody = zeros(size(xbodytmp));
ybody = xbody;
zbody = xbody;

% propeller at x positive
xproptmp = rp*x + l;
yproptmp = rp*y;
zproptmp = w*z/10+h/2-w/2;

xprop = zeros(size(xproptmp));
yprop = xprop;
zprop = xprop;

for i=1:2
    body = (rot*([xbodytmp(i,:)' ybodytmp(i,:)' zbodytmp(i,:)']*esc)')' ...
        + kron(ones(length(xbodytmp),1),position);
    
    xbody(i,:) = body(:,1);
    ybody(i,:) = body(:,2);
    zbody(i,:) = body(:,3);
    
        prop = (rot*([xproptmp(i,:)' yproptmp(i,:)' zproptmp(i,:)']*esc)')' ...
            + kron(ones(length(xproptmp),1),position);
    
        xprop(i,:) = prop(:,1);
        yprop(i,:) = prop(:,2);
        zprop(i,:) = prop(:,3);
end

%%% Bars - Just x positive

% main bar
vert_bar_xpos = [2*rp w/2 w/2 %1
    2*rp w/2 -w/2
    2*rp -w/2 w/2 % 3
    2*rp -w/2 -w/2
    l w/2 w/2 % 5
    l w/2 -w/2
    l -w/2 w/2 % 7
    l -w/2 -w/2];

face_bar_xpos = [  1  2  4 3
    5  6  8  7
    1  2  6  5
    1  3  7  5
    4  2  6  8
    4  3  7  8 ];

% propeller bar

vert_propbar_xpos = [l-w/2 w/2 h/2
    l-w/2 w/2 -w/2
    l-w/2 -w/2 h/2
    l-w/2 -w/2 -w/2
    l+w/2 w/2 h/2
    l+w/2 w/2 -w/2
    l+w/2 -w/2 h/2
    l+w/2 -w/2 -w/2];

face_propbar_xpos = [  1  2  4 3
    5  6  8  7
    1  2  6  5
    1  3  7  5
    4  2  6  5
    4  3  7  8 ];


% Plot fuselage
surf(xbody,ybody,zbody,'FaceColor',maincolor,'FaceAlpha',alpha,'EdgeColor',maincolor,'EdgeAlpha',alpha)
% h = findobj('Type','surface');
% 
% % Change color for fuselage
color = zeros(size(zbody,1),size(zbody,2),3);
color(:,:,1) = maincolor(1)*ones(size(zbody));
color(:,:,2) = maincolor(2)*ones(size(zbody));
color(:,:,3) = maincolor(3)*ones(size(zbody));
% set(h,'Cdata',color)

fill3(xbody(1,:),ybody(1,:),zbody(1,:),maincolor,'FaceAlpha',alpha,'EdgeColor',edgecolor,'EdgeAlpha',alpha)
fill3(xbody(2,:),ybody(2,:),zbody(2,:),maincolor,'FaceAlpha',alpha,'EdgeColor',edgecolor,'EdgeAlpha',alpha)

% Plot bars
for i = 0:3
    % Decide color to apply
    if i == 0
        coloring = mainbarcolor;
    else
        coloring = barcolor;
    end
    R90deg = [0 -1 0; 1 0 0; 0 0 1];
    
    % Draw bars and propellers
    vertices = (rot*R90deg^(i)*vert_bar_xpos'*esc)' + kron(ones(length(vert_bar_xpos),1),position);
    patch('Vertices',vertices,'Faces',face_bar_xpos,'FaceColor', coloring,'LineWidth',LW,'FaceAlpha',alpha,'EdgeColor',edgecolor,'EdgeAlpha',alpha);
    vertices = (rot*R90deg^(i)*vert_propbar_xpos'*esc)' + kron(ones(length(vert_propbar_xpos),1),position);
    patch('Vertices',vertices,'Faces',face_propbar_xpos,'FaceColor',coloring,'LineWidth',LW,'FaceAlpha',alpha,'EdgeColor',edgecolor,'EdgeAlpha',alpha);
    
    % Compute appropriate proppeler coordinates
    
    for j=1:2
        prop = (rot*R90deg^(i)*([xproptmp(j,:)' yproptmp(j,:)' zproptmp(j,:)']*esc)')' ...
            + kron(ones(length(xproptmp),1),position);
        
        xprop(j,:) = prop(:,1);
        yprop(j,:) = prop(:,2);
        zprop(j,:) = prop(:,3);
    end
    
    surf(xprop,yprop,zprop,'FaceAlpha',alpha,'EdgeColor',edgecolor,'EdgeAlpha',alpha)
    
    fill3(xprop(1,:),yprop(1,:),zprop(1,:),propcolor,'FaceAlpha',alpha,'EdgeColor',edgecolor,'EdgeAlpha',alpha)
    fill3(xprop(2,:),yprop(2,:),zprop(2,:),propcolor,'FaceAlpha',alpha,'EdgeColor',edgecolor,'EdgeAlpha',alpha)
end

if auto
    grid on;
    axis equal;
    hold off;
end
