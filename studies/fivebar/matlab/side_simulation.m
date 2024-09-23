% simulates the device moving around the edges of the workspace.  Plots the
% device at the 4 corners

clear
clc
close all
set(0,'defaultfigurewindowstyle','docked')

% plot?
plot_linkage = true;

% max current per motor
Imax = 6;   %A
kt = .0234; %Nm/A
Tmax = Imax*kt*ones(2,1); %Nm

% link lengths
a1 = .25;
a2 = .25;
a3 = a2;
a4 = a1;
a5 = .1;

% origin
x1 = 0;
y1 = 0;

% the rectangle
xcenter = -(x1+a5)/2;
ycenter = .34;
w = .2794;
h = .2159;

% find solve inverse kinematics at the boundary of the paper
number = 30;
xpoints1 = linspace(xcenter      , xcenter + w/2, number);
xpoints2 = linspace(xcenter + w/2, xcenter + w/2, number);
xpoints3 = linspace(xcenter + w/2, xcenter - w/2, number);
xpoints4 = linspace(xcenter - w/2, xcenter - w/2, number);
xpoints5 = linspace(xcenter - w/2, xcenter + w/2, number);

xpoints = [xpoints1 xpoints2 xpoints3 xpoints4 xpoints5];

ypoints1 = linspace(ycenter      , ycenter + h/2, number);
ypoints2 = linspace(ycenter + h/2, ycenter - h/2, number);
ypoints3 = linspace(ycenter - h/2, ycenter - h/2, number);
ypoints4 = linspace(ycenter - h/2, ycenter + h/2, number);
ypoints5 = linspace(ycenter + h/2, ycenter + h/2, number);

ypoints = [ypoints1 ypoints2 ypoints3 ypoints4 ypoints5];

theta = zeros(2,length(xpoints));
for i = 1:length(xpoints)
    
    % %find joint angles
    x3 = xpoints(i);
    y3 = ypoints(i);
    
    theta = inv_kinematics(a1,a2,a3,a4,a5,x3,y3);
    t1 = theta(1);
    t5 = theta(2);
    
    % jacobian
    J(:,:,i) = fwd_kinematics(a1,a2,a3,a4,a5,t1,t5,plot_linkage);
    
    % figure set up
    xlabel('-x');ylabel('-y')
    set(gcf,'color','white');
    axis equal
    axis([-.25 .3 -.45 0.03])
    grid on
    
    % plot the rectangle
    rectangle('Position',[-(xcenter+w/2) -(ycenter+h/2) w h])
    
    frame(i) = getframe;
    
    
end




%-----------------------------Plot 4 positions---------------------
figure
subplot(2,2,1)
theta = inv_kinematics(a1,a2,a3,a4,a5,max(xpoints),max(ypoints));
t1 = theta(1);
t5 = theta(2);
J(:,:,i) = fwd_kinematics(a1,a2,a3,a4,a5,t1,t5,plot_linkage);
% figure set up
xlabel('-x');ylabel('-y')
set(gcf,'color','white');
axis equal
axis([-.25 .35 -.45 0.03])
grid on
rectangle('Position',[-(xcenter+w/2) -(ycenter+h/2) w h])


subplot(2,2,2)
theta = inv_kinematics(a1,a2,a3,a4,a5,min(xpoints),max(ypoints));
t1 = theta(1);
t5 = theta(2);
J(:,:,i) = fwd_kinematics(a1,a2,a3,a4,a5,t1,t5,plot_linkage);
% figure set up
xlabel('-x');ylabel('-y')
set(gcf,'color','white');
axis equal
axis([-.25 .35 -.45 0.03])
grid on
rectangle('Position',[-(xcenter+w/2) -(ycenter+h/2) w h])

subplot(2,2,3)
theta = inv_kinematics(a1,a2,a3,a4,a5,max(xpoints),min(ypoints));
t1 = theta(1);
t5 = theta(2);
J(:,:,i) = fwd_kinematics(a1,a2,a3,a4,a5,t1,t5,plot_linkage);
% figure set up
xlabel('-x');ylabel('-y')
set(gcf,'color','white');
axis equal
axis([-.25 .35 -.45 0.03])
grid on
rectangle('Position',[-(xcenter+w/2) -(ycenter+h/2) w h])

subplot(2,2,4)
theta = inv_kinematics(a1,a2,a3,a4,a5,min(xpoints),min(ypoints));
t1 = theta(1);
t5 = theta(2);
J(:,:,i) = fwd_kinematics(a1,a2,a3,a4,a5,t1,t5,plot_linkage);
% figure set up
xlabel('-x');ylabel('-y')
set(gcf,'color','white');
axis equal
axis([-.25 .35 -.45 0.03])
grid on
rectangle('Position',[-(xcenter+w/2) -(ycenter+h/2) w h])










