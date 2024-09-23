% kinematics of 2-DOF 5 bar planar mechanism
% 
% following the work of: 
% "The Pantograph Mk-II: A Haptic Instrument"
% Hayward, 2005
% 
% NOTE: THE AUTHOR'S DERIVATING OF THE JACOBIAN MISSES TWO NEGATIVE SIGNS
% (NOTED BELOW)
%
% inputs
% ai: the length of link i

function J = fwd_kinematics(a1,a2,a3,a4,a5,t1,t5,plot_linkage)

% Forward Kimematics% 

% by definition
x1 = 0;
y1 = 0;


x2 = a1*cos(t1);
y2 = a1*sin(t1);

x4 = a4*cos(t5)-a5;
y4 = a4*sin(t5);

x5 = -a5;
y5 = 0;

P2 = [x2;y2];
P4 = [x4;y4];

P2Ph = (a2^2-a3^2 + norm(P4-P2)^2) / (2 * norm(P4-P2));
Ph   = P2 + (P2Ph/norm(P2-P4)) * (P4 - P2);
P3Ph = sqrt(a2^2 - (P2Ph)^2);

x3 = Ph(1) + (P3Ph/norm(P2-P4)) * (y4 - y2);
y3 = Ph(2) - (P3Ph/norm(P2-P4)) * (x4 - x2);

P3 = [x3;y3];


%plot 
if plot_linkage == true
    plot(-[x1 x1],-[y1 y1],'o',-[x2 x2],-[y2 y2],'o',-[x3 x3],-[y3 y3],'o',-[x4 x4],-[y4 y4],'o',-[x5 x5],-[y5 y5],'o',-[x1 x2],-[x1 y2],-[x2 x3],-[y2 y3],-[x3 x4],-[y3 y4],-[x4 x5],-[y4 y5],-[x5 x1],-[y5 y1]);   
end


% -------------------------------Jacobian--------------------------------
d = norm(P2-P4);
b = norm(P2-Ph);
h = norm(P3-Ph);

del1_x2 = -a1*sin(t1);  %NOTE: THE AUTHOR FORGOT NEGATIVE SIGN IN THE PAPER
del1_y2 = a1*cos(t1);
del5_x4 = -a4*sin(t5);  %NOTE: THE AUTHOR FORGOT NEGATIVE SIGN IN THE PAPER
del5_y4 = a4*cos(t5);

del1_y4 = 0;
del1_x4 = 0;
del5_y2 = 0;
del5_x2 = 0;

% joint 1
del1_d = ( (x4-x2)*(del1_x4-del1_x2) + (y4-y2)*(del1_y4-del1_y2) ) / d;
del1_b = del1_d - (del1_d*(a2^2-a3^2+d^2))/(2*d^2);
del1_h = -b*del1_b / h;

del1_yh = del1_y2 + (del1_b*d-del1_d*b)/d^2 * (y4-y2) + b/d * (del1_y4 - del1_y2);
del1_xh = del1_x2 + (del1_b*d-del1_d*b)/d^2 * (x4-x2) + b/d * (del1_x4 - del1_x2);

del1_y3 = del1_yh - h/d * (del1_x4-del1_x2) - (del1_h*d - del1_d*h)/d^2 *(x4 - x2);
del1_x3 = del1_xh + h/d * (del1_y4-del1_y2) + (del1_h*d - del1_d*h)/d^2 *(y4 - y2);

% joint 2
del5_d = ( (x4-x2)*(del5_x4-del5_x2)+(y4-y2)*(del5_y4-del5_y2) ) / d;
del5_b = del5_d - (del5_d*(a2^2-a3^2+d^2))/(2*d^2);
del5_h = -b*del5_b / h;

del5_yh = del5_y2 + (del5_b*d-del5_d*b)/d^2 * (y4-y2) + b/d * (del5_y4 - del5_y2);
del5_xh = del5_x2 + (del5_b*d-del5_d*b)/d^2 * (x4-x2) + b/d * (del5_x4 - del5_x2);

del5_y3 = del5_yh - h/d * (del5_x4-del5_x2) - (del5_h*d - del5_d*h)/d^2 *(x4 - x2);
del5_x3 = del5_xh + h/d * (del5_y4-del5_y2) + (del5_h*d - del5_d*h)/d^2 *(y4 - y2);

J = [del1_x3 del5_x3; 
     del1_y3 del5_y3];
 

