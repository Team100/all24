% inverse kinematics of the device based on link lengths and end-effector
% position

function t = inv_kinematics(a1,a2,a3,a4,a5,x3,y3)

P13 = sqrt(x3.^2+y3.^2);
P53 = sqrt((x3+a5).^2 + y3.^2);

alphaOne = acos((a1.^2 + P13.^2 - a2.^2)./(2.*a1.*P13));
betaOne = atan2(y3,-x3);
thetaOne = pi - alphaOne - betaOne;

alphaFive = atan2(y3,x3+a5);
betaFive = acos((P53.^2 + a4.^2 - a3.^2)./(2.*P53.*a4));
thetaFive = alphaFive + betaFive;


t = [thetaOne thetaFive];