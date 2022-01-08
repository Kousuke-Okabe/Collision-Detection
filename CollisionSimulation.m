clear all

type3 = '2d_RRR';
type2 = '2d_RR';

q = rand(3,1)*pi/3;
Fr = [-0.1;-0.1;0];
Fi = [0.0;0];

% Force on task space
[J3,Jp,U,error] = fJacobi_q(type3,q);
Je = [J3;U'];
tau_r = Je'*Fr;
r = fKinematics(type3,q);

% Force on intermediate point
[J2,Jp] = fJacobi_minus_q(type2,q(1:2));
tau_i = [J2'*Fi;0];
r2 = fKinematics(type2,q(1:2));

% Torque
tau = tau_r+tau_i;
utau = U'*tau;

% figure
FH = 1;
figure(FH)
clf(FH)
fRoboAnimation(type3,FH,q, zeros(2,1),0)
quiver(r(1),r(2), Fr(1),Fr(2),'r', 'AutoScaleFactor',1)
quiver(r2(1),r2(2), Fi(1),Fi(2),'b', 'AutoScaleFactor',1)
xlim([0,0.4]);
ylim([0,0.4]);

if norm(utau) > 10^-5
    title(strcat('u^T\tau=',num2str(utau),' : Collision'))
else
    title(strcat('u^T\tau=',num2str(utau),' : without Collision'))
end