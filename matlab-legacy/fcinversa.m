function Q=fcinversa(px,py,pz,A,B,C,handles)
try
% P=evalin('base','P');

L1=5; L2=5; L3=5;

r2=px^2+py^2;
R2=r2+(pz-L1)^2;

cq3=(R2-L2^2-L3^2)/(2*L2*L3);
try
   sq3=sqrt(1-cq3^2);
   q3=atan2(sq3,cq3);
catch
   sq3=-sqrt(1-cq3^2); 
   q3=atan2(sq3,cq3);
end

q1=atan2(py,px);
  
alfa=atan2(pz-L1,sqrt(r2));
beta=atan2(L3*sq3,L2+L3*cq3);

    q2=alfa-beta;
if q2<0
    q2=alfa+beta;
    q3=-atan2(sq3,cq3);
end


% if q1<0
%    q2=abs(q2-180)
%    q3=abs(q2-180)
% end

q1=rad2deg(q1);
q2=rad2deg(q2);
q3=rad2deg(q3);

Rx=[cos(A) -sin(A) 0; sin(A) cos(A) 0; 0 0 1];
Ry=[cos(B) 0 sin(B); 0 1 0; -sin(B) 0 cos(B)];
Rz=[1 0 0; 0 cos(C) -sin(C); 0 sin(C) cos(C)];
R=Rx*Rz*Ry;

A1=evalin('base','A1');
A2=evalin('base','A2');
A3=evalin('base','A3');

Aa=[A1(1,1) A1(1,2) A1(1,3); A1(2,1) A1(2,2) A1(2,3); A1(3,1) A1(3,2) A1(3,3)];
Ab=[A2(1,1) A2(1,2) A2(1,3); A2(2,1) A2(2,2) A2(2,3); A2(3,1) A2(3,2) A2(3,3)];
Ac=[A3(1,1) A3(1,2) A3(1,3); A3(2,1) A3(2,2) A3(2,3); A3(3,1) A3(3,2) A3(3,3)];

AR=(Aa*Ab*Ac);
AT=transpose(AR);
MT=AT*R;

q4=atan2(MT(3,2),MT(2,2))
q5=atan2(-MT(1,2),sqrt((MT(2,2))^2+(MT(3,2))^2))
q6=atan2(MT(1,3),MT(1,1))

% Q=[rad2deg(q1),rad2deg(q2),rad2deg(q3),px,py,pz];
% assignin('base','Q',Q);
Q=[q1 q2 q3 q4 q5 q6];
catch
    msgbox('Singularidad');
    Q=evalin('base','Q');
    q1=Q(1); q2=Q(2); q3=Q(3); q4=Q(4); q5=Q(5); q6=Q(6); 
    Q=[q1 q2 q3 q4 q5 q6];
end