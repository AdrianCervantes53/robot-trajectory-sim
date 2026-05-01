function theta=PTPpoli(po,pf,T)

syms t
dt=T*0.125;

po1=po;
pf1=pf;

t1=2*dt;
t2=T-2*dt;

m=(pf1-po1)/(T-2*dt);

pos=m*(t-dt)+po;

po2=subs(pos,t,2*dt);
pf2=subs(pos,t,T-2*dt);

% Etapa de aceleraci�n
po=po; pF=po2;
vo=0; vf=m;
ao=0; af=0;

a0=po;
a1=vo;
a2=ao/2;
a3=(1/(2*t1^3))*(20*(pF-po)-(8*vf+12*vo)*t1);
a4=(1/(2*t1^4))*(30*(po-pF)+(14*vf+16*vo)*t1);

theta1=a0+a1*t+a2*t^2+a3*t^3+a4*t^4;

theta2=m*(t-2*dt)+po2;

% Etapa de desaceleracion
po=pf2; pf=pf;
vo=m; vf=0;
ao=0; af=0;

a0=po;
a1=vo;
a2=ao/2;
a3=(1/(2*t1^3))*(20*(pf-po)-(8*vf+12*vo)*t1);
a4=(1/(2*t1^4))*(30*(po-pf)+(14*vf+16*vo)*t1);

theta3=a0+a1*(t-t2)+a2*(t-t2)^2+a3*(t-t2)^3+a4*(t-t2)^4;

theta=[theta1,theta2,theta3];

