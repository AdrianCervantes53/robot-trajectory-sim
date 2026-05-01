function PTP(pf1,pf2,pf3,handles)

    Q=evalin('base','Q');
    P=evalin('base','P');
    PEF=[P(1) P(2) P(3)];
    assignin('base','PEF',PEF);
    
    q4=P(4);
    q5=P(5);
    q6=P(6);

    po1=Q(1);
    po2=Q(2);
    po3=Q(3);
    po=[po1 po2 po3];
    
    Velocidad=evalin('base','Vel');
    Vell=abs(Velocidad-100);
    Vel=Vell/100;
    velmax=2*pi;
    pf=[pf1 pf2 pf3];        
    dismax=max(abs(pf-po));
    T=dismax/velmax*Vel;
    dt=T*0.125;
           
    theta1=PTPpoli(po1,pf1,T);
    theta2=PTPpoli(po2,pf2,T);
    theta3=PTPpoli(po3,pf3,T);        
        
    syms t
    cla   
    for y=0:0.1:T
       if y<=(T/4)
           P1=subs(theta1(1),t,y);
           P2=subs(theta2(1),t,y);
           P3=subs(theta3(1),t,y);
       end
       if (y<=(T-T/4))&&(y>T/4)
           P1=subs(theta1(2),t,y);
           P2=subs(theta2(2),t,y);
           P3=subs(theta3(2),t,y);
       end
       if (y>(T-T/4))
           P1=subs(theta1(3),t,y);
           P2=subs(theta2(3),t,y);
           P3=subs(theta3(3),t,y);
       end
       
       P1=double(P1);
       P2=double(P2);
       P3=double(P3);
       fcdirecta(P1,P2,P3,q4,q5,q6,handles);
       l=evalin('base','l');
       TPEF(l);
       
    end
    l=2;
    assignin('base','l',l);
