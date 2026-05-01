function LIN(pf1,pf2,pf3,handles)

        T=evalin('base','T');
        dt=0.1;
        n=T/dt;
        
        P=evalin('base','P');
        PEF=[P(1) P(2) P(3)];
        assignin('base','PEF',PEF);
        
        q4=P(4);
        q5=P(5);
        q6=P(6);

        po1=P(1);
        po2=P(2);
        po3=P(3);

        dx=(pf1-po1)/n;
        dy=(pf2-po2)/n;
        dz=(pf3-po3)/n;
        
        px=po1;
        py=po2;
        pz=po3;
        for i=0:n
           L=fcinversa(px,py,pz,P(4),P(5),P(6),handles);
           fcdirecta(L(1),L(2),L(3),q4,q5,q6,handles);
           px=px+dx;
           py=py+dy;
           pz=pz+dz;
           l=evalin('base','l');
           TPEF(l);
           pause(0.1);
        end
        l=2;
        assignin('base','l',l);