clear all
close all

L(1) = Link([0 675 350 -pi/2], 'standard');
L(2) = Link([0 0 1350 0], 'standard');
L(3) = Link([0 0 1220 0], 'standard');
L(4) = Link([0 0 280 pi/2], 'standard');
L(5) = Link([0 0 0 0], 'standard');

KR = SerialLink(L);

qf = [pi/2 -pi/6 -pi/4 pi/2 0];
Tf = KR.fkine(qf);

q0 = [0 0 0 0 0];
q = KR.ikine(Tf,q0,'mask', [1 1 1 1 1 0]);

t = 0:0.1:5;
Q = jtraj(q0,qf,t);
Tr = fkine(KR,Q)

for i = 1:1:length(t)
    T = Tr(i);
    trs = transl(T);
    xx(i) = trs(1);
    yy(i) = trs(2);
    zz(i) = trs(3);

end

plot(KR,Q);
hold on
plot3(xx,yy,zz,'color',[1 0 0], 'LineWidth', 2)

