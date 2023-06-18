function [s] = ms_force(r,b,m2)  %m1은 m_target, %m2은 m_source
u0=4*pi*10^-7;
m1=(10^-9)*1.4/u0*b/norm(b);
m2=(10^-6)*1.4/u0*m2/norm(m2);
s=3*u0/(4*pi*norm(r)^5) *(dot(m1,r)*m2+dot(m2,r)*m1+dot(m1,m2)*r-5*dot(m1,r)*dot(m2,r)*r/norm(r)^2);

