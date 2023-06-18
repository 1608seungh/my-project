function k = ms(r,m)
U0=4*pi*10^-7;   %자석고유값
I=eye(3);
rv = r/norm(r);
k= U0*(3*rv*rv.'-I)*m./(4*pi*norm(r)^3);
end