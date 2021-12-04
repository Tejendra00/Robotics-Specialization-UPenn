syms F Mb1 Mb2 Mb3 b c p1 p2 p3 p4
U=[F;Mb1;Mb2;Mb3]
M=[1,1,1,1;
    b,-b,b,-b;
    -b,-b,b,b;
    -c,c,c,-c]
P=[p1;p2;p3;p4]

I=inv(M)
P=I*U
simplify(P)