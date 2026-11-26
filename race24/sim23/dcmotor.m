%pkg load control
R=1; L=.01; J=.01; b=.1; kt=.05; kb=.05;
s=tf('s');
Gs=kt/((L*s+R)*(J*s+b)+kt*kb)
step(Gs), hold
L=0;
Gs1=kt/((L*s+R)*(J*s+b)+kt*kb)
step(Gs1)