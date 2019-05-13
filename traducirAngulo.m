function betha=traducirAngulo(delta, alpha)
%Lo primero es introducir las longitudes de las barras reales de
%nuestro diseño.
L1=0.055; 
L2=0.180;
L3=0.050;
L6=0.100;L5=0.006; 
L=0.180; 

%Calculamos la longitud del vector CA, que aunque es ficticio, define el
%eslabon fijo del problema y depende de el angulo alpha.
L4=sqrt((-L6-L*cos(alpha))^2+(L5+L*sin(alpha))^2);

%Calculamos el angulo que formann los vector CE y CA y el angulo gamma:

CE=[-L6; L5];
CA=[-L6-L*cos(alpha); L5+L*sin(alpha)];
landa=acos((CE'*CA)/(norm(CE)*norm(CA)));
gamma=atan(L5/L6);

%Para lograr tener un angulo delta en el codo necesitaremos el sigiente theta1 (q1):

q2=alpha-gamma-landa;

q1=pi-delta-q2;

%Implementamos la expresion analitica de un mecanismo de 4 eslabones, 1 fijo: 

c1=L4/L1; c3=L4/L3; c4=(L1^2-L2^2+L3^2+L4^2)/(2*L1*L3);

p=[(c1+c4+cos(q1)*(-1-c3)) -2*sin(q1) (-c1+c4+cos(q1)*(1-c3))];
x=roots(p);
q3=abs(2*atan(x));

%Devolvemos el angulo que debe tener el servo:
betha=(pi-gamma-landa-q3)*180/pi;
end