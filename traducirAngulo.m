function q3=traducirAngulo(q, alpha)

%q es el angulo que queremos fijar en el codo. Por lo que nuestro angulo
%será:
q1=pi-q;
%Lo primero es cambiar las longitudes de las barras por las reales de
%nuestro diseño.
L1=1;%L1 es la distancia del eje del codo al eje con la barra paralela que tira
L2=1;%L2 es la longitud entre ejes de la barra paralela
L3=1;%L3 es la longitud entre ejes de la barrita que va al servo.
L4=1;%L4 es la distancia entre ejes del eslabon principal
L5=1;L6=1; %distancias horizontal y vertical entre ejes de los servos
L=1; %distancia del brazo principal
%Implementamos la expresion analitica de un mecanismo de 4 eslabones, 1 fijo: 

c1=L4/L1; c3=L4/l3; c4=(L1^2-L2^2+L3^2+L4^2)/(2*L1*L3);

p=[(c1+c4+cos(q1)*(-1-c3)) -2*sin(q1) (-c1+c4+cos(-q1)*(1-c3))];
x=root(p);
y=atan(2*x);

%calculamos el angulo que forma el vector entre ejes de los servos y el eje
%del servo2 con el eje del codo:

vector1=[-L5;L6];
vector2=[-L5+L*cos(alpha);L6+L*sin(alpha)];
betha=acos((vector1'*vector2)/(norm(vector1)*norm(vector2)));

%devolvemos el angulo que debe tener el servo:
q3=pi-y-betha;
end