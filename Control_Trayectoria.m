clc;
clear all;
close all;
to = 0;
tf = 60;
h = 0.02;
t = to:h:tf;


% % Cartesianas deseadas
ydr = 2*cos(0.2*t);
xdr = 2*sin(0.2*t);

% Calculo de velocidad en x y y deseada
xdrp(1) = 0;
ydrp(1) = 0;
for q = 2:length(t)
    ydrp(q) = (ydr(q)- ydr(q-1))/h;
    xdrp(q) = (xdr(q)- xdr(q-1))/h;
end
% Calculo de theta deseada
phir(1) = 0;
for q = 2:length(t)
    phir(q) = tan_inv(ydrp(q),xdrp(q));
end
% Calculo de velocidad en theta
for q = 2:length(t)
    phirp(q) = (phir(q) -phir(q-1))/h;
end

for q = 1:length(t) 
    inversa = pinv([cos(phir(q)),0;sin(phir(q)),0;0,1]);
    Q(:,q) = inversa*[xdrp(q);ydrp(q);phirp(q)];   
end

% % Condiciones iniciales
xc(1) = 0;
yc(1) = 0;
phic(1) = pi/2;
kp1 = 0.3145;
kp2 = 0.7342;
q1 = 0.5;
q2 = q1;
q3 = q1;

q = 1;

Rotz = [cos(phir(q)),sin(phir(q)),0;-sin(phir(q)),cos(phir(q)),0;0,0,1];
%Calculo del error
q = 1
ZigmaE(:,q) = Rotz* [ xdr(q) - xc(q); ydr(q) - yc(q); phir(q) - phic(q)]
xe = ZigmaE(1,q); 
ye = ZigmaE(2,q); 
thetae = ZigmaE(3,q); 
Vr(:) = Q(1,:);
Wr(:) = Q(2,:);

for q = 1:length(t)

    Vr(q) = Q(1,q);
    Wr(q) = Q(2,q);
    Rotz = [cos(phir(q)),sin(phir(q)),0;-sin(phir(q)),cos(phir(q)),0;0,0,1];
    %Calculo del error
    ZigmaE(:,q) = Rotz* [ xdr(q) - xc(q); ydr(q) - yc(q); phir(q) - phic(q)];
    xe = ZigmaE(1,q);
    ye = ZigmaE(2,q);
    thetae = ZigmaE(3,q);

    %Control propuesto
    V(q) = kp1*xe +Vr(q)*cos(thetae);
    W(q) = kp2*sin(thetae)*cos(thetae) + Wr(q);
    W(q) = W(q) + (q2*Vr(q)*ye*sin(thetae)^2)/((q1*xe^2+q2*ye^2+q3)*cos(thetae));
    
    %Calculo de velocidades cartesianas
    xrp(q) = V(q)*cos(phic(q));
    yrp(q) = V(q)*sin(phic(q));
    xrpa = xrp(q);
    yrpa = yrp(q);
    Wa = W(q);
    %Integral numerica
     
    if q == 1
        xc(q+1) = xc(q) + h*xrp(q);
        yc(q+1) = yc(q) + h*yrp(q);
        phic(q+1) = phic(q)+h*W(q); 
    else
        %Integral por metodo del trapecio
        xc(q+1) = xc(q) + h*(xrp(q)+ xrpa)/2; 
        yc(q+1) = yc(q) + h*(yrp(q)+ yrpa)/2;
        phic(q+1) = phic(q)+h*(W(q)+Wa)/2; 
    end
    
end
%-------------------
plot(xdr, ydr)
hold on
plot(xc,yc)
grid on
legend("Trayectoria deseada", "Trayectoria del controlador")
xlabel('x(m)')
ylabel('y(m)')

figure
plot(t,Vr,t,V)
legend("Velocidad lineal deseada", "Velocidad lineal del controlador")
xlabel("Tiempo (s)")
ylabel("Velocidad lineal (m/s)")
axis([0 60 0 0.7 ])

grid on
for  q = 1:length(t)-1
   
    if ZigmaE(3,q)<-5
        Wr(q) = Wr(q-1);
        W(q) = W(q-1);
        ZigmaE(3,q) = ZigmaE(3,q-1);
    end
end

figure
plot(t,ZigmaE(3,:))
axis([0 60 -2 2])
grid on
figure
y1 = t*0;
plot(t,ZigmaE(1,:),t,ZigmaE(2,:), t, ZigmaE(3,:))
legend("Error en X", "Error en Y","Error en \theta")
xlabel('Tiempo (s)')
ylabel('Errores')
grid on;

figure
plot(t,xdr,t,xc(1:end-1))
title("Trayectoria sobre el eje X")
legend("Trayectoria deseada en X", "Trayectoria deseada en X")
xlabel('Tiempo (s)')
ylabel('X(m)')


grid on
figure
plot(t,ydr,t,yc(1:end-1))
grid on
title("Trayectoria sobre el eje Y")
legend("Trayectoria deseada en Y", "Trayectoria deseada en Y")
xlabel('Tiempo (s)')
ylabel('Y(m)')


figure
plot(t,Wr,'r', t,W,'b--')
legend("Velocidad angular deseada", "Velocidad angular del controlador")
axis([0,60,-2 2])
xlabel('Tiempo (s)')
ylabel('Velocidad angular (rad/s)') 
grid on

