function udot=Modelo_nao_linear(x)
global m g rho Cd A uw f
u=x(1);
Fx=x(2);
Theta = x(3);
udot=(Fx/m)-(g*sin(Theta))-(f*g*cos(Theta))-(0.5*rho*A*Cd*(u+uw)^2)/m;
end
