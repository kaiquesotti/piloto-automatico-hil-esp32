% Parâmetros do modelo longitudinal - Modelo não Linear
% Exemplo Livro Automotive Control Systems

clc; clear all; close all

global m g rho Cd A uw f

% Dinâmica longitudinal - Parametros do modelo
u0=20; g=9.81; m=1000; f=0.015;
rho=1.202; A=1; Cd=0.5; uw=2;
Tau=(m/(rho*A*Cd*(u0+uw)));
K=Tau/m;

% Parametros da valvula Borboleta
Ka=10; TauA=0.2;