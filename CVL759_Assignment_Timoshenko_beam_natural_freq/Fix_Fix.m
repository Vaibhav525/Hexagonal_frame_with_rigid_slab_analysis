%To calculate natural frequency of Timoshenko beam for Fix-Fixcase
%STEP 1: Input geometrical and physical properties of beam
E=2.1e11;   %Pa
G=8.1e10;   %Pa
rho=7860;   %kg/m3
L=1;        %m
b=0.02;     %m
h=0.08;     %m
I=b*h^3/12; %m4
A=b*h;      %m2
k=1.2;      %Shear coefficient

format long g   %For output formatting


%STEP2: Applying Boundary conditions
%For w<root(GkA/rho.I)
%X(x)=P1 cosh(r1x) + P2 sinh(r1x) + P3 cos(r2x) + P4 sin(r2x)
%T(t)=ACoswt+BSinwt

%For w>root(GkA/rho.I)
%X(x)=P1 cos(r1x) + P2 sin(r1x) + P3 cos(r2x) + P4 sin(r2x)
%T(t)=ACoswt+BSinwt


%Take series of w values
w_l=[0:0.1:90000]';
Res=zeros(900001,1);
Det_prev=50;
counter=1;
for i=1:length(w_l)
    w=w_l(i);   %Assumed w
    %Calulate the parameters
    a=w*w*rho/(k*G);
    c=G*k*A/(E*I);
    b=(rho*w*w/E)-c;
    d=a+b+c;
    e=a*b;

    Delta= d*d-4*e;
    %Calculate r1 and r2 for assumed w, and form the A_matrix for given
    %boundary condition
    
    if w<sqrt(k*G*A/(rho*I))
        r1=sqrt(0.5*(-d+sqrt(Delta)));
        r2=sqrt(0.5*(d+sqrt(Delta)));
        A_matrix=[1 0 1 0;
                  0 r1*(r1*r1+a+c) 0 r2*(-r2*r2+a+c);
                  cosh(r1*L) sinh(r1*L) cos(r2*L) sin(r2*L);
                  r1*(r1*r1+a+c)*sinh(r1*L) r1*(r1*r1+a+c)*cosh(r1*L) r2*(r2*r2-a-c)*sin(r2*L) r2*(-r2*r2+a+c)*cos(r2*L)];

    else
        r1=sqrt(0.5*(d-sqrt(Delta)));
        r2=sqrt(0.5*(d+sqrt(Delta)));
        A_matrix=[1 0 1 0;
                  0 r1*(-r1*r1+a+c) 0 r2*(-r2*r2+a+c);
                  cos(r1*L) sin(r1*L) cos(r2*L) sin(r2*L);
                  r1*(r1*r1-a-c)*sin(r1*L) r1*(-r1*r1+a+c)*cos(r1*L) r2*(r2*r2-a-c)*sin(r2*L) r2*(-r2*r2+a+c)*cos(r2*L)];
    end

    %Calculate det(A_matrix)=0
    Det=det(A_matrix);
    Res(i)=Det; %Add the value for graph to results
    if(Det*Det_prev<0 && counter<6)

        %If Det(A_matrix) changes sign at this w then this must be a root

        disp(w);        %Display this natural frequency
        counter=counter+1;  %Display only first 5 natural frequencies
    end
    Det_prev=Det;
end

%Plot shows variation of det(A) vs w, roots of this curve are natural
%frequency

plot(w_l,Res)
xlabel("Frequecy(w),rad/s ------>")
ylabel("Det(A) ------>")

