A = input("Enter A matrix (In this format'[a,b;c,d]')= ");
%This is useful for symmetric matrix only
if det(A)==0
    fprintf("Det is ZERO.\nSo inverse is not possible.")
else
    N = length(A);
    L = zeros(N,N);
    U = zeros(N,N);
    IL = zeros(N,N);
    IU = zeros(N,N);
    L(1,1) = sqrt(A(1,1));
    U(1,1) = L(1,1);
    for a=2:N
        L(a,1) = A(a,1)/L(1,1);
        U(1,a) = L(a,1);
    end
    for i=2:N
        for j=i:N
           if i==j
               L(j,i) = sqrt(A(j,i)-(L(j,1:i-1)*U(1:i-1,j)));
               U(j,i) = L(j,i);
           else
               L(j,i) = (A(j,i)-(L(j,1:i-1)*U(1:i-1,i)))/L(i,i);
               U(i,j) = L(j,i);
           end
        end
    end
    for i=1:N
        for j=i:N
            if i==j
                IL(j,i) = 1/(L(j,i));
                IU(j,i) = IL(i,j);
            else
                IL(j,i) = (-L(j,i:j-1)*IL(i:j-1,i))/(L(j,j));
                IU(i,j) = IL(j,i);
            end
        end
    end
    A_inv = IU*IL
end