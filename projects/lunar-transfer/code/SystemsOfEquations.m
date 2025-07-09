function dy = SystemsOfEquations (x,y)
global m G n

% For each object there 3 position, and 3 velocity components. 6 in total.

dy = zeros(n*6,1); % YENİ SATIR (n: global cisim sayısı)

    for i=1:1:n
        toplam=[0 0 0];
        for j=1:1:n
            if i==j, continue, end;
            r=[ y((j-1)*6+1)-y((i-1)*6+1)  y((j-1)*6+2)-y((i-1)*6+2) y((j-1)*6+3)-y((i-1)*6+3)];
            toplam=toplam+(G*m(i)*m(j)*r/norm(r)^3);
        end
dy((i-1)*6+1) = y((i-1)*6+4);   % x coordinate of the object
dy((i-1)*6+2) = y((i-1)*6+5);   % y coordinate of the object
dy((i-1)*6+3) = y((i-1)*6+6);   % z coordinate of the object
dy((i-1)*6+4) = toplam(1)/m(i); % Velocity component along x 
dy((i-1)*6+5) = toplam(2)/m(i); % Velocity component along y 
dy((i-1)*6+6) = toplam(3)/m(i); % Velocity component along z    
    end;