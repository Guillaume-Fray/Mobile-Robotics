A = ones(3);
B = ones(3);

x = input('Choose x1:  \n');
fprintf('\n ');
y = input('Choose y1:  \n');
fprintf('\n ');
a1 = input('Choose a1:  \n');
fprintf('\n ');
x11 = cos(a1);
x12 = -sin(a1);
x13 = x;
x21 = sin(a1);
x22 = cos(a1);
x23 = y;
x31 = 0;
x32 = 0;
x33 = 1;


x2 = input('Choose x2:  \n');
fprintf('\n ');
y2 = input('Choose y2:  \n');
fprintf('\n ');
a2 = input('Choose a2:  \n');
fprintf('\n ');
w11 = cos(a2);
w12 = -sin(a2);
w13 = x2;
w21 = sin(a2);
w22 = cos(a2);
w23 = y2;
w31 = 0;
w32 = 0;
w33 = 1;

A(1, 1) = x11;
A(1, 2) = x12;
A(1, 3) = x13;
A(2, 1) = x21;
A(2, 2) = x22;
A(2, 3) = x23;
A(3, 1) = x31;
A(3, 2) = x32;
A(3, 3) = x33;

B(1, 1) = w11;
B(1, 2) = w12;
B(1, 3) = w13;
B(2, 1) = w21;
B(2, 2) = w22;
B(2, 3) = w23;
B(3, 1) = w31;
B(3, 2) = w32;
B(3, 3) = w33;

C = inv(A);
D = C\B;
E = B\C;
rel_target = C*B;


