
% Initialize bicycle car model attributes

global U a b C_f C_r m I

m = 3600.0 / 2.2; % Kg
L = 7.0 / 3.28; % m
b = L / (1 + (0.85));
a = L - b;
I = 0.5 * m * a * b;
C_f = 210.0 * (4.44822) * (180/pi); %Newtons per radian
K_u = 1.0; % Oversteer coefficient
C_r = K_u * (a/b) * C_f;

U_crit_top = ((a+b)^2) * C_f * C_r;
U_crit = U_crit_top / m*(a*C_f - b*C_r);
U_crit = sqrt(U_crit);

U = U_crit * 1.1;

% State variables: U, V, R
[t, s_out] = ode45(@bicycle_car_diff, [0, 5], [0, 0, 0, 0, 0]);

plot(t, s_out(:, 5));
