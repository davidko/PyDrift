function s_out = bicycle_car_diff(t, s)

global U a b C_f C_r m I

% State variables: x, y, theta, V, r
x = s(1);
y = s(2);
theta = s(3);
V = s(4);
r = s(5);

if t > 1 && t < 2
  delta = 20 * pi/180;
else
  delta=0;
end


alpha_f = delta - atan2((V + r*a), U);
alpha_r = -atan2((V - r*b), U);
F_f = C_f * alpha_f;
F_r = C_r * alpha_r;

r_dot_top = a * F_f * cos(delta) - b*F_r;
r_dot = r_dot_top / I;

v_dot = (F_f * delta + F_r)/m - r*U;

x_dot = U*cos(theta) - V*sin(theta);
y_dot = U*sin(theta) + V*cos(theta);

s_out = zeros(5, 1);
s_out(1) = x_dot;
s_out(2) = y_dot;
s_out(3) = r;
s_out(4) = v_dot;
s_out(5) = r_dot;
