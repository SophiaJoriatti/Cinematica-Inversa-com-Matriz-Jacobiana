clear; clc; close all;
pkg load symbolic

syms d th a alpha real
one = sym(1); zero = sym(0);
deg2rad_sym = @(d) sym(d) * sym(pi) / sym(180);

Tx = [one zero zero a; zero one zero zero; zero zero one zero; zero zero zero one];
Tz = [one zero zero zero; zero one zero zero; zero zero one d; zero zero zero one];
Rx = [one zero zero zero; zero cos(alpha) -sin(alpha) zero; zero sin(alpha) cos(alpha) zero; zero zero zero one];
Rz = [cos(th) -sin(th) zero zero; sin(th) cos(th) zero zero; zero zero one zero; zero zero zero one];

A = simplify(Tz * Rz * Tx * Rx);

syms th1 th2 th3 th4 th5 th6 th7 d1 a2 d3 a4 d5 a6 d7 real
I = [one zero zero zero; zero one zero zero; zero zero one zero; zero zero zero one];

A0 = subs(A, [d th a alpha], [zero th1 zero zero]);
A1 = subs(A, [d th a alpha], [d1 zero zero deg2rad_sym(90)]);
A2 = subs(A, [d th a alpha], [zero th2 a2 deg2rad_sym(-90)]);
A3 = subs(A, [d th a alpha], [d3 th3 zero deg2rad_sym(90)]);
A4 = subs(A, [d th a alpha], [zero th4 a4 deg2rad_sym(-90)]);
A5 = subs(A, [d th a alpha], [d5 th5 zero deg2rad_sym(90)]);
A6 = subs(A, [d th a alpha], [zero th6 a6 deg2rad_sym(-90)]);
A7 = subs(A, [d th a alpha], [d7 th7 zero deg2rad_sym(90)]);

F0 = simplify(I * A0);
F1 = simplify(A0 * A1);
F2 = simplify(A0 * A1 * A2);
F3 = simplify(A0 * A1 * A2 * A3);
F4 = simplify(A0 * A1 * A2 * A3 * A4);
F5 = simplify(A0 * A1 * A2 * A3 * A4 * A5);
F6 = simplify(A0 * A1 * A2 * A3 * A4 * A5 * A6);
F7 = simplify(A0 * A1 * A2 * A3 * A4 * A5 * A6 * A7);

V_J7 = F7(1:3,4);
printf('Calculando Jacobiano...\n');
J_end = jacobian(V_J7, [th1 th2 th3 th4 th5 th6 th7]);
printf('Jacobiano calculado.\n');

d1_val = 0.5; a2_val = 0.5; d3_val = 0.5; a4_val = 0.5;
d5_val = 0.5; a6_val = 0.5; d7_val = 0.5;

printf('Convertendo funções simbólicas para numéricas...\n');
F0_num = matlabFunction(F0, 'Vars', {d1, th1});
F1_num = matlabFunction(F1, 'Vars', {d1, a2, th1, th2});
F2_num = matlabFunction(F2, 'Vars', {d1, a2, d3, th1, th2, th3});
F3_num = matlabFunction(F3, 'Vars', {d1, a2, d3, a4, th1, th2, th3, th4});
F4_num = matlabFunction(F4, 'Vars', {d1, a2, d3, a4, d5, th1, th2, th3, th4, th5});
F5_num = matlabFunction(F5, 'Vars', {d1, a2, d3, a4, d5, a6, th1, th2, th3, th4, th5, th6});
F6_num = matlabFunction(F6, 'Vars', {d1, a2, d3, a4, d5, a6, d7, th1, th2, th3, th4, th5, th6, th7});
F7_num = matlabFunction(F7, 'Vars', {d1, a2, d3, a4, d5, a6, d7, th1, th2, th3, th4, th5, th6, th7});
J_end_num = matlabFunction(J_end, 'Vars', {d1, a2, d3, a4, d5, a6, d7, th1, th2, th3, th4, th5, th6, th7});
printf('Conversão concluída.\n');

N = -2:0.1:2;
Caminho_x = cos(2*pi*(N/4));
Caminho_y = sin(2*pi*(N/4));
Caminho_z = (N.^2)/2;

Theta = zeros(7,1);
n = 0.05; ksi = 1e-2; epsi = 1e-4;
Pose = []; PoseTH = []; Trajetoria_EE = [];

base_x = 0; base_y = 0; base_z = 0;
printf('Iniciando cinemática inversa...\n');
figure(1, 'Position', [100, 100, 1000, 800]);

for i = 1:length(N)
    printf('Ponto %d/%d: ', i, length(N));
    G = [Caminho_x(i); Caminho_y(i); Caminho_z(i)];
    
    try
        aux0 = F0_num(d1_val, Theta(1));
        aux1 = F1_num(d1_val, a2_val, Theta(1), Theta(2));
        aux2 = F2_num(d1_val, a2_val, d3_val, Theta(1), Theta(2), Theta(3));
        aux3 = F3_num(d1_val, a2_val, d3_val, a4_val, Theta(1), Theta(2), Theta(3), Theta(4));
        aux4 = F4_num(d1_val, a2_val, d3_val, a4_val, d5_val, Theta(1), Theta(2), Theta(3), Theta(4), Theta(5));
        aux5 = F5_num(d1_val, a2_val, d3_val, a4_val, d5_val, a6_val, Theta(1), Theta(2), Theta(3), Theta(4), Theta(5), Theta(6));
        aux6 = F6_num(d1_val, a2_val, d3_val, a4_val, d5_val, a6_val, d7_val, Theta(1), Theta(2), Theta(3), Theta(4), Theta(5), Theta(6), Theta(7));
        aux7 = F7_num(d1_val, a2_val, d3_val, a4_val, d5_val, a6_val, d7_val, Theta(1), Theta(2), Theta(3), Theta(4), Theta(5), Theta(6), Theta(7));
    catch
        printf('Erro no cálculo FK, continuando...\n');
        continue;
    end
    
    P_J0 = aux0(1:3,4); P_J1 = aux1(1:3,4); P_J2 = aux2(1:3,4); P_J3 = aux3(1:3,4);
    P_J4 = aux4(1:3,4); P_J5 = aux5(1:3,4); P_J6 = aux6(1:3,4); P_J7 = aux7(1:3,4);
    
    Trajetoria_EE = [Trajetoria_EE; P_J7'];
    erro = norm(G - P_J7);
    EQM = mean((G - P_J7).^2);
    
    iteration = 0; max_iter = 100;
    
    while (erro > ksi && iteration < max_iter)
        iteration = iteration + 1;
        GradEQM = (G - P_J7);
        
        try
            J = J_end_num(d1_val, a2_val, d3_val, a4_val, d5_val, a6_val, d7_val, ...
                Theta(1), Theta(2), Theta(3), Theta(4), Theta(5), Theta(6), Theta(7));
        catch
            printf('Erro no Jacobiano.\n');
            break;
        end
        
        lambda = 0.01;
        J_inv = J' / (J*J' + lambda*eye(3));
        GradTH = J_inv * GradEQM;
        Theta = Theta + n * GradTH;
        Theta = mod(Theta + pi, 2*pi) - pi;
        
        try
            aux7 = F7_num(d1_val, a2_val, d3_val, a4_val, d5_val, a6_val, d7_val, ...
                Theta(1), Theta(2), Theta(3), Theta(4), Theta(5), Theta(6), Theta(7));
            P_J7 = aux7(1:3,4);
        catch
            break;
        end
        
        erro = norm(G - P_J7);
        EQM = mean((G - P_J7).^2);
    end
    printf('Erro: %.4f (%d iterações)\n', erro, iteration);
    
    clf; hold on; grid on;
    
    plot3(Caminho_x, Caminho_y, Caminho_z, 'ob', 'linewidth', 1, 'markersize', 10);
    plot3(G(1), G(2), G(3), 'xm', 'linewidth', 2, 'markersize', 10);
    
    plot3(base_x, base_y, base_z, 'og', 'linewidth', 2, 'markersize', 10);
    plot3(base_x, base_y, base_z, 'og', 'linewidth', 2, 'markersize', 15);
    text(base_x+0.1, base_y+0.1, base_z+0.1, 'Frame {0}');
    
    plot3(P_J1(1), P_J1(2), P_J1(3), 'or', 'linewidth', 2, 'markersize', 10);
    plot3(P_J1(1), P_J1(2), P_J1(3), 'or', 'linewidth', 2, 'markersize', 15);
    text(P_J1(1)+0.1, P_J1(2)+0.1, P_J1(3)+0.1, 'Frame {1}');
    
    plot3(P_J2(1), P_J2(2), P_J2(3), 'sb', 'linewidth', 2, 'markersize', 10);
    plot3(P_J2(1), P_J2(2), P_J2(3), 'sb', 'linewidth', 2, 'markersize', 15);
    text(P_J2(1)+0.1, P_J2(2)+0.1, P_J2(3)+0.1, 'Frame {2}');
    
    plot3(P_J3(1), P_J3(2), P_J3(3), 'oc', 'linewidth', 2, 'markersize', 10);
    plot3(P_J3(1), P_J3(2), P_J3(3), 'oc', 'linewidth', 2, 'markersize', 15);
    text(P_J3(1)+0.1, P_J3(2)+0.1, P_J3(3)+0.1, 'Frame {3}');
    
    plot3(P_J4(1), P_J4(2), P_J4(3), 'sg', 'linewidth', 2, 'markersize', 10);
    plot3(P_J4(1), P_J4(2), P_J4(3), 'sg', 'linewidth', 2, 'markersize', 15);
    text(P_J4(1)+0.1, P_J4(2)+0.1, P_J4(3)+0.1, 'Frame {4}');
    
    plot3(P_J5(1), P_J5(2), P_J5(3), 'sr', 'linewidth', 2, 'markersize', 10);
    plot3(P_J5(1), P_J5(2), P_J5(3), 'sr', 'linewidth', 2, 'markersize', 15);
    text(P_J5(1)+0.1, P_J5(2)+0.1, P_J5(3)+0.1, 'Frame {5}');
    
    plot3(P_J6(1), P_J6(2), P_J6(3), 'ob', 'linewidth', 2, 'markersize', 10);
    plot3(P_J6(1), P_J6(2), P_J6(3), 'ob', 'linewidth', 2, 'markersize', 15);
    text(P_J6(1)+0.1, P_J6(2)+0.1, P_J6(3)+0.1, 'Frame {6}');
    
    plot3(P_J7(1), P_J7(2), P_J7(3), 'sk', 'linewidth', 2, 'markersize', 10);
    plot3(P_J7(1), P_J7(2), P_J7(3), 'sk', 'linewidth', 2, 'markersize', 15);
    text(P_J7(1)+0.1, P_J7(2)+0.1, P_J7(3)+0.1, 'Frame {7}');
    
    plot3([base_x, P_J1(1)], [base_y, P_J1(2)], [base_z, P_J1(3)], 'k', 'linewidth', 4, 'markersize', 10);
    plot3([P_J1(1), P_J2(1)], [P_J1(2), P_J2(2)], [P_J1(3), P_J2(3)], 'k', 'linewidth', 4, 'markersize', 10);
    plot3([P_J2(1), P_J3(1)], [P_J2(2), P_J3(2)], [P_J2(3), P_J3(3)], 'k', 'linewidth', 4, 'markersize', 10);
    plot3([P_J3(1), P_J4(1)], [P_J3(2), P_J4(2)], [P_J3(3), P_J4(3)], 'k', 'linewidth', 4, 'markersize', 10);
    plot3([P_J4(1), P_J5(1)], [P_J4(2), P_J5(2)], [P_J4(3), P_J5(3)], 'k', 'linewidth', 4, 'markersize', 10);
    plot3([P_J5(1), P_J6(1)], [P_J5(2), P_J6(2)], [P_J5(3), P_J6(3)], 'k', 'linewidth', 4, 'markersize', 10);
    plot3([P_J6(1), P_J7(1)], [P_J6(2), P_J7(2)], [P_J6(3), P_J7(3)], 'k', 'linewidth', 4, 'markersize', 10);
    
    if size(Trajetoria_EE, 1) > 1
        plot3(Trajetoria_EE(:,1), Trajetoria_EE(:,2), Trajetoria_EE(:,3), 'm-', 'linewidth', 2);
    end
    
    xlabel('x'); ylabel('y'); zlabel('z');
    title(sprintf('Ponto %d/%d | EE: [%.3f, %.3f, %.3f] | Alvo: [%.3f, %.3f, %.3f] | Erro: %.4f', ...
        i, length(N), P_J7(1), P_J7(2), P_J7(3), G(1), G(2), G(3), erro));
    view(3); axis equal; axis([-2 2 -2 2 -1 3]); hold off; drawnow;
end
printf('Cinemática inversa concluída!\n');
