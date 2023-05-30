function output = my_mpc(input1, input2, input3, input4)
    import casadi.*

    N = 10;  % Control/prediction horizon
    dt = 0.1;  % Time step

    % Define the variables
    X = SX.sym('X', N);  % States
    U = SX.sym('U', N);  % Control inputs

    % Define the state equation
    Xdot = U;

    % Define the cost function
    J = X'*X + U'*U;

    % Create the optimization problem
    opts = struct('ipopt', struct('print_level', 0), 'print_time', 0);
    nlp = struct('x', [X; U], 'f', J, 'g', X - dt*Xdot);
    solver = nlpsol('solver', 'ipopt', nlp, opts);

    % Define the constraints and initial guess
    lbx = [-inf*ones(N, 1); -1*ones(N, 1)];  % Lower bounds on [X; U]
    ubx = [ inf*ones(N, 1);  1*ones(N, 1)];  % Upper bounds on [X; U]
    x0 = [input1; input2; input3; input4; zeros(N-4, 1); zeros(N, 1)];  % Initial guess

    % Solve the optimization problem
    sol = solver('x0', x0, 'lbx', lbx, 'ubx', ubx, 'lbg', 0, 'ubg', 0);

    % Extract the first optimal control input
    u_opt = full(sol.x(N+1));

    % Define the output
    output = u_opt;
end