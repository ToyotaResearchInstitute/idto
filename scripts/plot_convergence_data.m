dd=importdata('../../bazel-out/k8-opt/bin/traj_opt/examples/spinner.runfiles/drake/solver_stats.csv', ',', 1);
d=dd.data;

iters = d(:,1);
cost = d(:,3);
delta = d(:,6);
q_norms = d(:,7);
dq = d(:,8);
dqH = d(:,9);
rho = d(:,10);
gnorm = d(:,11);
dLdqH = d(:,12);
dLdq = d(:,13);


figure(1)
h=semilogy(iters, dq, iters, dqH);
%set(h,'LineWidth',2)
legend('\Deltaq', '\DeltaqH');
xlabel('Iteration','FontName','Times', 'FontSize',16)
ylabel('\Deltaq','FontName','Times', 'FontSize',16)
set(gca, 'FontName','Times', 'FontSize',16)


figure(2)
h=semilogy(iters,abs(dLdqH), '-r', ...
         iters, abs(dLdq), '-g', ... 
         iters, gnorm.*dq/cost, '-b',...
         iters(2:end), abs(diff(cost))./cost(2:end), '-c',...
         iters, dq./q_norms, '-k');
legend('dLdqH', 'dLdq','|g|*|dq|/L','|\DeltaL|/L','\Deltaq|/|q|')
xlabel('Iteration','FontName','Times', 'FontSize',16)
ylabel('dLdq/L','FontName','Times', 'FontSize',16)
set(gca, 'FontName','Times', 'FontSize',16)
