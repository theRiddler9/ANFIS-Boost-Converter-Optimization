function training_convergence_plot(trainError, chkError, savePath)
%TRAINING_CONVERGENCE_PLOT Plots and saves ANFIS training/validation
%error curves. Called from train_anfis.m, or standalone if you've saved
%trainError/chkError separately.
%
%   training_convergence_plot(trainError, chkError, savePath)

figure;
plot(trainError, 'LineWidth', 1.5); hold on;
plot(chkError, 'LineWidth', 1.5);
xlabel('Epoch'); ylabel('RMSE');
legend('Training error', 'Validation error');
title('ANFIS Training Convergence');
grid on;

if nargin >= 3 && ~isempty(savePath)
    saveas(gcf, savePath);
    fprintf('Saved convergence plot to: %s\n', savePath);
end

end
