clear; clc; close all;

%% 1. System Parameters Setting
f = 915e6;              % Carrier frequency: 915 MHz
c = 3e8;                % Speed of light: 3*10^8 m/s
lambda = c / f;         % Wavelength

%% 2. Power and Noise Configurations
Pt = 0.5;               % Reader transmit power: 0.5 W
noise_dBm = -80;        % Noise Power (dBm)
noise_W = (10^(noise_dBm / 10)) / 1000; % Convert dBm to Watts
sigma_R2 = noise_W;     % Noise power at the Reader
sigma_E2 = noise_W;     % Noise power at the Eavesdropper

%% 3. System Efficiency and Thresholds
eta_b = 0.8;            % Backscatter efficiency of the tag
eta_e = 0.8;            % Energy harvesting efficiency of the tag
P_th = 1e-6;            % Minimum power threshold for tag to operate (1 uW)
m_th = 0.2;             % Minimum backscatter reflection modulation depth

rho_SIC = 1e-4;         % Self-Interference Cancellation (SIC) factor at the reader

%% 4. Distance and Simulation Loop Settings
d_RU = 10;              % Fixed distance between Reader and User (Tag): 10 m
d_UE_list = 5:5:50;     % Distance list between User (Tag) and Eavesdropper
N_list = [3, 4, 5, 6];  % List of antenna numbers at the Reader

num_N = length(N_list);
num_d = length(d_UE_list);

% Matrices to store the final results for plotting
SR_results = zeros(num_N, num_d);       
RR_results = zeros(num_N, num_d);       
Gamma0_results = zeros(num_N, num_d);   
Gamma1_results = zeros(num_N, num_d);   
SR_bf_results = zeros(num_N, num_d);    

example_SR_history = []; % To record the convergence history for Figure 4

disp('Start the Simulation...');

%% 5. Main Simulation Loop
for n_idx = 1:num_N
    N = N_list(n_idx);
    
    for d_idx = 1:num_d
        d_UE = d_UE_list(d_idx);
        d_RE = d_RU + d_UE; % Direct distance from Reader to Eavesdropper
        
        rng(42); % Fix random seed for reproducibility
        
        % Generate wireless channels (Path Loss + Rayleigh Fading)
        h_RU = channel(d_RU, lambda, N);
        h_RE = channel(d_RE, lambda, N);
        h_UE = channel(d_UE, lambda, 1);

        % =========================================================
        % Baseline: Brute Force Method 
        % =========================================================
        % Use MRT beamforming directed purely at the tag
        w_bf = conj(h_RU) / norm(h_RU); 
        E_inc_bf = 0.5 * eta_e * Pt * abs(h_RU.' * w_bf)^2;
        gamma_step = 0.01; 
        Gamma_grid = 0 : gamma_step : 1; 
    
        SR_best_bf = 0;
        Gamma0_best_bf = 0;
        Gamma1_best_bf = 0;

        for g0_idx = 1:length(Gamma_grid)
            for g1_idx = 1:length(Gamma_grid)
                G0 = Gamma_grid(g0_idx);
                G1 = Gamma_grid(g1_idx);
                
                if G1 >= G0, continue; end
                m_val = (G0 - G1) / 2;
                if m_val < m_th, continue; end
                
                % Check energy harvesting constraint
                energy_harvested = E_inc_bf * (2 - G0^2 - G1^2);
                if energy_harvested < P_th, continue; end
        
                % Calculate SNR and Spectral Efficiency for Brute Force
                SNR_R_bf = (Pt * eta_b * norm(h_RU)^2 * abs(h_RU.' * w_bf)^2 * m_val^2) / sigma_R2;
                R_R_bf = log2(1 + SNR_R_bf);
                
                z_bf = abs(h_RE.' * w_bf)^2;
                Jamming_E = Pt * rho_SIC * z_bf; % Interference caused by Reader to Eve
                SNR_E_bf = (Pt * eta_b * abs(h_UE)^2 * abs(h_RU.' * w_bf)^2 * m_val^2) / (sigma_E2 + Jamming_E);
                R_E_bf = log2(1 + SNR_E_bf);
        
                SR_val = max(0, R_R_bf - R_E_bf);
        
                % Update best Brute Force records
                if SR_val > SR_best_bf
                    SR_best_bf = SR_val;
                    Gamma0_best_bf = G0;
                    Gamma1_best_bf = G1;
                end
            end
        end

        % =========================================================
        % Advanced: Alternating Optimization ___ Damped SCA & SDR
        % =========================================================
        max_iter = 15;        
        epsilon = 1e-4;       
        SR_previous = -100;      
        
        % Channel Covariance Matrices for Semidefinite Relaxation
        H_RU_mat = conj(h_RU) * h_RU.';
        H_RE_mat = conj(h_RE) * h_RE.';
        
        w_noise = (randn(N, 1) + 1i * randn(N, 1));
        w_noise = w_noise / norm(w_noise);
        w_init = sqrt(0.85) * w_bf + sqrt(0.15) * w_noise; 
        w_init = w_init / norm(w_init);
        
        % Lift the vector w to a matrix W
        W_opt = w_init * w_init';
        
        temp_SR_history = zeros(max_iter, 1);
        
        for iter = 1:max_iter
            
            % ---------------------------------------------------------
            % Sub-problem 1: Optimize Reflection Coefficients (\Gamma)
            % ---------------------------------------------------------
            x_val = max(real(trace(H_RU_mat * W_opt)), 0); % Incident power variable
            E_inc = 0.5 * eta_e * Pt * x_val;
            Gamma_square_sum_limit = 2 - (P_th / (E_inc + eps));
            
            cvx_begin quiet
                variables Gamma0 Gamma1;
                maximize( Gamma0 - Gamma1 );
                subject to
                    0 <= Gamma1;
                    Gamma1 <= Gamma0;
                    Gamma0 <= 1;
                    (Gamma0 - Gamma1) / 2 >= m_th;
                    square(Gamma0) + square(Gamma1) <= Gamma_square_sum_limit;
            cvx_end
            
            if strcmp(cvx_status, 'Infeasible') || strcmp(cvx_status, 'Failed')
                Gamma0_opt = 1; Gamma1_opt = 0; 
            else
                Gamma0_opt = Gamma0; Gamma1_opt = Gamma1;
            end
            
            % ---------------------------------------------------------
            % Sub-problem 2: Optimize Precoding Matrix W 
            % ---------------------------------------------------------
            m_val = (Gamma0_opt - Gamma1_opt) / 2;
            Gamma_energy_factor = 2 - Gamma0_opt^2 - Gamma1_opt^2;
            
            CR = (Pt * eta_b * norm(h_RU)^2 * m_val^2) / sigma_R2;
            CE_tilde = (Pt * eta_b * abs(h_UE)^2 * m_val^2) / sigma_E2;
            CRE_tilde = (Pt * rho_SIC) / sigma_E2; 
            
            % Calculate local points for the first-order Taylor approximation
            x0 = max(real(trace(H_RU_mat * W_opt)), 0);
            z0 = max(real(trace(H_RE_mat * W_opt)), 0);
            y0_tilde = 1 + CRE_tilde * z0 + CE_tilde * x0; % Normalized denominator
            
            cvx_expert true; 
            cvx_begin quiet
                variable W(N, N) hermitian semidefinite; % SDR: Relax w*w^H to matrix W
                expression x; expression z; expression y_tilde; expression taylor_approx;
                
                x = real(trace(H_RU_mat * W));
                z = real(trace(H_RE_mat * W));
                y_tilde = 1 + CRE_tilde * z + CE_tilde * x;
                
                % SCA: First-order Taylor Expansion to linearize the non-convex Eve rate
                taylor_approx = -log(y0_tilde)/log(2) - (y_tilde - y0_tilde) / (y0_tilde * log(2));
                
                % Objective: Maximize Secrecy Rate
                maximize( log(1 + CR * x)/log(2) + log(1 + CRE_tilde * z)/log(2) + taylor_approx );
                subject to
                    real(trace(W)) <= 1; % Power constraint
                    x >= P_th / (0.5 * eta_e * Pt * Gamma_energy_factor + eps); % Energy constraint
                    x >= 0; z >= 0;
            cvx_end
            
            % Instead of full update, we take a fractional step (0.3) towards the CVX solution.
            
            if ~strcmp(cvx_status, 'Infeasible') && ~strcmp(cvx_status, 'Failed')
                 step_size = 0.3; 
                 W_opt = (1 - step_size) * W_opt + step_size * W; 
            end
            
            % Record the theoretical SR of the relaxed matrix W for the convergence plot
            x_rel = max(real(trace(H_RU_mat * W_opt)), 0);
            z_rel = max(real(trace(H_RE_mat * W_opt)), 0);
            
            SNR_R_rel = CR * x_rel;
            SNR_E_rel = (CE_tilde * x_rel) / (1 + CRE_tilde * z_rel);
            
            SR_iter_val = max(0, log2(1 + SNR_R_rel) - log2(1 + SNR_E_rel));
            temp_SR_history(iter) = SR_iter_val;
            
            if abs(SR_iter_val - SR_previous) < epsilon
                break;
            end
            SR_previous = SR_iter_val;
        end
        
        % =========================================================
        % Physical Beam Extraction (Rank-1 Projection)
        % =========================================================
        W_eval_final = full((W_opt + W_opt')/2);
        [V_f, D_f] = eig(W_eval_final);
        [lMax_f, max_idx_f] = max(real(diag(D_f)));
        w_final = V_f(:, max_idx_f) * sqrt(max(lMax_f, 0));
        w_final = w_final / (norm(w_final) + eps); % Final normalized physical beam
        
        % Calculate actual physical metrics
        x_final = abs(h_RU.' * w_final)^2;
        z_final = abs(h_RE.' * w_final)^2;
        
        SNR_R_final = (Pt * eta_b * norm(h_RU)^2 * x_final * m_val^2) / sigma_R2;
        R_R_final = log2(1 + SNR_R_final);
        SNR_E_final = (Pt * eta_b * abs(h_UE)^2 * x_final * m_val^2) / (sigma_E2 + Pt * rho_SIC * z_final); 
        R_E_final = log2(1 + SNR_E_final);
        
        SR_final = max(0, R_R_final - R_E_final);
        
        % Write records to main matrices
        SR_results(n_idx, d_idx) = SR_final;
        RR_results(n_idx, d_idx) = R_R_final;
        Gamma0_results(n_idx, d_idx) = Gamma0_opt;
        Gamma1_results(n_idx, d_idx) = Gamma1_opt;
        SR_bf_results(n_idx, d_idx) = SR_best_bf;
        
        % Export the convergence history of a specific scenario
        if N == 4 && d_UE == 20
            example_SR_history = temp_SR_history(1:iter);
        end
        
    end
    disp(['The simulation of the case N = ', num2str(N), ' has completed']);
end

disp('Simulation part has completed. Plotting...');

% =========================================================
% Plotting Section
% =========================================================
colors = {'r', 'b', 'k', 'm'};
markers = {'o', 's', 'd', '^'};

%% Figure 1: Secrecy Rate (SR) vs. Transmission Distance
figure(1); hold on; box on; grid on;

for n_idx = 1:num_N
    plot(d_UE_list, SR_results(n_idx, :), ['-', colors{n_idx}, markers{n_idx}], ...
        'LineWidth', 1.5, 'MarkerSize', 7, 'DisplayName', ['CVX: N = ', num2str(N_list(n_idx))]);
end

if exist('SR_bf_results', 'var')
    for n_idx = 1:num_N
        plot(d_UE_list, SR_bf_results(n_idx, :), 'k*', ...
            'LineWidth', 1.5, 'MarkerSize', 8, 'HandleVisibility', 'off'); 
    end
    plot(NaN, NaN, 'k*', 'LineWidth', 1.5, 'DisplayName', 'Brute Force Benchmark');
end

xlabel('Distance between Tag and Easesdropper d_{UE} (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('SR (bps/Hz)', 'FontSize', 12, 'FontWeight', 'bold');
title('SR vs. Transmission Distance', 'FontSize', 14);
legend('Location', 'best', 'FontSize', 10);

%% Figure 2: Reflection Coefficients vs. Transmission Distance
figure(2); 
for n_idx = 1:num_N
    subplot(2, 2, n_idx);
    hold on; box on; grid on;
    
    plot(d_UE_list, Gamma0_results(n_idx, :), '-ro', 'LineWidth', 1.5, 'MarkerSize', 6);
    plot(d_UE_list, Gamma1_results(n_idx, :), '-bs', 'LineWidth', 1.5, 'MarkerSize', 6);
    
    xlabel('d_{UE} (m)');
    ylabel(' Reflection Coefficient \Gamma');
    title(['N = ', num2str(N_list(n_idx))]);
    
    if n_idx == 1 
        legend('\Gamma_0', '\Gamma_1', 'Location', 'best');
    end
end
sgtitle('Reflection Coefficient vs. Transmission Distance', 'FontSize', 14, 'FontWeight', 'bold');

%% Figure 3: Legitimate Reader Spectral Efficiency
figure(3); hold on; box on; grid on;

for n_idx = 1:num_N
    plot(d_UE_list, RR_results(n_idx, :), ['-', colors{n_idx}, markers{n_idx}], ...
        'LineWidth', 1.5, 'MarkerSize', 7, 'DisplayName', ['N = ', num2str(N_list(n_idx))]);
end

xlabel('Distance between Tag and Easesdropper d_{UE} (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Reader Spectral Efficiency R_R (bps/Hz)', 'FontSize', 12, 'FontWeight', 'bold');
title('SE at Reader vs. Transmission Distance', 'FontSize', 14);
legend('Location', 'best', 'FontSize', 10);

%% Figure 4: SCA Convergence Plot
figure(4); hold on; box on; grid on;

if exist('example_SR_history', 'var') && ~isempty(example_SR_history)
    valid_iters = find(example_SR_history > 0); 
    plot(valid_iters, example_SR_history(valid_iters), '-*b', 'LineWidth', 2, 'MarkerSize', 8);
    
    xlabel('Iteration Number', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('SR (bps/Hz)', 'FontSize', 12, 'FontWeight', 'bold');
    title('Convergence Plot Example for N=4, d_{UE}=20', 'FontSize', 14);
else
    disp('Error, Skip Figure 4');
end