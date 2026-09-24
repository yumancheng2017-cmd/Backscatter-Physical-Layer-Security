function h = channel(d, lambda, N)


    % Path Loss
    pl_amp = lambda / (4 * pi * d);
    
    % Rayleigh Fading
    h_s = sqrt(0.5) * (randn(N, 1) + 1i * randn(N, 1));
    
    h = pl_amp * h_s;
end