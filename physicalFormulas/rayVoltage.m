function voltages = rayVoltage(rays, cfg)


    fc = cfg.transmit_params.fc;
    c = cfg.transmit_params.c;
    lambda = c / fc;
    % P_TX = V_TX^2 / (Z_a + Z_S);
    % -> V_TX = sqrt(P_TX * (Z_a + Z_S));
    % Z_a = Z_S = 720*pi/32;
    V_TX = sqrt(cfg.transmit_params.TX_power * (720 * pi / 16));

    voltages = 1j * rays.reflexionAttenuation .* (2*lambda*exp(-1j*2*pi*fc*rays.lengths/c) ./ (3*pi^2*rays.lengths)) * V_TX;

end
