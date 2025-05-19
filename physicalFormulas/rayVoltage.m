function voltages = rayVoltage(rays, cfg)


    fc = cfg.transmit_params.fc;
    c = cfg.transmit_params.c;
    lambda = c / fc;

    electricField = rays.reflexionAttenuation .* sqrt(60 * (16/(3*pi)) * cfg.transmit_params.TX_power) .* exp(-1j*2*pi*fc*rays.lengths./c) ./ rays.lengths;

    voltages = 0.5 .* electricField .* lambda/pi;

end
