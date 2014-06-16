function show_response(system)
    N = fliplr(system{1});
    D = fliplr(system{2});

    f = linspace(0, 0.5, 2^16);
    z = exp(2j * pi * f);
    r = polyval(N, z) ./ polyval(D, z);

    subplot(2,2,1);
    loglog(f, abs(r));
    subplot(2,2,3);
    semilogx(f, 180/pi*unwrap(angle(r)));

    subplot(2,2,[2 4]);
    rN = roots(N);
    rD = roots(D);
    c = exp(1j * linspace(0, 2*pi, 1024));
    plot(real(rN), imag(rN), 'o', ...
        real(rD), imag(rD), 'x', ...
        real(c), imag(c), '--');
    axis equal;
end
