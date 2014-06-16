% Fits FIR with given parameters.  Experimental.
function [f, ff] = fit_fir(w0, delta, delay, phi0, N, alpha, beta)
    M = zeros(N);
    V = zeros(N, 1);
    for n = 0:N-1
        for m = 0:N-1
            M(n+1, m+1) = I(w0, delta, m - n, 0) + beta;
        end
        M(n+1, n+1) = M(n+1, n+1) + alpha;
        V(n+1) = I(w0, delta, -delay-n, delay * w0 - phi0);
    end
    f = linsolve(M, V);
    %f = f - mean(f);
    ff = fft([f; zeros(1024-N,1)]);
    disp(cond(M));
end


function s = sinc(x)
    s = ones(size(x));
    ix = find(x);
    s(ix) = sin(x(ix)) ./ x(ix);
end

function x = I(w0, delta, k, ph)
    x = 2 * delta * cos(w0 * k + ph) .* sinc(k * delta);
end
