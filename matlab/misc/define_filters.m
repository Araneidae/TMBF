% Some miscellaneous experimental ideas for filter design and experiments.  Run
% this script to define the filter management functions listed here.

% Simple one pole IIR
IIR = @(b) { [b]; [-(1-b) 1] };

% Integrator
I = { [1]; [-1 1] };

delay = { [1 0]; [1] };

filter_one = { [1]; [1] };

filter_k = @(k, a) { k * a{1}; a{2} };

% Compose two filters
filter_mul = @(a, b) { conv(a{1}, b{1}); conv(a{2}, b{2}) };
filter_inv = @(a) { a{2}; a{1} };
filter_div = @(a, b) filter_mul(a, filter_inv(b));

% Pad polynomial to specified length
poly_pad = @(p, n) [zeros(1, n-length(p)) p];

% Trim leading zeros from polynomial
poly_trim = @(p) p(find(p, 1, 'first'):end);

% Add two polynomials
poly_add_ = @(a, b, n) poly_pad(a, n) + poly_pad(b, n);
poly_add = @(a, b) poly_trim(poly_add_(a, b, max(length(a), length(b))));

% Add two filters
filter_reduce__ = @(a, b, g) { deconv(a, g); deconv(b, g) };
filter_reduce_ = @(a, b) filter_reduce__(a, b, poly_gcd(a, b));
%filter_reduce_ = @(a, b) { a; b };
filter_add = @(a, b) filter_reduce_( ...
    poly_add(conv(a{1}, b{2}), conv(a{2}, b{1})), conv(a{2}, b{2}));

filter_sub = @(a, b) filter_add(a, {-b{1}; -b{2}});

% Filter feedback
feedback = @(f) filter_inv(filter_sub(filter_one, f));
feedback2 = @(f) filter_div(f, filter_sub(filter_one, f));

PI = @(ki, kp) filter_add(filter_k(ki, I), filter_k(kp, filter_one));
