function [b,knot]=bsplinesSymbolic(d,knot)
% b returns a 3-level cell array: b{k}{i}{j} is the value of the k-th
% degree i-th bspline on the j-th knot interval (from knot(j) to knot(j+1))

% in this way we keep simple (polynomial formulations) on each interval
% and later on the computations are simpler
syms t
b={};

m=length(knot)-1;

k=1;
for i=1:m
    for j=1:m
        b{k}{i}{j}=0;
    end
end

for i=1:m
    if knot(i)<knot(i+1)
        b{k}{i}{i}=1;
    end
end   

for k=2:d
    for i=1:m-k+1
        for j=1:m
            if (knot(i+k-1)-knot(i)~=0) && (knot(i+k)-knot(i+1)~=0)
                b{k}{i}{j}=((b{k-1}{i}{j}.*(t-knot(i))./(knot(i+k-1)-knot(i)) + b{k-1}{i+1}{j}.*(knot(i+k)-t)./(knot(i+k)-knot(i+1))));
            elseif (knot(i+k-1)-knot(i)~=0)
                b{k}{i}{j}=(b{k-1}{i}{j}.*(t-knot(i))./(knot(i+k-1)-knot(i)));
            elseif (knot(i+k)-knot(i+1)~=0)
                b{k}{i}{j}=(b{k-1}{i+1}{j}.*(knot(i+k)-t)./(knot(i+k)-knot(i+1)));
            else
                b{k}{i}{j}=(zeros(size(t)));
            end
        end
    end
end