function result = Observation_belongto(x,a,b)
% This function can distinguish whether [x] belongs to interval [a,b] or not
% input: [x] object needs to be distinguished [scalar]
%        [a] a side of interval [scalar]
%        [b] the other side of interval [scalar]
% output [result] [scalar,0/1]
floor = min(a,b);
ceil = max(a,b);
if x >= floor && x <= ceil
    result = 1;
else
    result = 0;
end
end