function peak = Observation_FPeakFind(P1,f,TPF)
%UNTITLED 此处提供此函数的摘要
%   此处提供详细说明


L = length(P1);
j = 1;
for i = 1:L
    if f(i) > TPF*0.98 && f(i) < TPF*1.02
        P1_peak(j) = P1(i);
        j = j + 1;
    end
end
peak = mean(P1_peak);

end