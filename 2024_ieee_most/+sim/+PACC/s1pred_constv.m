function [s1pred, v1pred] = s1pred_constv(s1, v1, tpred)
v1pred = v1 * ones(size(tpred));
s1pred = s1 + cumtrapz(tpred, v1pred);