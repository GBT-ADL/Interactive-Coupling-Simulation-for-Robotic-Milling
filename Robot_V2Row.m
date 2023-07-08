function row_vector = Robot_V2Row(vector)
% This function can convert any vector into row vector
% input:[vector] any vector [row/col]
% output:[row_vector] row vector [row]

% Data process
if isrow(vector)
    row_vector = vector;
elseif iscolumn(vector)
    row_vector = vector.';
else 
    error('The input is not a vector!')
end