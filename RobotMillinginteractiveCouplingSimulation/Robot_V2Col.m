function col_vector = Robot_V2Col(vector)
% This function can convert any vector into column vector
% input:[vector] any vector [row/col]
% output:[col_vector] col vector [col]

% Data process
if isrow(vector)
    col_vector = vector.';
elseif iscolumn(vector)
    col_vector = vector;
else 
    error('The input is not a vector!')
end