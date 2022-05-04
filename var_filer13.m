function [A,B] = var_filer13(p)
global NE

B = reshape(p,1,4*NE);
A = p(1,1);