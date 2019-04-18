function T_inv = invert_homo_trans(T)

%HOMO_TRANS2ROT_MAT inverts the homo_trans. meaning T_WC gets to T_CW
T_inv = zeros(3,4);

T_inv(1:3,4) = -T(1:3,1:3)'*T(1:3,4);
T_inv(1:3,1:3) = T(1:3,1:3)';

end