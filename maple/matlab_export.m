t1 = cos(0.2e1);
t2 = cos(0.3e1);
t4 = sin(0.3e1);
t6 = sin(0.2e1);
t7 = cos(0.1e1);
t8 = t7 * t4;
t9 = sin(0.1e1);
t10 = t2 * t9;
t13 = t7 * t2;
t14 = t4 * t9;
unknown(1,1) = t1 * t2;
unknown(1,2) = -t1 * t4;
unknown(1,3) = t6;
unknown(2,1) = t10 * t6 + t8;
unknown(2,2) = -t14 * t6 + t13;
unknown(2,3) = -t1 * t9;
unknown(3,1) = -t13 * t6 + t14;
unknown(3,2) = t8 * t6 + t10;
unknown(3,3) = t1 * t7;