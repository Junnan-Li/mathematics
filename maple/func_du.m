function func_dureturn = func_du(x, t)
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s24 = 0.3098386677e2 * t - 0.2323790008e2;
  else
    s24 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s23 = 0.3098386677e2 * x - 0.2323790008e2;
  else
    s23 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s22 = 0.1732050808e1;
  else
    s22 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s21 = 0.3098386677e2 * x - 0.7745966692e1;
  else
    s21 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s20 = 0.1732050808e1;
  else
    s20 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s19 = 0.1732050808e1;
  else
    s19 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s18 = 0.3098386677e2 * x - 0.2323790008e2;
  else
    s18 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s17 = 0.1732050808e1;
  else
    s17 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s16 = 0.3098386677e2 * x - 0.7745966692e1;
  else
    s16 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s15 = 0.1732050808e1;
  else
    s15 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s14 = 0.3098386677e2 * t - 0.7745966692e1;
  else
    s14 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s13 = 0.3098386677e2 * x - 0.2323790008e2;
  else
    s13 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s12 = 0.1732050808e1;
  else
    s12 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s11 = 0.3098386677e2 * x - 0.7745966692e1;
  else
    s11 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s10 = 0.1732050808e1;
  else
    s10 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s9 = 0.1732050808e1;
  else
    s9 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s8 = 0.3098386677e2 * x - 0.2323790008e2;
  else
    s8 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s7 = 0.1732050808e1;
  else
    s7 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s6 = 0.3098386677e2 * x - 0.7745966692e1;
  else
    s6 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s5 = 0.1732050808e1;
  else
    s5 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s4 = 0.3098386677e2 * t - 0.2323790008e2;
  else
    s4 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s3 = 0.1732050808e1;
  else
    s3 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s2 = 0.3098386677e2 * t - 0.7745966692e1;
  else
    s2 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s1 = 0.1732050808e1;
  else
    s1 = 0.0e0;
  end
  du = 0.1e1 / (0.1e1 + exp(x)) ^ 2 + 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 - 0.2500000000e0 + x * (0.1e1 / (0.1e1 + exp(0.1e1 - 0.5e1 * t)) ^ 2 - 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + 0.1776705118e0 + 0.415431679756514e-1 * s1 + 0.922094377856479e-2 * s2 + 0.603742508215732e-1 * s3 - 0.399645630498528e-2 * s4) + (-0.243051684581302e-2 * s5 - 0.809061198761621e-3 * s6 - 0.152377552205917e-1 * s7 - 0.195593427342862e-2 * s8) * s9 + (-0.433590063316381e-3 * s10 - 0.146112803263678e-3 * s11 - 0.319022339097685e-2 * s12 - 0.477063086307787e-3 * s13) * s14 + (-0.276114805649180e-2 * s15 - 0.933166016624500e-3 * s16 - 0.207984584912892e-1 * s17 - 0.314360556336114e-2 * s18) * s19 + (0.172746997599710e-3 * s20 + 0.586775450031145e-4 * s21 + 0.136190009033518e-2 * s22 + 0.211410172315387e-3 * s23) * s24;
  func_dureturn = du;
function func_dureturn = func_du(x, t)
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s24 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s24 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s23 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s23 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s22 = 0.1732050808e1;
  else
    s22 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s21 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s21 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s20 = 0.1732050808e1;
  else
    s20 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s19 = 0.1732050808e1;
  else
    s19 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s18 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s18 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s17 = 0.1732050808e1;
  else
    s17 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s16 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s16 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s15 = 0.1732050808e1;
  else
    s15 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s14 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s14 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s13 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s13 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s12 = 0.1732050808e1;
  else
    s12 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s11 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s11 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s10 = 0.1732050808e1;
  else
    s10 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s9 = 0.1732050808e1;
  else
    s9 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s8 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s8 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s7 = 0.1732050808e1;
  else
    s7 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s6 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s6 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s5 = 0.1732050808e1;
  else
    s5 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s4 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s4 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s3 = 0.1732050808e1;
  else
    s3 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s2 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s2 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s1 = 0.1732050808e1;
  else
    s1 = 0.0e0;
  end
  du = -0.2500000000e0 + 0.1e1 / (0.1e1 + exp(x)) ^ 2 + 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + x * (0.1776705118e0 + 0.1e1 / (0.1e1 + exp(0.1e1 - 0.5e1 * t)) ^ 2 - 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + 0.415431679756514e-1 * s1 + 0.922094377856479e-2 * s2 + 0.603742508215732e-1 * s3 - 0.399645630498528e-2 * s4) + (-0.243051684581302e-2 * s5 - 0.809061198761621e-3 * s6 - 0.152377552205917e-1 * s7 - 0.195593427342862e-2 * s8) * s9 + (-0.433590063316381e-3 * s10 - 0.146112803263678e-3 * s11 - 0.319022339097685e-2 * s12 - 0.477063086307787e-3 * s13) * s14 + (-0.276114805649180e-2 * s15 - 0.933166016624500e-3 * s16 - 0.207984584912892e-1 * s17 - 0.314360556336114e-2 * s18) * s19 + (0.172746997599710e-3 * s20 + 0.586775450031145e-4 * s21 + 0.136190009033518e-2 * s22 + 0.211410172315387e-3 * s23) * s24;
  func_dureturn = du;
function func_dureturn = func_du(x, t)
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s24 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s24 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s23 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s23 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s22 = 0.1732050808e1;
  else
    s22 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s21 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s21 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s20 = 0.1732050808e1;
  else
    s20 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s19 = 0.1732050808e1;
  else
    s19 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s18 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s18 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s17 = 0.1732050808e1;
  else
    s17 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s16 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s16 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s15 = 0.1732050808e1;
  else
    s15 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s14 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s14 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s13 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s13 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s12 = 0.1732050808e1;
  else
    s12 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s11 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s11 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s10 = 0.1732050808e1;
  else
    s10 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s9 = 0.1732050808e1;
  else
    s9 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s8 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s8 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s7 = 0.1732050808e1;
  else
    s7 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s6 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s6 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s5 = 0.1732050808e1;
  else
    s5 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s4 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s4 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s3 = 0.1732050808e1;
  else
    s3 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s2 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s2 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s1 = 0.1732050808e1;
  else
    s1 = 0.0e0;
  end
  du = -0.2500000000e0 + 0.1e1 / (0.1e1 + exp(x)) ^ 2 + 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + x * (0.1776705118e0 + 0.1e1 / (0.1e1 + exp(0.1e1 - 0.5e1 * t)) ^ 2 - 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + 0.415431679756514e-1 * s1 + 0.922094377856479e-2 * s2 + 0.603742508215732e-1 * s3 - 0.399645630498528e-2 * s4) + (-0.243051684581302e-2 * s5 - 0.809061198761621e-3 * s6 - 0.152377552205917e-1 * s7 - 0.195593427342862e-2 * s8) * s9 + (-0.433590063316381e-3 * s10 - 0.146112803263678e-3 * s11 - 0.319022339097685e-2 * s12 - 0.477063086307787e-3 * s13) * s14 + (-0.276114805649180e-2 * s15 - 0.933166016624500e-3 * s16 - 0.207984584912892e-1 * s17 - 0.314360556336114e-2 * s18) * s19 + (0.172746997599710e-3 * s20 + 0.586775450031145e-4 * s21 + 0.136190009033518e-2 * s22 + 0.211410172315387e-3 * s23) * s24;
  func_dureturn = du;
function func_dureturn = func_du(x, t)
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s24 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s24 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s23 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s23 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s22 = 0.1732050808e1;
  else
    s22 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s21 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s21 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s20 = 0.1732050808e1;
  else
    s20 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s19 = 0.1732050808e1;
  else
    s19 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s18 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s18 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s17 = 0.1732050808e1;
  else
    s17 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s16 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s16 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s15 = 0.1732050808e1;
  else
    s15 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s14 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s14 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s13 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s13 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s12 = 0.1732050808e1;
  else
    s12 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s11 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s11 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s10 = 0.1732050808e1;
  else
    s10 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s9 = 0.1732050808e1;
  else
    s9 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s8 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s8 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s7 = 0.1732050808e1;
  else
    s7 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s6 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s6 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s5 = 0.1732050808e1;
  else
    s5 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s4 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s4 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s3 = 0.1732050808e1;
  else
    s3 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s2 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s2 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s1 = 0.1732050808e1;
  else
    s1 = 0.0e0;
  end
  du = -0.2500000000e0 + 0.1e1 / (0.1e1 + exp(x)) ^ 2 + 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + x * (0.1776705118e0 + 0.1e1 / (0.1e1 + exp(0.1e1 - 0.5e1 * t)) ^ 2 - 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + 0.415431679756514e-1 * s1 + 0.922094377856479e-2 * s2 + 0.603742508215732e-1 * s3 - 0.399645630498528e-2 * s4) + (-0.243051684581302e-2 * s5 - 0.809061198761621e-3 * s6 - 0.152377552205917e-1 * s7 - 0.195593427342862e-2 * s8) * s9 + (-0.433590063316381e-3 * s10 - 0.146112803263678e-3 * s11 - 0.319022339097685e-2 * s12 - 0.477063086307787e-3 * s13) * s14 + (-0.276114805649180e-2 * s15 - 0.933166016624500e-3 * s16 - 0.207984584912892e-1 * s17 - 0.314360556336114e-2 * s18) * s19 + (0.172746997599710e-3 * s20 + 0.586775450031145e-4 * s21 + 0.136190009033518e-2 * s22 + 0.211410172315387e-3 * s23) * s24;
  func_dureturn = du;
function func_dureturn = func_du(x, t)
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s24 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s24 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s23 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s23 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s22 = 0.1732050808e1;
  else
    s22 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s21 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s21 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s20 = 0.1732050808e1;
  else
    s20 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s19 = 0.1732050808e1;
  else
    s19 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s18 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s18 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s17 = 0.1732050808e1;
  else
    s17 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s16 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s16 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s15 = 0.1732050808e1;
  else
    s15 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s14 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s14 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s13 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s13 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s12 = 0.1732050808e1;
  else
    s12 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s11 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s11 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s10 = 0.1732050808e1;
  else
    s10 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s9 = 0.1732050808e1;
  else
    s9 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s8 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s8 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s7 = 0.1732050808e1;
  else
    s7 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s6 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s6 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s5 = 0.1732050808e1;
  else
    s5 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s4 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s4 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s3 = 0.1732050808e1;
  else
    s3 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s2 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s2 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s1 = 0.1732050808e1;
  else
    s1 = 0.0e0;
  end
  du = -0.2500000000e0 + 0.1e1 / (0.1e1 + exp(x)) ^ 2 + 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + x * (0.1776705118e0 + 0.1e1 / (0.1e1 + exp(0.1e1 - 0.5e1 * t)) ^ 2 - 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + 0.415431679756514e-1 * s1 + 0.922094377856479e-2 * s2 + 0.603742508215732e-1 * s3 - 0.399645630498528e-2 * s4) + (-0.243051684581302e-2 * s5 - 0.809061198761621e-3 * s6 - 0.152377552205917e-1 * s7 - 0.195593427342862e-2 * s8) * s9 + (-0.433590063316381e-3 * s10 - 0.146112803263678e-3 * s11 - 0.319022339097685e-2 * s12 - 0.477063086307787e-3 * s13) * s14 + (-0.276114805649180e-2 * s15 - 0.933166016624500e-3 * s16 - 0.207984584912892e-1 * s17 - 0.314360556336114e-2 * s18) * s19 + (0.172746997599710e-3 * s20 + 0.586775450031145e-4 * s21 + 0.136190009033518e-2 * s22 + 0.211410172315387e-3 * s23) * s24;
  func_dureturn = du;
function func_dureturn = func_du(x, t)
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s24 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s24 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s23 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s23 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s22 = 0.1732050808e1;
  else
    s22 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s21 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s21 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s20 = 0.1732050808e1;
  else
    s20 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s19 = 0.1732050808e1;
  else
    s19 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s18 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s18 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s17 = 0.1732050808e1;
  else
    s17 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s16 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s16 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s15 = 0.1732050808e1;
  else
    s15 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s14 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s14 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s13 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s13 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s12 = 0.1732050808e1;
  else
    s12 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s11 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s11 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s10 = 0.1732050808e1;
  else
    s10 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s9 = 0.1732050808e1;
  else
    s9 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s8 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s8 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s7 = 0.1732050808e1;
  else
    s7 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s6 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s6 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s5 = 0.1732050808e1;
  else
    s5 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s4 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s4 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s3 = 0.1732050808e1;
  else
    s3 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s2 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s2 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s1 = 0.1732050808e1;
  else
    s1 = 0.0e0;
  end
  du = -0.2500000000e0 + 0.1e1 / (0.1e1 + exp(x)) ^ 2 + 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + x * (0.1776705118e0 + 0.1e1 / (0.1e1 + exp(0.1e1 - 0.5e1 * t)) ^ 2 - 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + 0.415431679756514e-1 * s1 + 0.922094377856479e-2 * s2 + 0.603742508215732e-1 * s3 - 0.399645630498528e-2 * s4) + (-0.243051684581302e-2 * s5 - 0.809061198761621e-3 * s6 - 0.152377552205917e-1 * s7 - 0.195593427342862e-2 * s8) * s9 + (-0.433590063316381e-3 * s10 - 0.146112803263678e-3 * s11 - 0.319022339097685e-2 * s12 - 0.477063086307787e-3 * s13) * s14 + (-0.276114805649180e-2 * s15 - 0.933166016624500e-3 * s16 - 0.207984584912892e-1 * s17 - 0.314360556336114e-2 * s18) * s19 + (0.172746997599710e-3 * s20 + 0.586775450031145e-4 * s21 + 0.136190009033518e-2 * s22 + 0.211410172315387e-3 * s23) * s24;
  func_dureturn = du;
function func_dureturn = func_du(x, t)
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s24 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s24 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s23 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s23 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s22 = 0.1732050808e1;
  else
    s22 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s21 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s21 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s20 = 0.1732050808e1;
  else
    s20 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s19 = 0.1732050808e1;
  else
    s19 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s18 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s18 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s17 = 0.1732050808e1;
  else
    s17 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s16 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s16 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s15 = 0.1732050808e1;
  else
    s15 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s14 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s14 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s13 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s13 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s12 = 0.1732050808e1;
  else
    s12 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s11 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s11 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s10 = 0.1732050808e1;
  else
    s10 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s9 = 0.1732050808e1;
  else
    s9 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s8 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s8 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s7 = 0.1732050808e1;
  else
    s7 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s6 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s6 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s5 = 0.1732050808e1;
  else
    s5 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s4 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s4 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s3 = 0.1732050808e1;
  else
    s3 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s2 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s2 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s1 = 0.1732050808e1;
  else
    s1 = 0.0e0;
  end
  du = -0.2500000000e0 + 0.1e1 / (0.1e1 + exp(x)) ^ 2 + 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + x * (0.1776705118e0 + 0.1e1 / (0.1e1 + exp(0.1e1 - 0.5e1 * t)) ^ 2 - 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + 0.415431679756514e-1 * s1 + 0.922094377856479e-2 * s2 + 0.603742508215732e-1 * s3 - 0.399645630498528e-2 * s4) + (-0.243051684581302e-2 * s5 - 0.809061198761621e-3 * s6 - 0.152377552205917e-1 * s7 - 0.195593427342862e-2 * s8) * s9 + (-0.433590063316381e-3 * s10 - 0.146112803263678e-3 * s11 - 0.319022339097685e-2 * s12 - 0.477063086307787e-3 * s13) * s14 + (-0.276114805649180e-2 * s15 - 0.933166016624500e-3 * s16 - 0.207984584912892e-1 * s17 - 0.314360556336114e-2 * s18) * s19 + (0.172746997599710e-3 * s20 + 0.586775450031145e-4 * s21 + 0.136190009033518e-2 * s22 + 0.211410172315387e-3 * s23) * s24;
  func_dureturn = du;
function func_dureturn = func_du(x, t)
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s24 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s24 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s23 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s23 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s22 = 0.1732050808e1;
  else
    s22 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s21 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s21 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s20 = 0.1732050808e1;
  else
    s20 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s19 = 0.1732050808e1;
  else
    s19 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s18 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s18 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s17 = 0.1732050808e1;
  else
    s17 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s16 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s16 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s15 = 0.1732050808e1;
  else
    s15 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s14 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s14 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s13 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s13 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s12 = 0.1732050808e1;
  else
    s12 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s11 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s11 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s10 = 0.1732050808e1;
  else
    s10 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s9 = 0.1732050808e1;
  else
    s9 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s8 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s8 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s7 = 0.1732050808e1;
  else
    s7 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s6 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s6 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s5 = 0.1732050808e1;
  else
    s5 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s4 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s4 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s3 = 0.1732050808e1;
  else
    s3 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s2 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s2 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s1 = 0.1732050808e1;
  else
    s1 = 0.0e0;
  end
  du = -0.2500000000e0 + 0.1e1 / (0.1e1 + exp(x)) ^ 2 + 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + x * (0.1776705118e0 + 0.1e1 / (0.1e1 + exp(0.1e1 - 0.5e1 * t)) ^ 2 - 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + 0.415431679756514e-1 * s1 + 0.922094377856479e-2 * s2 + 0.603742508215732e-1 * s3 - 0.399645630498528e-2 * s4) + (-0.243051684581302e-2 * s5 - 0.809061198761621e-3 * s6 - 0.152377552205917e-1 * s7 - 0.195593427342862e-2 * s8) * s9 + (-0.433590063316381e-3 * s10 - 0.146112803263678e-3 * s11 - 0.319022339097685e-2 * s12 - 0.477063086307787e-3 * s13) * s14 + (-0.276114805649180e-2 * s15 - 0.933166016624500e-3 * s16 - 0.207984584912892e-1 * s17 - 0.314360556336114e-2 * s18) * s19 + (0.172746997599710e-3 * s20 + 0.586775450031145e-4 * s21 + 0.136190009033518e-2 * s22 + 0.211410172315387e-3 * s23) * s24;
  func_dureturn = du;
function func_dureturn = func_du(x, t)
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s24 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s24 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s23 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s23 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s22 = 0.1732050808e1;
  else
    s22 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s21 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s21 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s20 = 0.1732050808e1;
  else
    s20 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s19 = 0.1732050808e1;
  else
    s19 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s18 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s18 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s17 = 0.1732050808e1;
  else
    s17 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s16 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s16 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s15 = 0.1732050808e1;
  else
    s15 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s14 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s14 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s13 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s13 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s12 = 0.1732050808e1;
  else
    s12 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s11 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s11 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s10 = 0.1732050808e1;
  else
    s10 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s9 = 0.1732050808e1;
  else
    s9 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s8 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s8 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s7 = 0.1732050808e1;
  else
    s7 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s6 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s6 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s5 = 0.1732050808e1;
  else
    s5 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s4 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s4 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s3 = 0.1732050808e1;
  else
    s3 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s2 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s2 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s1 = 0.1732050808e1;
  else
    s1 = 0.0e0;
  end
  du = -0.2500000000e0 + 0.1e1 / (0.1e1 + exp(x)) ^ 2 + 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + x * (0.1776705118e0 + 0.1e1 / (0.1e1 + exp(0.1e1 - 0.5e1 * t)) ^ 2 - 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + 0.415431679756514e-1 * s1 + 0.922094377856479e-2 * s2 + 0.603742508215732e-1 * s3 - 0.399645630498528e-2 * s4) + (-0.243051684581302e-2 * s5 - 0.809061198761621e-3 * s6 - 0.152377552205917e-1 * s7 - 0.195593427342862e-2 * s8) * s9 + (-0.433590063316381e-3 * s10 - 0.146112803263678e-3 * s11 - 0.319022339097685e-2 * s12 - 0.477063086307787e-3 * s13) * s14 + (-0.276114805649180e-2 * s15 - 0.933166016624500e-3 * s16 - 0.207984584912892e-1 * s17 - 0.314360556336114e-2 * s18) * s19 + (0.172746997599710e-3 * s20 + 0.586775450031145e-4 * s21 + 0.136190009033518e-2 * s22 + 0.211410172315387e-3 * s23) * s24;
  func_dureturn = du;
function func_dureturn = func_du(x, t)
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s24 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s24 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s23 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s23 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s22 = 0.1732050808e1;
  else
    s22 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s21 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s21 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s20 = 0.1732050808e1;
  else
    s20 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s19 = 0.1732050808e1;
  else
    s19 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s18 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s18 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s17 = 0.1732050808e1;
  else
    s17 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s16 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s16 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s15 = 0.1732050808e1;
  else
    s15 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s14 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s14 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s13 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s13 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s12 = 0.1732050808e1;
  else
    s12 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s11 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s11 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s10 = 0.1732050808e1;
  else
    s10 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s9 = 0.1732050808e1;
  else
    s9 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s8 = -0.2323790008e2 + 0.3098386677e2 * x;
  else
    s8 = 0.0e0;
  end
  if (0.5000000000e0 <= x && x <= 0.1e1)
    s7 = 0.1732050808e1;
  else
    s7 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s6 = -0.7745966692e1 + 0.3098386677e2 * x;
  else
    s6 = 0.0e0;
  end
  if (0.0e0 <= x && x <= 0.5000000000e0)
    s5 = 0.1732050808e1;
  else
    s5 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s4 = -0.2323790008e2 + 0.3098386677e2 * t;
  else
    s4 = 0.0e0;
  end
  if (0.5000000000e0 <= t && t <= 0.1e1)
    s3 = 0.1732050808e1;
  else
    s3 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s2 = -0.7745966692e1 + 0.3098386677e2 * t;
  else
    s2 = 0.0e0;
  end
  if (0.0e0 <= t && t <= 0.5000000000e0)
    s1 = 0.1732050808e1;
  else
    s1 = 0.0e0;
  end
  du = -0.2500000000e0 + 0.1e1 / (0.1e1 + exp(x)) ^ 2 + 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + x * (0.1776705118e0 + 0.1e1 / (0.1e1 + exp(0.1e1 - 0.5e1 * t)) ^ 2 - 0.1e1 / (0.1e1 + exp(-0.5e1 * t)) ^ 2 + 0.415431679756514e-1 * s1 + 0.922094377856479e-2 * s2 + 0.603742508215732e-1 * s3 - 0.399645630498528e-2 * s4) + (-0.243051684581302e-2 * s5 - 0.809061198761621e-3 * s6 - 0.152377552205917e-1 * s7 - 0.195593427342862e-2 * s8) * s9 + (-0.433590063316381e-3 * s10 - 0.146112803263678e-3 * s11 - 0.319022339097685e-2 * s12 - 0.477063086307787e-3 * s13) * s14 + (-0.276114805649180e-2 * s15 - 0.933166016624500e-3 * s16 - 0.207984584912892e-1 * s17 - 0.314360556336114e-2 * s18) * s19 + (0.172746997599710e-3 * s20 + 0.586775450031145e-4 * s21 + 0.136190009033518e-2 * s22 + 0.211410172315387e-3 * s23) * s24;
  func_dureturn = du;