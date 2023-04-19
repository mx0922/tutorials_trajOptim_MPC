function S = getPolyCoeff(T, p0, v0, a0, pm, p1, v1, a1)

[S.a,S.b,S.c,S.d,S.e,S.f,S.g] = autoGen_sixOrderPolyCoeff(T,p0,v0,a0,pm,p1,v1,a1);

end