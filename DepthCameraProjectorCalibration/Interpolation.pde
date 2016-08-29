int cubic(int[] data, float x, float y, int w, int mask) {
  int i = (int)x;
  int j = (int)y;
  float u = x - i;
  float v = y - j;
  Jama.Matrix A, B, C;
  A = new Jama.Matrix(new double[][]{{S(1 + v), S(v), S(1 - v), S(2 - v)}});
  double[][] bArr = new double[4][4];
  for (int ox = -1; ox < 3; ++ox)
    for (int oy = -1; oy < 3; ++oy)
      bArr[1 + ox][1 + oy] = data[(j + oy) * w + (i + ox)] & mask;  
  B = new Jama.Matrix(bArr);
  C = new Jama.Matrix(new double[][]{{S(1 + u)}, {S(u)}, {S(1 - u)}, {S(2 - u)}});
  return ((int)A.times(B).times(C).get(0, 0)) & mask;
}

float S(float x) {
  float abx = x > 0 ? x : -x;
  if (abx >= 2) return 0;
  if (abx >= 1) 
    return 4 + abx * (-8 + abx * (5 - abx));
  return 1 + abx * abx * (-2 + abx);
}