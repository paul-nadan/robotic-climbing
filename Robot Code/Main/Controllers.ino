// Convert a vector to a skew-symmetric matrix
BLA::Matrix<3, 3> getSkew(float x, float y, float z) {
  BLA::Matrix<3, 3> skew = {0, -z, y, z, 0, -x, -y, x, 0};
  return (skew);
}

// Compute the grasp matrix
BLA::Matrix<12, 12> getG() {
  BLA::Matrix<12, 12> G;
  G.Fill(0);
  BLA::Matrix<1, 3> X;
  int row = 6;
  for (int i = 0; i < 4; i++) {
    G.Submatrix<3, 3>(0, 3 * i) = Identity<3, 3>();
    float *x = getFootPos(i + 1);
    X = {x[0], x[1], x[2]};
    G.Submatrix<3, 3>(3, 3 * i) = getSkew(x[0], x[1], x[2]);
    for (int j = i + 1; j < 4; j++) {
      G.Submatrix<1, 3>(row, 3 * i) += X;
      G.Submatrix<1, 3>(row, 3 * j) -= X;
      row++;
    }
  }
  return (G);
}

// Compute the grasp matrix
BLA::Matrix<6, 13> getGtail() {
  BLA::Matrix<6, 13> G;
  for (int i = 0; i < 4; i++) {
    G.Submatrix<3, 3>(0, 3 * i) = Identity<3, 3>();
    float *x = getFootPos(i + 1);
    G.Submatrix<3, 3>(3, 3 * i) = getSkew(x[0], x[1], x[2]);
  }
  float *x = getTailPos();
  BLA::Matrix<3, 1> zhat = {0, 0, 1};
  G.Submatrix<3, 1>(0, 12) = zhat;
  G.Submatrix<3, 1>(3, 12) = getSkew(x[0], x[1], x[2])*zhat;
  return (G);
}

// Compute the grasp matrix
BLA::Matrix<6, 12> getGpsuedo() {
  BLA::Matrix<6, 12> G;
  for (int i = 0; i < 4; i++) {
    G.Submatrix<3, 3>(0, 3 * i) = Identity<3, 3>();
    float *x = getFootPos(i + 1);
    G.Submatrix<3, 3>(3, 3 * i) = getSkew(x[0], x[1], x[2]);
  }
  return (G);
}

// Determine current foot position vector
BLA::Matrix<12, 1> getXf() {
  BLA::Matrix<12, 1> Xf;
  for (int i = 0; i < 4; i++) {
    float *x = getFootPos(i + 1);
    for (int j = 0; j < 3; j++) {
      Xf(i * 3 + j) = x[j];
    }
  }
  return (Xf);
}

// Determine current foot position setpoint vector
BLA::Matrix<12, 1> getXf0() {
  BLA::Matrix<12, 1> Xf;
  for (int i = 0; i < 4; i++) {
    float *x = getFootPosGoal(i + 1);
    for (int j = 0; j < 3; j++) {
      Xf(i * 3 + j) = x[j];
    }
  }
  return (Xf);
}

// Determine current foot position vector
BLA::Matrix<13, 1> getXftail() {
  BLA::Matrix<13, 1> Xf;
  for (int i = 0; i < 4; i++) {
    float *x = getFootPos(i + 1);
    for (int j = 0; j < 3; j++) {
      Xf(i * 3 + j) = x[j];
    }
  }
  Xf(12) = getTailPos()[2];
  return (Xf);
}

// Determine current foot position setpoint vector
BLA::Matrix<13, 1> getXf0tail() {
  BLA::Matrix<13, 1> Xf;
  for (int i = 0; i < 4; i++) {
    float *x = getFootPosGoal(i + 1);
    for (int j = 0; j < 3; j++) {
      Xf(i * 3 + j) = x[j];
    }
  }
  Xf(12) = getTailPosGoal()[2];
  return (Xf);
}

// Maintain current position using pseudo-inverse impedance control strategy, including tail
void impedanceControlTail() {
  BLA::Matrix<6, 13> G;
  BLA::Matrix<6, 1> Xb, Fb;
  BLA::Matrix<6, 1> k = {1, 1, 1, 1, 1, 1};
  BLA::Matrix<13, 1> Xf, Ff, Xf0;
  BLA::Matrix<13, 1> w = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  BLA::Matrix<13, 6> temp;
  float kP = .5, kD = 0, kI = 0;
  G = getGtail();
  Xf = getXftail();
  Xf0 = getXf0tail();
//  Xb = ~pinv(G) * (Xf - Xf0); // 2 ms
  Xb = ~((~G) * BLA::Inverse(G * (~G))) * (Xf - Xf0); // 2 ms
  Fb = -vscale(k, Xb);
  integral.Submatrix<6, 1>(0, 0) += scale(Fb, dt) / 1000.0;
  temp = pinv(hscale(G, w));
  Ff = vscale(w, temp) * (scale(Fb, kP) + scale(integral.Submatrix<6, 1>(0, 0), kI));
  for (int i = 0; i < 4; i++) {
    limitFootForce(i + 1, Ff(3 * i), Ff(3 * i + 1), Ff(3 * i + 2)); // 16 ms (total)
  }
  limitTailForce(Ff(12));
  Serial << int(millis() - t0) << "  |  " << Fb << "  |   " << integral.Submatrix<6, 1>(0, 0) << "  |   " << Ff << "\n";
}

// Maintain current position using pseudo-inverse impedance control strategy
void impedanceControlPseudoInverse() {
  BLA::Matrix<6, 12> G;
  BLA::Matrix<6, 1> Xb, Fb;
  BLA::Matrix<6, 1> k = {1, 1, 1, 1, 1, 1};
  BLA::Matrix<12, 1> Xf, Ff, Xf0;
  BLA::Matrix<12, 1> w = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  BLA::Matrix<12, 6> temp;
  float kP = 1, kI = 0;
  G = getGpsuedo();
  Xf = getXf();
  Xf0 = getXf0();
//  Xb = ~pinv(G) * (Xf - Xf0); // 2 ms
  Xb = ~((~G) * BLA::Inverse(G * (~G))) * (Xf - Xf0); // 2 ms
  Fb = -vscale(k, Xb);
  integral.Submatrix<6, 1>(0, 0) += scale(Fb, dt) / 1000.0;
  temp = pinv(hscale(G, w));
  Ff = vscale(w, temp) * (scale(Fb, kP) + scale(integral.Submatrix<6, 1>(0, 0), kI));
  for (int i = 0; i < 4; i++) {
    limitFootForce(i + 1, Ff(3 * i), Ff(3 * i + 1), Ff(3 * i + 2)); // 16 ms (total)
  }
  Serial << int(millis() - t0) << "  |  " << Fb << "  |   " << integral.Submatrix<6, 1>(0, 0) << "  |   " << Ff << "\n";
}

// Maintain current position using impedance control strategy
void impedanceControl() {
  BLA::Matrix<12, 12> G, Ginv;
  BLA::Matrix<12, 1> Xb, Fb;
  BLA::Matrix<12, 1> k = {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0};
  BLA::Matrix<12, 1> Xf, Ff, Xf0;
  float kP = 10, kI = 0;
  G = getG();
  Ginv = BLA::Inverse(G); // 5 ms
  Xf = getXf();
  Xf0 = getXf0();
  Xb = ~Ginv * (Xf - Xf0);
  Fb = -vscale(k, Xb);
  integral += scale(Fb, dt) / 1000.0;
  Ff = Ginv * (scale(Fb, kP) + scale(integral, kI));
  for (int i = 0; i < 4; i++) {
    limitFootForce(i + 1, Ff(3 * i), Ff(3 * i + 1), Ff(3 * i + 2)); // 16 ms (total)
  }
  Serial << int(millis() - t0) << "  |  " << Fb << "  |   " << integral << "  |   " << Ff << "\n";
}

// Maintain current position using decentralized impedance control strategy
void impedanceControlDecentralized() {
  BLA::Matrix<12, 1> k = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  BLA::Matrix<12, 1> Xf, Ff, Xf0;
  float kP = 0.1, kI = 0;
  Xf = getXf();
  Xf0 = getXf0();
  Xf = vscale(k, Xf - Xf0);
  integral += scale(Xf, dt) / 1000.0;
  Ff = -(scale(Xf, kP) + scale(integral, kI));
  for (int i = 0; i < 4; i++) {
    limitFootForce(i + 1, Ff(3 * i), Ff(3 * i + 1), Ff(3 * i + 2)); // 16 ms (total)
  }
  Serial << int(millis() - t0) << "  |  " << Xf << "  |   " << integral << "  |   " << Ff << "\n";
}

// Compute the pseudo-inverse of a matrix
template <int rows, int cols, class MemT>
BLA::Matrix<cols, rows, MemT> pinv(BLA::Matrix<rows, cols, MemT> A) {
  if (rows > cols) return (BLA::Inverse((~A) * A) * (~A));
  else return ((~A) * BLA::Inverse(A * (~A)));
}

// Convert vector to diagonal matrix
template <int rows, class MemT>
BLA::Matrix<rows, rows, MemT> diag(BLA::Matrix<rows, 1, MemT> V) {
  BLA::Matrix<rows, rows, MemT> A = BLA::Matrix<rows, rows>();
  for (int i = 0; i < rows; i++) {
    A(i, i) = V(i);
  }
  return (A);
}

// Multiply matrix by a scalar
template <int rows, int cols, class MemT>
BLA::Matrix<rows, cols, MemT> scale(BLA::Matrix<rows, cols, MemT> A, float s) {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      A(i, j) *= s;
    }
  }
  return (A);
}

// Multiply matrix by a vector elementwise
template <int rows, int cols, class MemT>
BLA::Matrix<rows, cols, MemT> hscale(BLA::Matrix<rows, cols, MemT> A, BLA::Matrix<cols, 1> S) {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      A(i, j) *= S(j);
    }
  }
  return (A);
}

// Multiply matrix by a vector elementwise
template <int rows, int cols, class MemT>
BLA::Matrix<rows, cols, MemT> vscale(BLA::Matrix<rows, 1> S, BLA::Matrix<rows, cols, MemT> A) {
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      A(i, j) *= S(i);
    }
  }
  return (A);
}
