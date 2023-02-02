BLA::Matrix<12, 1> integral;    // accumulated error integral from controller
BLA::Matrix<6, 1> prevFb;  // previous wrench error
long prevt;                     // timestamp of previous wrench error

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
  G.Submatrix<3, 1>(3, 12) = getSkew(x[0], x[1], x[2]) * zhat;
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

// Determine current foot position vector
BLA::Matrix<12, 1> getVf() {
  BLA::Matrix<12, 1> Vf;
  for (int i = 0; i < 4; i++) {
    float *v = getFootVel(i + 1);
    for (int j = 0; j < 3; j++) {
      Vf(i * 3 + j) = v[j];
    }
  }
  return (Vf);
}

// Determine current foot position vector
BLA::Matrix<13, 1> getVftail() {
  BLA::Matrix<13, 1> Vf;
  for (int i = 0; i < 4; i++) {
    float *v = getFootVel(i + 1);
    for (int j = 0; j < 3; j++) {
      Vf(i * 3 + j) = v[j];
    }
  }
  Vf(12) = getTailVel()[2];
  return (Vf);
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

// Directly move motors to their setpoints
void noControl() {
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 3; j++) {
      goal_angles[i][j] = setpoints[i][j];
      // goal_torques[i][j] = motor_torque;
    }
  }
}

// Maintain current position using pseudo-inverse impedance control strategy, including tail
void impedanceControlTail() {
  BLA::Matrix<6, 13> G;
  BLA::Matrix<13, 6> Ginv;
  BLA::Matrix<6, 1> Xb, Fb, Vb, dFb;
  BLA::Matrix<6, 1> k = {1, 1, 1, 1, 1, 1};
  BLA::Matrix<13, 1> Xf, Ff, Xf0, Vf;
  BLA::Matrix<13, 1> w = {foot_weights[0], foot_weights[0], foot_weights[0],
                          foot_weights[1], foot_weights[1], foot_weights[1],
                          foot_weights[2], foot_weights[2], foot_weights[2],
                          foot_weights[3], foot_weights[3], foot_weights[3], foot_weights[4]};
  BLA::Matrix<13, 6> temp;
  // float kP = 1, kD = 0, kI = 0;
  G = getGtail(); // 2 ms
  Xf = getXftail(); // 1 ms
  Vf = getVftail(); // ?
  Xf0 = getXf0tail();
  for (int i = 0; i < 13; i++) {
    Xf0(i) = Xf0(i) * w(i) + Xf(i) * (1-w(i));
    Vf(i) = Vf(i) * w(i);
  }
  Ginv = (~G) * BLA::Inverse(G * (~G)); // ?
  //  Xb = ~pinv(G) * (Xf - Xf0); // 2 ms
  // Xb = ~((~G) * BLA::Inverse(G * (~G))) * (Xf - Xf0); // 2 ms
  Xb = ~Ginv * (Xf - Xf0); // ?
  Vb = ~Ginv * Vf; // ?
  Fb = -vscale(k, Xb);
  dFb = -vscale(k, Vb);
  // dFb = (Fb - prevFb)*1000/(millis() - prevt);
  prevt = millis();
  prevFb = Fb;
  integral.Submatrix<6, 1>(0, 0) += scale(Fb, dt) / 1000.0;
  temp = pinv(hscale(G, w)); // 3 ms
  Ff = vscale(w, temp) * (scale(Fb, kP) + scale(dFb, kD) + scale(integral.Submatrix<6, 1>(0, 0), kI));
  // Ff(0) += 10; // inward grasping test
  // Ff(3) -= 10;
  for (int i = 0; i < 4; i++) {
    if (foot_weights[i] > 0) {
      setFootForce(i + 1, Ff(3 * i), Ff(3 * i + 1), Ff(3 * i + 2)); // 1 ms (total)
    } else {
      for (int j = 0; j < 3; j++) {
        goal_angles[i][j] = setpoints[i][j];
      }
    }
  }
  setTailForce(Ff(12));
  // Serial << "P: " << scale(Fb, kP) << ", D: " << scale(dFb, kD) << "\n";
  // Serial << Ff << "\n";
  // Serial << int(millis() - t0) << "  |  " << Fb << "  |   " << integral.Submatrix<6, 1>(0, 0) << "  |   " << Ff << "\n";
}

// Maintain current position using pseudo-inverse impedance control strategy
void impedanceControlPseudoInverse() {
  BLA::Matrix<6, 12> G;
  BLA::Matrix<6, 1> Xb, Fb;
  BLA::Matrix<6, 1> k = {1, 1, 1, 1, 1, 1};
  BLA::Matrix<12, 1> Xf, Ff, Xf0;
  BLA::Matrix<12, 1> w = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  BLA::Matrix<12, 6> temp;
  // float kP = 1, kI = 0;
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
  // Serial << int(millis() - t0) << "  |  " << Fb << "  |   " << integral.Submatrix<6, 1>(0, 0) << "  |   " << Ff << "\n";
}

// Maintain current position using impedance control strategy
void impedanceControl() {
  BLA::Matrix<12, 12> G, Ginv;
  BLA::Matrix<12, 1> Xb, Fb;
  BLA::Matrix<12, 1> k = {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0};
  BLA::Matrix<12, 1> Xf, Ff, Xf0;
  // float kP = 10, kI = 0;
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
  // Serial << int(millis() - t0) << "  |  " << Fb << "  |   " << integral << "  |   " << Ff << "\n";
}

// Maintain current position using decentralized impedance control strategy
void impedanceControlDecentralized() {
  BLA::Matrix<12, 1> k = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  BLA::Matrix<12, 1> Xf, Ff, Xf0, Vf;
  // float kP = 0.1, kI = 0;
  Xf = getXf();
  Xf0 = getXf0();
  Vf = getVf();
  Xf = vscale(k, Xf - Xf0);
  Vf = vscale(k, Vf);
  integral += scale(Xf, dt) / 1000.0;
  Ff = -(scale(Xf, kP) + scale(Vf, kD) + scale(integral, kI));
  for (int i = 0; i < 4; i++) {
    setFootForce(i + 1, Ff(3 * i), Ff(3 * i + 1), Ff(3 * i + 2)); // 16 ms (total)
  }
  // Serial << scale(Vf, kD) << "\n";
  // Serial << int(millis() - t0) << "  |  " << Xf << "  |   " << integral << "  |   " << Ff << "\n";
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

// Recenter body to minimize twist
void recenter(int step) {
  BLA::Matrix<6, 12> G;
  BLA::Matrix<6, 13> Gt;
  BLA::Matrix<12, 6> Ginv;
  BLA::Matrix<6, 1> Xb;
  BLA::Matrix<12, 1> Xf, Xf0;
  BLA::Matrix<13, 1> Yf;
  Gt = getGtail(); // 2 ms
  G = Gt.Submatrix<6, 12>(0, 0);
  Xf = getXf0(); // 1 ms
  Xf0 = getNominalStance(step);

  // float zmax = -1000;
  // float zmean = (Xf(2) + Xf(5) + Xf(8) + Xf(11))/4;
  // for (int i = 0; i < 4; i++) {
  //   zmax = max(zmax, Xf(i*3+2));
  // }
  // float offset = zmax -zmean;
  // Serial << zmax << "\n";
  // Serial << zmean << "\n";
  // for (int i = 0; i < 4; i++) {
  //   Xf0(i*3+2) -= offset;
  // }
  // Xf0(12) -= offset;

  Ginv = (~G) * BLA::Inverse(G * (~G));
  Xb = ~Ginv * (Xf0 - Xf);
  Yf = (~Gt) * Xb;

  float dmax = 0;
  for (int i = 0; i < 4; i++) {
    float d = sqrt(Yf(3*i)*Yf(3*i) + Yf(3*i+1)*Yf(3*i+1) + Yf(3*i+2)*Yf(3*i+2));
    if (d > dmax) dmax = d;
  }
  for (int i = 0; i < 4; i++) {
    float d = sqrt(Yf(3*i)*Yf(3*i) + Yf(3*i+1)*Yf(3*i+1) + Yf(3*i+2)*Yf(3*i+2));
    float v = min(40*d/dmax, d*1000/dt);
    moveFootInDirection(Yf(3*i), Yf(3*i+1), Yf(3*i+2), v, i+1);
  }
  // float vt = min(40*abs(Yf(12))/dmax, abs(Yf(12))*1000/dt);
  // moveTailInDirection(Yf(12), vt);
}