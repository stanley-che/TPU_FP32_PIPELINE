//gen_attention_hex.cpp
/*
g++ -O2 -std=c++17 -o gen_attention_hex gen_attention_hex.cpp
./gen_attention_hex 32 32 64 ./test

*/
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

// ------------------------------------------------------------
// FP32 helpers
// ------------------------------------------------------------
static uint32_t f2u(float f) {
  uint32_t u;
  std::memcpy(&u, &f, sizeof(u));
  return u;
}

// ------------------------------------------------------------
// Deterministic value generator in [-2, 2]
// Periodic, mixed-sign, no overflow
// ------------------------------------------------------------
static float gen_val(int i, int d) {
  // periodic deterministic pattern
  // range roughly [-2, 2]
  float x = std::sin(0.7f * i + 0.3f * d)
          + 0.5f * std::cos(0.2f * i - 0.6f * d);

  // normalize to [-1,1]
  x *= 0.8f;

  // scale to [-2,2]
  return 2.0f * x;
}

// ------------------------------------------------------------
// Write hex file (one 32-bit hex per line)
// ------------------------------------------------------------
static void write_hex(const std::string& path,
                      const std::vector<uint32_t>& v) {
  std::ofstream ofs(path);
  if (!ofs) {
    throw std::runtime_error("Cannot open " + path);
  }
  ofs << std::hex << std::setfill('0');
  for (uint32_t u : v) {
    ofs << std::setw(8) << u << "\n";
  }
}

// ------------------------------------------------------------
// Main
// ------------------------------------------------------------
int main(int argc, char** argv) {
  int T     = 8;   // 4,8,16,32,64
  int D_LEN = 8;
  int DMAX  = 16;
  std::string outdir = "./testvec";

  if (argc >= 2) T     = std::stoi(argv[1]);
  if (argc >= 3) D_LEN = std::stoi(argv[2]);
  if (argc >= 4) DMAX  = std::stoi(argv[3]);
  if (argc >= 5) outdir = argv[4];

  if (D_LEN > DMAX) {
    std::cerr << "[FATAL] D_LEN must be <= DMAX\n";
    return 1;
  }

  std::cout << "[gen] T=" << T
            << " D_LEN=" << D_LEN
            << " DMAX=" << DMAX
            << " outdir=" << outdir << "\n";

  // ------------------------------------------------------------
  // Allocate
  // ------------------------------------------------------------
  std::vector<float> Q(T * D_LEN);
  std::vector<float> K(T * D_LEN);
  std::vector<float> V(T * D_LEN);

  // ------------------------------------------------------------
  // Generate Q, K in [-2,2], V = Q
  // ------------------------------------------------------------
  for (int i = 0; i < T; i++) {
    for (int d = 0; d < D_LEN; d++) {
      float q = gen_val(i, d);
      float k = gen_val(i + 1, d + 3); // decorrelate

      Q[i * D_LEN + d] = q;
      K[i * D_LEN + d] = k;
      V[i * D_LEN + d] = q;            // easy verify
    }
  }

  // ------------------------------------------------------------
  // Attention reference
  // S = Q*K^T
  // Ssc = S / sqrt(D_LEN)
  // P = softmax row-wise (rowmax-sub)
  // O = P*V
  // ------------------------------------------------------------
  const float alpha = 1.0f / std::sqrt((float)D_LEN);

  std::vector<float> S(T * T, 0.0f);
  std::vector<float> P(T * T, 0.0f);
  std::vector<float> O(T * D_LEN, 0.0f);

  // Q*K^T
  for (int i = 0; i < T; i++) {
    for (int j = 0; j < T; j++) {
      float acc = 0.0f;
      for (int d = 0; d < D_LEN; d++) {
        acc += Q[i * D_LEN + d] * K[j * D_LEN + d];
      }
      S[i * T + j] = acc * alpha;
    }
  }

  // softmax
  for (int i = 0; i < T; i++) {
    float rowmax = -std::numeric_limits<float>::infinity();
    for (int j = 0; j < T; j++) {
      rowmax = std::max(rowmax, S[i * T + j]);
    }

    float sum = 0.0f;
    for (int j = 0; j < T; j++) {
      float e = std::exp(S[i * T + j] - rowmax);
      P[i * T + j] = e;
      sum += e;
    }

    float inv = 1.0f / sum;
    for (int j = 0; j < T; j++) {
      P[i * T + j] *= inv;
    }
  }

  // O = P * V
  for (int i = 0; i < T; i++) {
    for (int d = 0; d < D_LEN; d++) {
      float acc = 0.0f;
      for (int k = 0; k < T; k++) {
        acc += P[i * T + k] * V[k * D_LEN + d];
      }
      O[i * D_LEN + d] = acc;
    }
  }

  // ------------------------------------------------------------
  // Pack FP32 to hex
  // ------------------------------------------------------------
  std::vector<uint32_t> Q_hex(T * D_LEN);
  std::vector<uint32_t> K_hex(T * D_LEN);
  std::vector<uint32_t> V_hex(T * D_LEN);
  std::vector<uint32_t> O_hex(T * D_LEN);

  for (int i = 0; i < T * D_LEN; i++) {
    Q_hex[i] = f2u(Q[i]);
    K_hex[i] = f2u(K[i]);
    V_hex[i] = f2u(V[i]);
    O_hex[i] = f2u(O[i]);
  }

  try {
    write_hex(outdir + "/Q.hex", Q_hex);
    write_hex(outdir + "/K.hex", K_hex);
    write_hex(outdir + "/V.hex", V_hex);
    write_hex(outdir + "/O_golden.hex", O_hex);
  } catch (const std::exception& e) {
    std::cerr << "[FATAL] " << e.what() << "\n";
    return 1;
  }

  std::cout << "[gen] Done.\n";
  return 0;
}
