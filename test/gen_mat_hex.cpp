/*
g++ -O3 -std=c++17 ./test/gen_mat_hex.cpp -o gen_mat_hex
./gen_mat_hex 64   A.hex B.hex C_golden.hex eye       # A=I, C==B (最好 debug)
./gen_mat_hex 64   A.hex B.hex C_golden.hex smallint  # 小整數 pattern (bit-exact 容易)
./gen_mat_hex 64   A.hex B.hex C_golden.hex rand      # random (可能需要 tolerance 比對)
./gen_mat_hex 1024 A.hex B.hex C_golden.hex eye
*/

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

static inline uint32_t f32_to_u32(float x) {
  uint32_t u;
  std::memcpy(&u, &x, sizeof(u));
  return u;
}

// row-major index
static inline size_t idx(size_t r, size_t c, size_t T) { return r * T + c; }

static void dump_hex(const std::string& path, const std::vector<float>& M) {
  std::ofstream ofs(path);
  if (!ofs) {
    std::cerr << "Cannot open " << path << "\n";
    std::exit(1);
  }
  ofs << std::hex << std::setfill('0');
  for (size_t n = 0; n < M.size(); n++) {
    uint32_t u = f32_to_u32(M[n]);
    ofs << std::setw(8) << u << "\n";
  }
}

int main(int argc, char** argv) {
  const int T = (argc >= 2) ? std::stoi(argv[1]) : 1024;
  const std::string outA = (argc >= 3) ? argv[2] : "A.hex";
  const std::string outB = (argc >= 4) ? argv[3] : "B.hex";
  const std::string outC = (argc >= 5) ? argv[4] : "C_golden.hex";
  const std::string mode = (argc >= 6) ? argv[5] : "eye"; // eye|smallint|rand

  std::cout << "Generate T=" << T << " mode=" << mode << " -> "
            << outA << ", " << outB << ", " << outC << "\n";

  std::vector<float> A((size_t)T * T), B((size_t)T * T), C((size_t)T * T, 0.0f);

  // rng for rand/eye B
  std::mt19937 rng(123);
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);

  // --------- 1) generate A/B ----------
  if (mode == "eye") {
    // A = I, B = random => C should equal B (best for HW sanity)
    for (int r = 0; r < T; r++) {
      for (int c = 0; c < T; c++) {
        A[idx(r, c, T)] = (r == c) ? 1.0f : 0.0f;
        B[idx(r, c, T)] = dist(rng);
      }
    }
  } else if (mode == "smallint") {
    // small integers exactly representable in FP32 => easier bit-exact compare
    for (int r = 0; r < T; r++) {
      for (int c = 0; c < T; c++) {
        A[idx(r, c, T)] = float((r + c) & 3);         // 0..3
        B[idx(r, c, T)] = float(((r * 3) + c) & 3);   // 0..3
      }
    }
  } else { // "rand"
    for (int r = 0; r < T; r++) {
      for (int c = 0; c < T; c++) {
        A[idx(r, c, T)] = dist(rng);
        B[idx(r, c, T)] = dist(rng);
      }
    }
  }

  // --------- 2) golden C = A*B ----------
  // loop order i-k-j (generally decent cache reuse for B row)
  for (int i = 0; i < T; i++) {
    for (int k = 0; k < T; k++) {
      float a = A[idx(i, k, T)];
      const size_t bk = (size_t)k * T;
      const size_t ci = (size_t)i * T;
      for (int j = 0; j < T; j++) {
        C[ci + j] += a * B[bk + j];
      }
    }
  }

  // --------- 3) dump hex ----------
  dump_hex(outA, A);
  dump_hex(outB, B);
  dump_hex(outC, C);

  std::cout << "Done.\n";
  return 0;
}
