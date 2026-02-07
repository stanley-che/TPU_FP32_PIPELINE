/*
g++ -O2 exp_lut_1over128_gen.cpp -o gen_exp_lut
./gen_exp_lut
*/
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cstring>   // ← 一定要有

static uint32_t f2u(float f) {
    uint32_t u;
    std::memcpy(&u, &f, sizeof(u));  // bit-exact FP32
    return u;
}

int main() {
    const int STEP_INV = 64;
    const int XMAX_Q   = 8;
    const int N        = XMAX_Q * STEP_INV + 1; // 513

    std::ofstream ofs("./src/EPU/attention_score/exp_lut.memh");
    if (!ofs) {
        std::cerr << "Failed to open output file\n";
        return 1;
    }

    ofs << std::hex << std::setfill('0');

    for (int k = 0; k < N; ++k) {
        float x = -static_cast<float>(k) / STEP_INV; // 0, -1/64, ..., -8
        float y = std::exp(x);
        uint32_t bits = f2u(y);

        ofs << std::setw(8) << bits << "\n";
    }

    ofs.close();
    return 0;
}
