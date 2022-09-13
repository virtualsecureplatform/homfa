#include "archive.hpp"
#include "tfhepp_util.hpp"

#include <tbb/parallel_for.h>
#include <CLI/CLI.hpp>

#include <filesystem>
#include <optional>

namespace {
const uint32_t one_over_eight = (1u << 29), minus_one_over_eight = -(1u << 29);
}

class Weight {
private:
    std::array<TRLWELvl1, 2> w;

public:
    Weight()
    {
        for (auto&& c : w)
            c = trivial_TRLWELvl1_zero();
    }

    size_t size() const
    {
        return w.size();
    }

    TRLWELvl1& at(size_t i)
    {
        return w.at(i);
    }

    TRLWELvl1& res()
    {
        return w.at(0);
    }

    TRLWELvl1& counter(size_t i)
    {
        return w.at(1 + i);
    }

    const TRLWELvl1& at(size_t i) const
    {
        return w.at(i);
    }

    const TRLWELvl1& res() const
    {
        return w.at(0);
    }

    const TRLWELvl1& counter(size_t i) const
    {
        return w.at(1 + i);
    }

    std::tuple<bool, uint64_t, uint64_t> decrypt(const SecretKey& key,
                                                 size_t log2_num_states) const
    {
        bool accept = false;
        uint64_t res = 0;
        {
            std::array<bool, Lvl1::n> poly =
                TFHEpp::trlweSymDecrypt<Lvl1>(this->res(), key.key.lvl1);
            accept = poly[0];
            for (size_t i = 0; i < log2_num_states; i++)
                if (poly[i + 1])
                    res |= (1u << i);
        }

        uint64_t cnt = 0;
        {
            std::array<bool, Lvl1::n> poly =
                TFHEpp::trlweSymDecrypt<Lvl1>(this->counter(0), key.key.lvl1);
            for (bool b : poly)
                std::cerr << b;
            for (; cnt < Lvl1::n; cnt++)
                if (poly[cnt])
                    break;
            cnt = Lvl1::n - cnt;
        }

        return std::make_tuple(accept, res, cnt);
    }
};

Weight alter_weight(const Weight& src, bool inc_res, bool reset_counter,
                    bool inc_counter, const TRLWELvl1& counter_zero)
{
    Weight ret{src};
    if (inc_res)
        TRLWELvl1_add(ret.res(), trivial_TRLWELvl1_1over8());
    if (reset_counter)
        ret.counter(0) = counter_zero;
    if (inc_counter)
        TRLWELvl1_mult_X_k(ret.counter(0), src.counter(0), 1);
    return ret;
}

void cmux(Weight& out, const TRGSWLvl1FFT& sel, const Weight& in1,
          const Weight& in0)
{
    assert(out.size() == in1.size());
    assert(out.size() == in0.size());

    for (size_t i = 0; i < out.size(); i++)
        TFHEpp::CMUXFFT<Lvl1>(out.at(i), sel, in1.at(i), in0.at(i));
}

void lookup_table(std::vector<Weight>& table,
                  std::vector<TRGSWLvl1FFT>::const_iterator input_begin,
                  std::vector<TRGSWLvl1FFT>::const_iterator input_end,
                  std::vector<Weight>& workspace)
{
    const size_t input_size = std::distance(input_begin, input_end);
    assert(table.size() == (1 << input_size));  // FIXME: relax this condition
    if (input_size == 0)
        return;

    std::vector<Weight>& tmp = workspace;
    tmp.clear();
    tmp.resize(1 << (input_size - 1));

    size_t i = 0;
    for (auto it = input_begin; it != input_end; ++it, ++i) {
        for (size_t j = 0; j < 1 << (input_size - i - 1); j++) {
            cmux(tmp.at(j), *it, table.at(j * 2 + 1), table.at(j * 2));
        }
        // tbb::parallel_for(0, 1 << (input_size - i - 1), [&](size_t j) {
        //     TFHEpp::CMUXFFT<Lvl1>(tmp.at(j), *it, table.at(j * 2 + 1),
        //                           table.at(j * 2));
        // });
        tmp.swap(table);
    }
}

void f(Weight& state, const TRGSWLvl1FFT& raw_input,
       const TRLWELvl1& counter_zero, const GateKey& gate_key,
       const CircuitKey& circuit_key)
{
    std::cerr << ".";

    const size_t num_states = 9, log2_num_states = 4;

    std::vector<Weight> weight;
    weight.resize(num_states);
    for (size_t i = 1; i <= 3; i++) {
        // Store state #
        for (size_t j = 0; j < 4; j++)
            if (((i >> j) & 1u) == 0)
                weight.at(i).res()[1][j + 1] = -(1u << 29);  // -1/8
            else
                weight.at(i).res()[1][j + 1] = (1u << 29);  // 1/8

        // Store counter
        weight.at(i).counter(0) = state.counter(0);
    }

    TRGSWLvl1FFT counter_bootstrapped;
    {
        TLWELvl1 t1;
        TFHEpp::SampleExtractIndex<Lvl1>(t1, state.counter(0), 0);
        TLWELvl0 t0;
        TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(t0, t1, gate_key.ksk);
        CircuitBootstrappingFFTLvl01(counter_bootstrapped, t0, circuit_key);
    }

    std::vector<TRGSWLvl1FFT> inputs = {
        // FIXME CB to countre
        counter_bootstrapped,
        raw_input,
    };

    std::vector<std::tuple<size_t, bool, bool, bool>> delta = {
        /* state input dst inc_res reset_counter inc_counter */
        /* 0 0 */ {4, false, false, false},
        /* 0 1 */ {4, false, false, false},
        /* 1 0 */ {5, false, false, false},
        /* 1 1 */ {5, false, false, false},
        /* 2 0 */ {7, false, false, false},
        /* 2 1 */ {6, false, false, false},
        /* 3 0 */ {8, false, false, false},
        /* 3 1 */ {8, false, false, false},
        /* 4 0 */ {3, false, false, false},
        /* 4 1 */ {1, true, false, false},
        /* 5 0 */ {2, true, true, false},
        /* 5 1 */ {1, true, false, false},
        /* 6 0 */ {3, false, false, false},
        /* 6 1 */ {1, true, false, false},
        /* 7 0 */ {2, true, false, true},
        /* 7 1 */ {1, true, false, false},
        /* 8 0 */ {3, false, false, false},
        /* 8 1 */ {1, true, false, false},
    };

    std::vector<Weight> temp;
    temp.resize(weight.size());
    for (int i = inputs.size() - 1; i >= 0; i--) {
        auto&& input = inputs.at(i);
        for (size_t i = 0; i < num_states; i++) {
            using std::get;

            Weight& out = temp.at(i);
            auto&& d0 = delta.at(i * 2 + 0);
            auto&& d1 = delta.at(i * 2 + 1);
            Weight in0 = alter_weight(weight.at(get<0>(d0)), get<1>(d0),
                                      get<2>(d0), get<3>(d0), counter_zero);
            Weight in1 = alter_weight(weight.at(get<0>(d1)), get<1>(d1),
                                      get<2>(d1), get<3>(d1), counter_zero);
            cmux(out, input, in1, in0);
        }
        weight.swap(temp);
    }

    std::vector<TRGSWLvl1FFT> cond(log2_num_states);
    for (size_t i = 0; i < log2_num_states; i++) {
        TLWELvl1 t1;
        TFHEpp::SampleExtractIndex<Lvl1>(t1, state.res(), i + 1);
        TLWELvl0 t0;
        TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(t0, t1, gate_key.ksk);
        CircuitBootstrappingFFTLvl01(cond.at(i), t0, circuit_key);
    }
    /*
    tbb::parallel_for(0, log2_num_states, [&](size_t i) {
        TLWELvl1 t1;
        TFHEpp::SampleExtractIndex<Lvl1>(t1, state.res(), i + 1);
        TLWELvl0 t0;
        TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(t0, t1, gate_key.ksk);
        CircuitBootstrappingFFTLvl01(cond.at(i), t0, circuit_key);
    });
    */

    weight.resize(1 << log2_num_states);
    lookup_table(weight, cond.begin(), cond.end(), temp);
    state = weight.at(0);
}

void test(const SecretKey& skey, const BKey& bkey)
{
    const size_t alpha = 100;

    TRGSWLvl1FFT b1 = encrypt_bit_to_TRGSWLvl1FFT(true, skey);
    TRGSWLvl1FFT b0 = encrypt_bit_to_TRGSWLvl1FFT(false, skey);

    TRLWELvl1 counter_zero;
    {
        PolyLvl1 poly;
        for (size_t i = 0; i < Lvl1::n; i++) {
            poly[i] =
                (i < Lvl1::n - alpha) ? minus_one_over_eight : one_over_eight;
        }
        counter_zero = trivial_TRLWELvl1(poly);
    }

    Weight state;
    state.counter(0) = counter_zero;
    {
        PolyLvl1 poly;
        for (size_t i = 0; i < Lvl1::n; i++)
            poly[i] = minus_one_over_eight;
        state.res() = trivial_TRLWELvl1(poly);
    }

    // 0 -> 4 -> 1
    f(state, b1, counter_zero, *bkey.gkey, *bkey.circuit_key);
    // 1 -> 5 -> 2
    f(state, b0, counter_zero, *bkey.gkey, *bkey.circuit_key);
    // 2 -> 7 -> 2 -> 7 -> 2 -> ... -> 2
    for (int i = 0; i < 98; i++) {
        f(state, b0, counter_zero, *bkey.gkey, *bkey.circuit_key);

        auto [accept, res, cnt] = state.decrypt(skey, 4);
        std::cout << "\n"
                  << "acc: " << accept << "\n"
                  << "res: " << res << "\n"
                  << "cnt: " << cnt << "\n";
    }
}

int main(int argc, char** argv)
{
    CLI::App app{"Benchmark runner"};
    // app.require_subcommand();
    CLI11_PARSE(app, argc, argv);

    std::optional<SecretKey> skey;
    std::optional<BKey> bkey;

    const std::string skey_filename = "_skey", bkey_filename = "_bkey";
    bool skey_exists = std::filesystem::is_regular_file(skey_filename);
    bool bkey_exists = std::filesystem::is_regular_file(bkey_filename);
    if (skey_exists && bkey_exists) {
        skey.emplace(read_from_archive<SecretKey>(skey_filename));
        bkey.emplace(read_from_archive<BKey>(bkey_filename));
    }
    else {
        skey.emplace();
        bkey.emplace(*skey);
        write_to_archive(skey_filename, *skey);
        write_to_archive(bkey_filename, *bkey);
    }

    test(*skey, *bkey);
    return 0;
}
