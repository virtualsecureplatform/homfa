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
    std::array<TRLWELvl1, 10 + 1> w;

private:
    void mark_res_bit(size_t i)
    {
        PolyLvl1 poly = {};
        poly[i] = (1 << 30);  // 1/4
        TRLWELvl1 t = trivial_TRLWELvl1(poly);
        TRLWELvl1_add(res(), t);
    }

public:
    Weight()
    {
        /*
           index    bit
           0
                    0    accept
                    1..  state #
                    n/2  borrow mark

           1..      0    i-th bit of the counter
                    1..  unused
        */
        res() = trivial_TRLWELvl1_zero();
        set_state_number(0);
        set_counter(0);
    }

    constexpr size_t counter_width() const
    {
        return w.size() - 1;
    }

    TRLWELvl1& counter_bit(size_t i)
    {
        return w.at(i + 1);
    }

    const TRLWELvl1& counter_bit(size_t i) const
    {
        return w.at(i + 1);
    }

    void set_state_number(uint64_t val)
    {
        PolyLvl1 poly = {};
        for (size_t i = 0; i < 64; i++) {
            bool b = ((val >> i) & 1) != 0;
            poly[i + 1] = (b ? one_over_eight : minus_one_over_eight);
        }
        poly[0] = poly[Lvl1::n / 2] = minus_one_over_eight;
        res() = trivial_TRLWELvl1(poly);
    }

    void set_counter(size_t val)
    {
        for (size_t i = 0; i < counter_width(); i++) {
            bool b = ((val >> i) & 1) != 0;
            counter_bit(i) = (b ? trivial_TRLWELvl1_1over8()
                                : trivial_TRLWELvl1_minus_1over8());
        }
    }

    void mark_counter()
    {
        mark_res_bit(Lvl1::n / 2);
    }

    void get_counter_mark(TLWELvl1& out)
    {
        TFHEpp::SampleExtractIndex<Lvl1>(out, res(), Lvl1::n / 2);
    }

    void mark_accept()
    {
        mark_res_bit(0);
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

    const TRLWELvl1& at(size_t i) const
    {
        return w.at(i);
    }

    const TRLWELvl1& res() const
    {
        return w.at(0);
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
        for (size_t i = 0; i < counter_width(); i++) {
            std::array<bool, Lvl1::n> poly =
                TFHEpp::trlweSymDecrypt<Lvl1>(counter_bit(i), key.key.lvl1);
            if (poly[0])
                cnt |= (1u << i);
        }

        return std::make_tuple(accept, res, cnt);
    }

    void dump(std::ostream& os, const SecretKey& skey) const
    {
        auto [accept, res, cnt] = decrypt(skey, 4);
        os << "\n"
           << "acc: " << accept << "\n"
           << "res: " << res << "\n"
           << "cnt: " << cnt << "\n";
    }
};

Weight alter_weight(const Weight& src, bool inc_res, bool reset_counter,
                    bool inc_counter, size_t counter_init_value)
{
    Weight ret{src};
    if (inc_res)
        ret.mark_accept();
    if (reset_counter)
        ret.set_counter(counter_init_value);
    if (inc_counter)
        ret.mark_counter();
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

void copy_counter(Weight& dst, const Weight& src)
{
    for (size_t i = 0; i < dst.counter_width(); i++)
        dst.counter_bit(i) = src.counter_bit(i);
}

void f(Weight& state, size_t counter_init_value, const TRGSWLvl1FFT& raw_input,
       const GateKey& gate_key, const CircuitKey& circuit_key,
       const SecretKey& debug_secret_key)
{
    std::cerr << ".";

    const size_t num_states = 9, log2_num_states = 4;

    std::vector<Weight> weight;
    weight.resize(num_states);
    for (size_t i = 1; i <= 3; i++) {
        // Store state #
        weight.at(i).set_state_number(i);
        // Store counter
        copy_counter(weight.at(i), state);
    }

    TRGSWLvl1FFT counter_cond;
    {
        TLWELvl0 acc = trivial_TLWELvl0_minus_1over8();
        for (size_t i = 0; i < state.counter_width(); i++) {
            TLWELvl1 t1;
            TFHEpp::SampleExtractIndex<Lvl1>(t1, state.counter_bit(i), 0);
            TLWELvl0 t0;
            TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(t0, t1, gate_key.ksk);
            TFHEpp::HomOR(acc, acc, t0, gate_key);
        }
        TFHEpp::HomNOT(acc, acc);
        CircuitBootstrappingFFTLvl01(counter_cond, acc, circuit_key);
    }

    std::vector<TRGSWLvl1FFT> inputs = {
        counter_cond,
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
            Weight in0 =
                alter_weight(weight.at(get<0>(d0)), get<1>(d0), get<2>(d0),
                             get<3>(d0), counter_init_value);
            Weight in1 =
                alter_weight(weight.at(get<0>(d1)), get<1>(d1), get<2>(d1),
                             get<3>(d1), counter_init_value);
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
        TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(t0, t1,
        gate_key.ksk); CircuitBootstrappingFFTLvl01(cond.at(i), t0,
        circuit_key);
    });
    */

    weight.resize(1 << log2_num_states);
    lookup_table(weight, cond.begin(), cond.end(), temp);
    state = weight.at(0);

    // Refresh state
    {
        TLWELvl1 borrow_l1;
        state.get_counter_mark(borrow_l1);
        TLWELvl0 borrow;
        TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(borrow, borrow_l1,
                                                      gate_key.ksk);

        for (size_t i = 0; i < state.counter_width(); i++) {
            // Minus 1
            TLWELvl1 t1;
            TFHEpp::SampleExtractIndex<Lvl1>(t1, state.counter_bit(i), 0);
            TLWELvl0 t0, new_bit;
            TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(t0, t1, gate_key.ksk);
            TFHEpp::HomXOR(new_bit, t0, borrow, gate_key);
            TFHEpp::GateBootstrappingTLWE2TRLWEFFT<TFHEpp::lvl01param>(
                state.counter_bit(i), new_bit, gate_key.bkfftlvl01);
            TFHEpp::HomANDNY(borrow, t0, borrow, gate_key);
        }
    }
}

void test(const SecretKey& skey, const BKey& bkey)
{
    const size_t alpha = 10;

    TRGSWLvl1FFT b1 = encrypt_bit_to_TRGSWLvl1FFT(true, skey);
    TRGSWLvl1FFT b0 = encrypt_bit_to_TRGSWLvl1FFT(false, skey);

    Weight state;
    state.set_counter(alpha);

    // 0 -> 4 -> 1
    f(state, alpha, b1, *bkey.gkey, *bkey.circuit_key, skey);
    state.dump(std::cerr, skey);
    // 1 -> 5 -> 2
    f(state, alpha, b0, *bkey.gkey, *bkey.circuit_key, skey);
    state.dump(std::cerr, skey);
    // 2 -> 7 -> 2 -> 7 -> 2 -> ... -> 2
    for (int i = 0; i < 11; i++) {
        f(state, alpha, b0, *bkey.gkey, *bkey.circuit_key, skey);

        state.dump(std::cerr, skey);
    }
    state.dump(std::cerr, skey);
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
