#include "archive.hpp"
#include "tfhepp_util.hpp"

#include <tbb/parallel_for.h>
#include <CLI/CLI.hpp>

#include <execution>
#include <filesystem>
#include <optional>

namespace {
const uint32_t one_over_eight = (1u << 29), minus_one_over_eight = -(1u << 29);
}

class Weight;

class Counter {
private:
    std::vector<TRLWELvl1> c_;
    size_t bit_width_;
    uint64_t init_val_;

public:
    Counter(size_t bit_width, uint64_t init_val)
        : bit_width_(bit_width), init_val_(init_val)
    {
        reset();
    }

    void reset()
    {
        c_.resize(bit_width_);
        for (size_t i = 0; i < bit_width_; i++) {
            bool b = ((init_val_ >> i) & 1) != 0;
            c_.at(i) = (b ? trivial_TRLWELvl1_1over8()
                          : trivial_TRLWELvl1_minus_1over8());
        }
    }

    template <class ExecutionPolicy>
    void is_zero(ExecutionPolicy&& exec, TLWELvl0& out,
                 const GateKey& gate_key) const
    {
        out = std::transform_reduce(
            exec, c_.begin(), c_.end(), trivial_TLWELvl0_minus_1over8(),
            [&gate_key](const TLWELvl0& acc, const TLWELvl0& c) {  // reduce
                TLWELvl0 t;
                TFHEpp::HomOR(t, acc, c, gate_key);
                return t;
            },
            [&gate_key](const TRLWELvl1& c) {  // transform
                TLWELvl1 t1;
                TFHEpp::SampleExtractIndex<Lvl1>(t1, c, 0);
                TLWELvl0 t0;
                TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(t0, t1,
                                                              gate_key.ksk);
                return t0;
            });
        TFHEpp::HomNOT(out, out);
    }

    void minus_one(std::execution::sequenced_policy, TLWELvl0 borrow,
                   const GateKey& gate_key)
    {
        // FIXME: parallel_policy?
        // FIXME: Implement and use HomXORwoSEI
        for (size_t i = 0; i < bit_width_; i++) {
            TLWELvl1 t1;
            TFHEpp::SampleExtractIndex<Lvl1>(t1, c_.at(i), 0);
            TLWELvl0 t0, new_bit;
            TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(t0, t1, gate_key.ksk);
            TFHEpp::HomXOR(new_bit, t0, borrow, gate_key);
            TFHEpp::GateBootstrappingTLWE2TRLWEFFT<TFHEpp::lvl01param>(
                c_.at(i), new_bit, gate_key.bkfftlvl01);
            TFHEpp::HomANDNY(borrow, t0, borrow, gate_key);
        }
    }

    uint64_t decrypt(const SecretKey& skey) const
    {
        uint64_t res = 0;
        for (size_t i = 0; i < bit_width_; i++) {
            auto poly = TFHEpp::trlweSymDecrypt<Lvl1>(c_.at(i), skey.key.lvl1);
            if (poly[0])
                res |= (1u << i);
        }
        return res;
    }

    static void cmux(Counter& out, const TRGSWLvl1FFT& sel, const Counter& in1,
                     const Counter& in0)
    {
        assert(out.c_.size() == in1.c_.size());
        assert(out.c_.size() == in0.c_.size());
        for (size_t i = 0; i < out.c_.size(); i++) {
            TFHEpp::CMUXFFT<Lvl1>(out.c_.at(i), sel, in1.c_.at(i),
                                  in0.c_.at(i));
        }
    }
};

class Weight {
private:
    TRLWELvl1 general_;
    /* Structure of general
         bit
         0    accept
         1..  state #
         n/2  borrow mark
   */
    std::vector<Counter> counter_;

private:
    void mark_general_bit(size_t i)
    {
        PolyLvl1 poly = {};
        poly[i] = (1 << 30);  // 1/4
        TRLWELvl1 t = trivial_TRLWELvl1(poly);
        TRLWELvl1_add(general_, t);
    }

    void get_general_bit(TLWELvl1& out, size_t i)
    {
        TFHEpp::SampleExtractIndex<Lvl1>(out, general_, i);
    }

    void initialize_general(uint64_t state_no)
    {
        PolyLvl1 poly = {};
        poly[0] = minus_one_over_eight;
        for (size_t i = 0; i < counter_.size(); i++) {
            poly[Lvl1::n / 2 + i] = minus_one_over_eight;
        }
        for (size_t i = 0; i < 64; i++) {
            bool b = ((state_no >> i) & 1) != 0;
            poly[i + 1] = (b ? one_over_eight : minus_one_over_eight);
        }
        general_ = trivial_TRLWELvl1(poly);
    }

public:
    Weight(uint64_t state_no,
           const std::vector<std::tuple<size_t, uint64_t>>& counter_spec)
    {
        initialize_general(state_no);

        // Initialize counters
        counter_.reserve(counter_spec.size());
        for (auto [bit_width, int_val] : counter_spec) {
            counter_.emplace_back(bit_width, int_val);
        }
    }

    // Use src as a source of counters
    Weight(uint64_t state_no, const Weight& src) : counter_(src.counter_)
    {
        initialize_general(state_no);
    }

    void mark_accept()
    {
        mark_general_bit(0);
    }

    void mark_counter(size_t i)
    {
        mark_general_bit(Lvl1::n / 2 + i);
    }

    void reset_counter(size_t i)
    {
        counter_.at(i).reset();
    }

    void get_counter_condition(std::execution::sequenced_policy,
                               std::vector<TRGSWLvl1FFT>& out,
                               const GateKey& gate_key,
                               const CircuitKey& circuit_key) const
    {
        out.resize(counter_.size());
        for (size_t i = 0; i < counter_.size(); i++) {
            TLWELvl0 temp;
            counter_.at(i).is_zero(std::execution::seq, temp, gate_key);
            CircuitBootstrappingFFTLvl01(out.at(i), temp, circuit_key);
        }
    }

    void get_selectors(std::execution::sequenced_policy,
                       std::vector<TRGSWLvl1FFT>& out, size_t log2_num_states,
                       const GateKey& gate_key,
                       const CircuitKey& circuit_key) const
    {
        out.resize(log2_num_states);
        for (size_t i = 0; i < log2_num_states; i++) {
            TLWELvl1 t1;
            TFHEpp::SampleExtractIndex<Lvl1>(t1, general_, i + 1);
            TLWELvl0 t0;
            TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(t0, t1, gate_key.ksk);
            CircuitBootstrappingFFTLvl01(out.at(i), t0, circuit_key);
        }
    }

    // For each counter, get its borrow and subtract it
    void update_counters(std::execution::sequenced_policy,
                         const GateKey& gate_key)
    {
        for (size_t i = 0; i < counter_.size(); i++) {
            TLWELvl1 borrow_l1;
            get_general_bit(borrow_l1, Lvl1::n / 2 + i);
            TLWELvl0 borrow;
            TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(borrow, borrow_l1,
                                                          gate_key.ksk);
            counter_.at(i).minus_one(std::execution::seq, borrow, gate_key);
        }
    }

    std::tuple<bool, size_t, size_t> decrypt(const SecretKey& skey) const
    {
        auto poly = TFHEpp::trlweSymDecrypt<Lvl1>(general_, skey.key.lvl1);

        bool accept = poly[0];

        uint64_t state_no = 0;
        for (size_t i = 0; i < 64; i++)
            if (poly[i + 1])
                state_no |= (1 << i);

        uint64_t counter_info = 0;
        for (size_t i = 0; i < counter_.size(); i++)
            if (poly[Lvl1::n / 2 + i])
                counter_info |= (1 << i);

        return std::make_tuple(accept, state_no, counter_info);
    }

    void dump(std::ostream& os, const SecretKey& skey) const
    {
        auto [accept, state_no, counter_info] = decrypt(skey);
        os << "\n"
           << "acc:  " << accept << "\n"
           << "st#:  " << state_no << "\n"
           << "info: " << counter_info << "\n";
        for (size_t i = 0; i < counter_.size(); i++)
            os << "cnt" << i << ": " << counter_.at(i).decrypt(skey);
        os << "\n";
    }

    static void cmux(Weight& out, const TRGSWLvl1FFT& sel, const Weight& in1,
                     const Weight& in0)
    {
        assert(out.counter_.size() == in1.counter_.size());
        assert(out.counter_.size() == in0.counter_.size());

        TFHEpp::CMUXFFT<Lvl1>(out.general_, sel, in1.general_, in0.general_);
        for (size_t i = 0; i < out.counter_.size(); i++)
            Counter::cmux(out.counter_.at(i), sel, in1.counter_.at(i),
                          in0.counter_.at(i));
    }
};

Weight alter_weight(const Weight& src, bool inc_res, bool reset_counter,
                    bool inc_counter)
{
    Weight ret{src};
    if (inc_res)
        ret.mark_accept();
    if (reset_counter)
        ret.reset_counter(0);
    if (inc_counter)
        ret.mark_counter(0);
    return ret;
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
    tmp.resize(1 << (input_size - 1), table.at(0) /* dummy */);

    size_t i = 0;
    for (auto it = input_begin; it != input_end; ++it, ++i) {
        for (size_t j = 0; j < 1 << (input_size - i - 1); j++) {
            Weight::cmux(tmp.at(j), *it, table.at(j * 2 + 1), table.at(j * 2));
        }
        // tbb::parallel_for(0, 1 << (input_size - i - 1), [&](size_t j) {
        //     TFHEpp::CMUXFFT<Lvl1>(tmp.at(j), *it, table.at(j * 2 + 1),
        //                           table.at(j * 2));
        // });
        tmp.swap(table);
    }
}

void f(Weight& state, const TRGSWLvl1FFT& raw_input, const GateKey& gate_key,
       const CircuitKey& circuit_key, const SecretKey& debug_secret_key)
{
    std::cerr << ".";

    const size_t num_states = 9, log2_num_states = 4;
    const std::vector<std::tuple<size_t, bool, bool, bool>> delta = {
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

    // Prepare weights
    std::vector<Weight> weight;
    weight.reserve(num_states);
    for (size_t i = 0; i < num_states; i++) {
        weight.emplace_back(i, state);
    }

    // Prepare inputs
    std::vector<TRGSWLvl1FFT> inputs;
    state.get_counter_condition(std::execution::seq, inputs, gate_key,
                                circuit_key);
    inputs.push_back(raw_input);

    // Let's do this
    std::vector<Weight> temp;
    temp.resize(weight.size(), state /* dummy */);
    for (int i = inputs.size() - 1; i >= 0; i--) {
        auto&& input = inputs.at(i);
        for (size_t i = 0; i < num_states; i++) {
            using std::get;

            Weight& out = temp.at(i);
            auto&& d0 = delta.at(i * 2 + 0);
            auto&& d1 = delta.at(i * 2 + 1);
            Weight in0 = alter_weight(weight.at(get<0>(d0)), get<1>(d0),
                                      get<2>(d0), get<3>(d0));
            Weight in1 = alter_weight(weight.at(get<0>(d1)), get<1>(d1),
                                      get<2>(d1), get<3>(d1));
            Weight::cmux(out, input, in1, in0);
        }
        weight.swap(temp);
    }

    std::vector<TRGSWLvl1FFT> selectors;
    state.get_selectors(std::execution::seq, selectors, log2_num_states,
                        gate_key, circuit_key);
    weight.resize(1 << log2_num_states, state /* dummy */);
    lookup_table(weight, selectors.begin(), selectors.end(), temp);

    state = weight.at(0);
    state.update_counters(std::execution::seq, gate_key);
}

void test(const SecretKey& skey, const BKey& bkey)
{
    const size_t alpha = 10;

    TRGSWLvl1FFT b1 = encrypt_bit_to_TRGSWLvl1FFT(true, skey);
    TRGSWLvl1FFT b0 = encrypt_bit_to_TRGSWLvl1FFT(false, skey);

    Weight state{0, std::vector{std::tuple<size_t, uint64_t>{10, alpha}}};

    // 0 -> 4 -> 1
    f(state, b1, *bkey.gkey, *bkey.circuit_key, skey);
    state.dump(std::cerr, skey);
    // 1 -> 5 -> 2
    f(state, b0, *bkey.gkey, *bkey.circuit_key, skey);
    state.dump(std::cerr, skey);
    // 2 -> 7 -> 2 -> 7 -> 2 -> ... -> 2
    for (int i = 0; i < 11; i++) {
        f(state, b0, *bkey.gkey, *bkey.circuit_key, skey);

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
