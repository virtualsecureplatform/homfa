#include <cassert>
#include <execution>
#include <fstream>
#include <iostream>
#include <queue>
#include <set>
#include <sstream>
#include <thread>

#include <tfhe++.hpp>

using TRGSWLvl1FFT = TFHEpp::TRGSWFFT<TFHEpp::lvl1param>;
using TRLWELvl1 = TFHEpp::TRLWE<TFHEpp::lvl1param>;
using PolyLvl1 = TFHEpp::Polynomial<TFHEpp::lvl1param>;
using SecretKey = TFHEpp::SecretKey;
using Lvl1 = TFHEpp::lvl1param;

TRLWELvl1 trivial_TRLWELvl1(const PolyLvl1 &src)
{
    TRLWELvl1 ret = {};
    ret[1] = src;
    return ret;
}

TRLWELvl1 trivial_TRLWELvl1_zero()
{
    TRLWELvl1 ret = {};
    return ret;
}

// out += src
void TRLWELvl1_add(TRLWELvl1 &out, const TRLWELvl1 &src)
{
    for (size_t i = 0; i < Lvl1::n; i++) {
        out[0][i] += src[0][i];
        out[1][i] += src[1][i];
    }
}

PolyLvl1 phase_of_TRLWELvl1(const TRLWELvl1 &src, const SecretKey &skey)
{
    PolyLvl1 as;
    TFHEpp::PolyMul<Lvl1>(as, src[0], skey.key.lvl1);
    PolyLvl1 phase = src[1];
    for (size_t i = 0; i < Lvl1::n; i++)
        phase[i] -= as[i];
    return phase;
}

PolyLvl1 uint2weight(uint64_t n)
{
    PolyLvl1 w;
    const uint32_t mu = 1u << 31;
    for (size_t i = 0; i < Lvl1::n; i++)
        if (i < 64)
            w[i] = ((n >> i) & 1u) ? mu : 0;
        else
            w[i] = 0;
    return w;
}

std::string weight2bitstring(const PolyLvl1 &w)
{
    const uint32_t mu25 = 1u << 30, mu75 = (1u << 30) + (1u << 31);
    std::stringstream ss;
    for (size_t i = 0; i < Lvl1::n; i++) {
        if (i % 32 == 0)
            ss << "\n";
        else if (i % 8 == 0)
            ss << " ";
        if (mu25 <= w[i] && w[i] <= mu75)
            ss << 1;
        else
            ss << 0;
    }
    ss << "\n";
    return ss.str();
}

class Graph {
public:
    using State = int;

private:
    // When on state `index`,
    //   input 0 -> next state is `child0`,
    //              next weight is (w_{org} * X^k + gain0)
    //   input 1 -> next state is `child1`
    //              next weight is (w_{org} * X^k + gain1)
    struct TableItem {
        State index, child0, child1;
        uint64_t gain0, gain1;
    };
    std::vector<TableItem> table_;
    std::vector<std::vector<State>> states_at_depth_;

public:
    Graph()
    {
    }

    Graph(const std::string &filename)
    {
        std::ifstream ifs{filename};
        assert(ifs);
        int N;
        ifs >> N;
        std::set<int> finalState;
        for (int i = 0; i < N; i++) {
            std::string no;
            int s0, s1;
            ifs >> no;
            ifs >> s0 >> s1;
            if (no.at(no.size() - 1) == '*')
                finalState.insert(i);
            table_.push_back(TableItem{i, s0, s1, 0, 0});
        }
        for (auto &&item : table_) {
            if (finalState.contains(item.child0))
                item.gain0 = 1;
            if (finalState.contains(item.child1))
                item.gain1 = 1;
        }
    }

    size_t size() const
    {
        return table_.size();
    }

    State next_state(State state, bool input) const
    {
        auto &t = table_.at(state);
        return input ? t.child1 : t.child0;
    }

    State initial_state() const
    {
        return 0;
    }

    uint64_t gain(State from, bool input) const
    {
        auto &s = table_.at(from);
        return input ? s.gain1 : s.gain0;
    }

    void reserve_states_at_depth(size_t depth)
    {
        states_at_depth_.clear();
        states_at_depth_.shrink_to_fit();
        states_at_depth_.reserve(depth);

        std::set<int> sts0, sts1;
        sts0.insert(initial_state());
        for (size_t i = 0; i < depth; i++) {
            states_at_depth_.emplace_back(sts0.begin(), sts0.end());

            sts1.clear();
            for (State st : sts0) {
                sts1.insert(next_state(st, false));
                sts1.insert(next_state(st, true));
            }
            {
                using std::swap;
                swap(sts0, sts1);
            }
        }
    }

    std::vector<State> states_at_depth(size_t depth) const
    {
        return states_at_depth_.at(depth);
    }
};

class DetWFARunner {
private:
    Graph graph_;
    std::vector<TRGSWLvl1FFT> input_;
    size_t weight_num_scale_;
    std::vector<TRLWELvl1> weight_;
    bool has_evaluated_;
    int shift_width_, shift_interval_;
    std::optional<SecretKey> secret_key_;

public:
    DetWFARunner(Graph graph, std::vector<TRGSWLvl1FFT> input,
                 std::optional<SecretKey> secret_key = std::nullopt)
        : graph_(std::move(graph)),
          input_(std::move(input)),
          weight_num_scale_(1 + input_.size() / Lvl1::n),
          weight_(weight_num_scale_ * graph_.size(), trivial_TRLWELvl1_zero()),
          has_evaluated_(false),
          shift_width_(1),
          shift_interval_(1),
          secret_key_(std::move(secret_key))
    {
        std::cerr << "Parameter:\n"
                  << "\tInput size:\t" << input_.size() << "\n"
                  << "\tState size:\t" << graph_.size() << "\n"
                  << "\tWeight Num Scale:\t" << weight_num_scale_ << "\n"
                  << "\tWeight size:\t" << weight_.size() << "\n"
                  << "\tConcurrency:\t" << std::thread::hardware_concurrency()
                  << "\n"
                  << "=====\n";
        //<< "\tShift width:\t" << shift_width_ << "\n"
        //<< "\tShift interval:\t" << shift_interval_ << "\n"
    }

    std::vector<TRLWELvl1> result() const
    {
        assert(has_evaluated_);
        auto it = weight_.begin() + graph_.initial_state() * weight_num_scale_;
        return std::vector<TRLWELvl1>{it, it + weight_num_scale_};
    }

    void eval()
    {
        assert(!has_evaluated_);
        has_evaluated_ = true;

        size_t total_cnt_cmux = 0;
        std::vector<TRLWELvl1> out(weight_.size(), trivial_TRLWELvl1_zero());
        for (int j = input_.size() - 1; j >= 0; --j) {
            auto states = graph_.states_at_depth(j);
            std::for_each(
                std::execution::par, states.begin(), states.end(),
                [&](auto &&q) {
                    for (size_t i = 0; i < weight_num_scale_; i++) {
                        TRLWELvl1 w0, w1;
                        next_weight(w1, i, j, q, true);
                        next_weight(w0, i, j, q, false);
                        TFHEpp::CMUXFFT<Lvl1>(out.at(q * weight_num_scale_ + i),
                                              input_.at(j), w1, w0);

                        if (secret_key_) {
                            auto dec_out = weight2bitstring(phase_of_TRLWELvl1(
                                out.at(q * weight_num_scale_ + i),
                                *secret_key_));
                            auto dec_in1 = weight2bitstring(
                                phase_of_TRLWELvl1(w1, *secret_key_));
                            auto dec_in0 = weight2bitstring(
                                phase_of_TRLWELvl1(w0, *secret_key_));
                            assert(dec_out == dec_in0 || dec_out == dec_in1);
                        }
                    }
                });
            {
                using std::swap;
                swap(out, weight_);
            }
            std::cerr << "[" << j << "] #CMUX : " << states.size() << "\n";
            total_cnt_cmux += states.size();
        }
        std::cerr << "Total #CMUX : " << total_cnt_cmux << "\n";
    }

private:
    void mult_X_k(TRLWELvl1 &out, const TRLWELvl1 &src, size_t k) const
    {
        TFHEpp::PolynomialMulByXai<Lvl1>(out[0], src[0], k);
        TFHEpp::PolynomialMulByXai<Lvl1>(out[1], src[1], k);
    }

    void next_weight(TRLWELvl1 &out, size_t i, int j, Graph::State from,
                     bool input) const
    {
        Graph::State to = graph_.next_state(from, input);
        uint64_t w = graph_.gain(from, input);

        if (j % shift_interval_ == shift_interval_ - 1 && i == j / Lvl1::n) {
            mult_X_k(out, weight_.at(to * weight_num_scale_ + i), shift_width_);
            TRLWELvl1_add(out, trivial_TRLWELvl1(uint2weight(w)));
        }
        else {
            out = weight_.at(to * weight_num_scale_ + i);
        }
    }
};

void det_wfa(const char *graph_filename, const char *input_filename)
{
    SecretKey skey;

    const std::vector<bool> plain_input = [&] {
        std::ifstream ifs{input_filename};
        assert(ifs);
        std::vector<bool> ret;
        while (ifs) {
            int ch = ifs.get();
            if (ch == EOF)
                break;
            for (int i = 0; i < 8; i++)
                ret.push_back(((static_cast<uint8_t>(ch) >> i) & 1u) != 0);
        }
        return ret;
    }();
    std::vector<TRGSWLvl1FFT> input;
    for (bool b : plain_input)
        input.emplace_back(
            TFHEpp::trgswfftSymEncrypt<Lvl1>(b, Lvl1::α, skey.key.lvl1));

    Graph gr{graph_filename};
    gr.reserve_states_at_depth(input.size());

    DetWFARunner runner{gr, input, skey};
    runner.eval();
    std::vector<TRLWELvl1> enc_res = runner.result();

    std::cout << "Result (bitstr):\n";
    for (auto &&t : enc_res)
        std::cout << weight2bitstring(phase_of_TRLWELvl1(t, skey)) << "\n";
}

int main(int argc, char **argv)
{
    if (argc != 3) {
        fprintf(stderr, "Usage: %s AUTOMATON-SPEC-FILE INPUT-FILE", argv[0]);
        return 1;
    }

    det_wfa(argv[1], argv[2]);

    return 0;
}
