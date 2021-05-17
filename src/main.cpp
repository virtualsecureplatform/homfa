#include <cassert>
#include <execution>
#include <fstream>
#include <iostream>
#include <queue>
#include <set>
#include <sstream>
#include <thread>

#include <spdlog/spdlog.h>
#include <tfhe++.hpp>

using Lvl0 = TFHEpp::lvl0param;
using TLWELvl0 = TFHEpp::TLWE<Lvl0>;
using Lvl1 = TFHEpp::lvl1param;
using TLWELvl1 = TFHEpp::TLWE<Lvl1>;
using TRGSWLvl1FFT = TFHEpp::TRGSWFFT<Lvl1>;
using TRLWELvl1 = TFHEpp::TRLWE<Lvl1>;
using PolyLvl1 = TFHEpp::Polynomial<Lvl1>;
using SecretKey = TFHEpp::SecretKey;
using GateKey = TFHEpp::GateKey;

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

uint32_t phase_of_TLWELvl1(const TLWELvl1 &src, const SecretKey &skey)
{
    uint32_t phase = src[Lvl1::n];
    for (size_t i = 0; i < Lvl1::n; i++)
        phase -= src[i] * skey.key.lvl1[i];
    return phase;
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

bool between_25_75(uint32_t n)
{
    const uint32_t mu25 = 1u << 30, mu75 = (1u << 30) + (1u << 31);
    return mu25 <= n && n <= mu75;
}

void dump_weight(std::ostream &os, const PolyLvl1 &w)
{
    for (size_t i = 0; i < Lvl1::n; i++) {
        if (i % 32 == 0)
            os << "\n";
        else if (i % 8 == 0)
            os << " ";
        if (between_25_75(w[i]))
            os << 1;
        else
            os << 0;
    }
    os << "\n";
}

std::string weight2bitstring(const PolyLvl1 &w)
{
    std::stringstream ss;
    dump_weight(ss, w);
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
    std::set<State> final_state_;

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
        for (int i = 0; i < N; i++) {
            std::string no;
            int s0, s1;
            ifs >> no;
            ifs >> s0 >> s1;
            if (no.at(no.size() - 1) == '*')
                final_state_.insert(i);
            table_.push_back(TableItem{i, s0, s1, 0, 0});
        }
        for (auto &&item : table_) {
            if (final_state_.contains(item.child0))
                item.gain0 = 1;
            if (final_state_.contains(item.child1))
                item.gain1 = 1;
        }
    }

    size_t size() const
    {
        return table_.size();
    }

    bool is_final_state(State state) const
    {
        return final_state_.contains(state);
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
    const Graph &graph_;
    const std::vector<TRGSWLvl1FFT> &input_;
    size_t weight_num_scale_;
    std::vector<TRLWELvl1> weight_;
    bool has_evaluated_;
    int shift_width_, shift_interval_;
    std::optional<SecretKey> secret_key_;

public:
    DetWFARunner(const Graph &graph, const std::vector<TRGSWLvl1FFT> &input,
                 std::optional<SecretKey> secret_key = std::nullopt)
        : graph_(graph),
          input_(input),
          weight_num_scale_(1 + input_.size() / Lvl1::n),
          weight_(weight_num_scale_ * graph_.size(), trivial_TRLWELvl1_zero()),
          has_evaluated_(false),
          shift_width_(1),
          shift_interval_(1),
          secret_key_(std::move(secret_key))
    {
        spdlog::info("Parameter:");
        spdlog::info("\tInput size:\t{}", input_.size());
        spdlog::info("\tState size:\t{}", graph_.size());
        spdlog::info("\tWeight Num Scale:\t{}", weight_num_scale_);
        spdlog::info("\tWeight size:\t{}", weight_.size());
        spdlog::info("\tConcurrency:\t{}", std::thread::hardware_concurrency());
        spdlog::info("");
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
            spdlog::debug("[{}] #CMUX : ", states.size());
            total_cnt_cmux += states.size();
        }
        spdlog::info("Total #CMUX : {}", total_cnt_cmux);
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

    // DetWFARunner runner{gr, input, skey};
    DetWFARunner runner{gr, input};
    runner.eval();
    std::vector<TRLWELvl1> enc_res = runner.result();

    spdlog::info("Result (bitstr):");
    {
        std::stringstream ss;
        for (auto &&t : enc_res)
            dump_weight(ss, phase_of_TRLWELvl1(t, skey));
        spdlog::info(ss.str());
    }
}

class OfflineFARunner {
    // Interval for bootstrapping
    const static size_t BOOT_INTERVAL = 8000;

private:
    const Graph &graph_;
    const std::vector<TRGSWLvl1FFT> &input_;
    std::vector<TRLWELvl1> weight_;
    bool has_evaluated_;
    std::shared_ptr<GateKey> gate_key_;

public:
    OfflineFARunner(const Graph &graph, const std::vector<TRGSWLvl1FFT> &input,
                    std::shared_ptr<GateKey> gate_key = nullptr)
        : graph_(graph),
          input_(input),
          weight_(graph_.size(), trivial_TRLWELvl1_zero()),
          has_evaluated_(false),
          gate_key_(std::move(gate_key))
    {
        for (Graph::State st = 0; st < graph_.size(); st++)
            if (graph_.is_final_state(st))
                weight_.at(st)[1][0] = (1u << 29);  // 1/8
            else
                weight_.at(st)[1][0] = -(1u << 29);  // -1/8

        spdlog::info("Parameter:");
        spdlog::info("\tMode:\t{}", "Offline FA Runner");
        spdlog::info("\tInput size:\t{}", input_.size());
        spdlog::info("\tState size:\t{}", graph_.size());
        spdlog::info("\tWeight size:\t{}", weight_.size());
        spdlog::info("\tConcurrency:\t{}", std::thread::hardware_concurrency());
        spdlog::info("");
    }

    TLWELvl1 result() const
    {
        auto w = weight_.at(graph_.initial_state());
        TLWELvl1 ret;
        TFHEpp::SampleExtractIndex<Lvl1>(ret, w, 0);
        return ret;
    }

    void eval()
    {
        assert(!has_evaluated_);
        has_evaluated_ = true;

        size_t total_cnt_cmux = 0;
        std::vector<TRLWELvl1> out(weight_.size(), trivial_TRLWELvl1_zero());
        for (int j = input_.size() - 1; j >= 0; --j) {
            auto states = graph_.states_at_depth(j);
            std::for_each(std::execution::par, states.begin(), states.end(),
                          [&](auto &&q) {
                              TRLWELvl1 w0, w1;
                              next_weight(w1, j, q, true);
                              next_weight(w0, j, q, false);
                              TFHEpp::CMUXFFT<Lvl1>(out.at(q), input_.at(j), w1,
                                                    w0);
                          });
            {
                using std::swap;
                swap(out, weight_);
            }

            if (size_t cur = input_.size() - j;
                gate_key_ && cur != 0 && cur % BOOT_INTERVAL == 0) {
                spdlog::info("Bootstrapping occurred");
                bootstrapping_of_weight();
            }

            spdlog::debug("[{}] #CMUX : ", states.size());
            total_cnt_cmux += states.size();
        }
        spdlog::info("Total #CMUX : {}", total_cnt_cmux);
    }

private:
    void next_weight(TRLWELvl1 &out, int j, Graph::State from, bool input) const
    {
        Graph::State to = graph_.next_state(from, input);
        out = weight_.at(to);
    }

    void bootstrapping_of_weight()
    {
        assert(gate_key_);
        const GateKey &gk = *gate_key_;
        std::for_each(
            std::execution::par, weight_.begin(), weight_.end(), [&](auto &&w) {
                TLWELvl1 tlwel1;
                TFHEpp::SampleExtractIndex<Lvl1>(tlwel1, w, 0);
                TLWELvl0 tlwel0;
                TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(tlwel0, tlwel1,
                                                              gk.ksk);
                TFHEpp::GateBootstrappingTLWE2TRLWEFFT<TFHEpp::lvl01param>(
                    w, tlwel0, gk.bkfftlvl01);
            });
    }
};

void offline_dfa(const char *graph_filename, const char *input_filename)
{
    SecretKey skey;
    auto gkey = std::make_shared<GateKey>(skey);

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

    OfflineFARunner runner{gr, input, gkey};
    runner.eval();

    TLWELvl1 enc_res = runner.result();
    bool res = TFHEpp::tlweSymDecrypt<Lvl1>(enc_res, skey.key.lvl1);
    spdlog::info("Result (bool): {}", res);
}

int main(int argc, char **argv)
{
    if (argc != 3) {
        spdlog::error("Usage: {} AUTOMATON-SPEC-FILE INPUT-FILE", argv[0]);
        return 1;
    }

    // det_wfa(argv[1], argv[2]);
    offline_dfa(argv[1], argv[2]);

    return 0;
}
