#include "archive.hpp"
#include "error.hpp"
#include "graph.hpp"
#include "tfhepp_util.hpp"

#include <cassert>
#include <execution>
#include <fstream>
#include <iostream>
#include <queue>
#include <set>
#include <sstream>
#include <thread>
#include <vector>

#include <spdlog/spdlog.h>

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
            spdlog::debug("[{}] #CMUX : {}", states.size());
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
