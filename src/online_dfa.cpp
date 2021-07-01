#include "online_dfa.hpp"

#include <execution>

#include <spdlog/spdlog.h>

/* OnlineDFARunner */
OnlineDFARunner::OnlineDFARunner(const Graph &graph,
                                 std::shared_ptr<GateKey> gate_key)
    : graph_(graph),
      weight_(graph.size(), trivial_TRLWELvl1_zero()),
      gate_key_(std::move(gate_key)),
      bootstrap_interval_(0),
      num_processed_inputs_(0)
{
    for (Graph::State st = 0; st < graph_.size(); st++)
        if (st == graph_.initial_state())
            weight_.at(st)[1][0] = (1u << 29);  // 1/8
        else
            weight_.at(st)[1][0] = -(1u << 29);  // -1/8

    // log_n(8000) = log(8000)/log(n)
    assert(graph_.size() > 1);
    bootstrap_interval_ = std::log(8000) / std::log(graph_.size());
    assert(bootstrap_interval_ > 0);
}

TLWELvl1 OnlineDFARunner::result()
{
    if (gate_key_)
        bootstrap_weight();  // FIXME: is this necessary?
    TRLWELvl1 acc = trivial_TRLWELvl1_minus_1over8(),
              offset = trivial_TRLWELvl1_1over8();
    for (Graph::State st = 0; st < graph_.size(); st++) {
        if (graph_.is_final_state(st)) {
            TRLWELvl1_add(acc, weight_.at(st));
            TRLWELvl1_add(acc, offset);
        }
    }
    if (gate_key_)
        do_SEI_IKS_GBTLWE2TRLWE(acc, *gate_key_);
    TLWELvl1 ret;
    TFHEpp::SampleExtractIndex<Lvl1>(ret, acc, 0);
    return ret;
}

void OnlineDFARunner::eval_one(const TRGSWLvl1FFT &input)
{
    std::vector<TRLWELvl1> out{weight_.size()};
    std::vector<Graph::State> states = graph_.all_states();
    std::for_each(
        std::execution::par, states.begin(), states.end(),
        [&](Graph::State st) {
            std::vector<Graph::State> parents0 = graph_.prev_states(st, false),
                                      parents1 = graph_.prev_states(st, true);
            TRLWELvl1 acc0 = trivial_TRLWELvl1_minus_1over8(),
                      acc1 = trivial_TRLWELvl1_minus_1over8(),
                      offset = trivial_TRLWELvl1_1over8();
            for (Graph::State p0 : parents0) {
                TRLWELvl1_add(acc0, weight_.at(p0));
                TRLWELvl1_add(acc0, offset);
            }
            for (Graph::State p1 : parents1) {
                TRLWELvl1_add(acc1, weight_.at(p1));
                TRLWELvl1_add(acc1, offset);
            }
            TFHEpp::CMUXFFT<Lvl1>(out.at(st), input, acc1, acc0);
        });
    {
        using std::swap;
        swap(out, weight_);
    }

    if (++num_processed_inputs_ % bootstrap_interval_ == 0)
        bootstrap_weight();
}

void OnlineDFARunner::bootstrap_weight()
{
    assert(gate_key_);
    std::for_each(
        std::execution::par, weight_.begin(), weight_.end(),
        [&](TRLWELvl1 &w) { do_SEI_IKS_GBTLWE2TRLWE(w, *gate_key_); });
}

/* OnlineDFARunner2 */
OnlineDFARunner2::OnlineDFARunner2(const Graph &graph,
                                   std::shared_ptr<GateKey> gate_key)
    : runner_(graph, std::nullopt, gate_key)
{
}

TLWELvl1 OnlineDFARunner2::result() const
{
    return runner_.result();
}

void OnlineDFARunner2::eval_one(const TRGSWLvl1FFT &input)
{
    return runner_.eval(input);
}

/* OnlineDFARunner3 */
OnlineDFARunner3::OnlineDFARunner3(
    const Graph &graph, const GateKey &gate_key,
    const TFHEpp::TLWE2TRLWEIKSKey<TFHEpp::lvl11param> &tlwel1_trlwel1_iks_key,
    std::optional<SecretKey> debug_skey)
    : graph_(graph),
      gate_key_(gate_key),
      tlwel1_trlwel1_iks_key_(tlwel1_trlwel1_iks_key),
      weight_(graph.size(), trivial_TRLWELvl1_zero()),
      queued_inputs_(),
      debug_skey_(std::move(debug_skey))
{
    assert(graph.size() < Lvl1::n);

    for (Graph::State st : graph_.all_states())
        if (st == graph_.initial_state())
            weight_.at(st)[1][0] = (1u << 30);  // 1/4

    queued_inputs_.reserve(QUEUE_SIZE);
    workspace_table1_.reserve(1 << QUEUE_SIZE);
    workspace_table2_.reserve(1 << QUEUE_SIZE);
}

TLWELvl1 OnlineDFARunner3::result()
{
    eval_queued_inputs();

    TRLWELvl1 acc = trivial_TRLWELvl1_zero();
    for (Graph::State st : graph_.all_states()) {
        if (graph_.is_final_state(st))
            TRLWELvl1_add(acc, weight_.at(st));
    }
    TRLWELvl1_add(acc, trivial_TRLWELvl1_minus_1over8());
    do_SEI_IKS_GBTLWE2TRLWE(acc, gate_key_);
    TLWELvl1 ret;
    TFHEpp::SampleExtractIndex<Lvl1>(ret, acc, 0);
    return ret;
}

void OnlineDFARunner3::eval_one(const TRGSWLvl1FFT &input)
{
    queued_inputs_.push_back(input);
    if (queued_inputs_.size() < QUEUE_SIZE)
        return;
    eval_queued_inputs();
}

template <class Func>
void execute_parallel(size_t begin, size_t end, Func func)
{
    std::vector<size_t> indices(end - begin);
    std::iota(indices.begin(), indices.end(), begin);
    std::for_each(std::execution::par, indices.begin(), indices.end(), func);
}

void OnlineDFARunner3::eval_queued_inputs()
{
    const size_t input_size = queued_inputs_.size();
    if (input_size == 0)
        return;
    const std::vector<Graph::State> all_states = graph_.all_states();

    // Create LUT
    std::vector<TRLWELvl1> &table = workspace_table1_;
    table.clear();
    table.resize(1 << input_size, trivial_TRLWELvl1_zero());
    execute_parallel(0, 1 << input_size, [&](size_t input) {
        for (Graph::State st_from : all_states) {
            // st_to = delta(st_from, input)
            Graph::State st_to = st_from;
            for (size_t i = 0; i < input_size; i++)
                st_to = graph_.next_state(st_to, (input & (1 << i)) != 0);

            // table_{input} += c_{st_from} * X^{st_to}
            TRLWELvl1 c;
            TRLWELvl1_mult_X_k(c, weight_.at(st_from), st_to);
            TRLWELvl1_add(table.at(input), c);
        }
    });

    // Select the correct entry using CMUX
    {
        std::vector<TRLWELvl1> &tmp = workspace_table2_;
        tmp.clear();
        tmp.resize(1 << (input_size - 1));
        for (size_t i = 0; i < input_size; i++) {
            const TRGSWLvl1FFT &input = queued_inputs_.at(i);
            execute_parallel(0, 1 << (input_size - i - 1), [&](size_t j) {
                TFHEpp::CMUXFFT<Lvl1>(tmp.at(j), input, table.at(j * 2 + 1),
                                      table.at(j * 2));
            });
            using std::swap;
            swap(tmp, table);
        }
    }
    const TRLWELvl1 &next_trlwe = table.at(0);

    /*
    if (debug_skey_) {
        TRLWELvl1 tmp = {};
        for (size_t i = 0; i < Lvl1::n; i++)
            tmp[1][i] = -(1u << 29);
        TRLWELvl1_add(tmp, next_trlwe);
        auto res = TFHEpp::trlweSymDecrypt<Lvl1>(tmp, debug_skey_->key.lvl1);
        for (Graph::State st : graph_.all_states())
            spdlog::debug("{} {}", st, res.at(st));
    }
    */

    // Split next_trlwe into |Q| TLWE, perform bootstrapping, and convert them
    // to |Q| TRLWE
    std::for_each(
        std::execution::par, all_states.begin(), all_states.end(),
        [&](Graph::State st) {
            TLWELvl1 tlwe_l1;
            TLWELvl0 tlwe_l0;
            TRLWELvl1 trlwe;

            // Split
            TFHEpp::SampleExtractIndex<Lvl1>(tlwe_l1, next_trlwe, st);
            // Bootstrap
            // FIXME: We can sometimes skip bootstrapping?
            TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(tlwe_l0, tlwe_l1,
                                                          gate_key_.ksk);
            TLWELvl0_add(tlwe_l0, trivial_TLWELvl0_minus_1over8());
            TFHEpp::GateBootstrappingTLWE2TRLWEFFT<TFHEpp::lvl01param>(
                trlwe, tlwe_l0, gate_key_.bkfftlvl01);
            TFHEpp::SampleExtractIndex<Lvl1>(tlwe_l1, trlwe, 0);
            TLWELvl1_add(tlwe_l1, trivial_TLWELvl1_1over8());
            // Convert
            TFHEpp::TLWE2TRLWEIKS<TFHEpp::lvl11param>(weight_.at(st), tlwe_l1,
                                                      tlwel1_trlwel1_iks_key_);
        });

    // Clear the queued inputs. Note that reserved space will NOT freed, which
    // is better.
    queued_inputs_.clear();
}
