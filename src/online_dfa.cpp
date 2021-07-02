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
    const Graph &graph, size_t first_lut_max_depth, const GateKey &gate_key,
    const TFHEpp::TLWE2TRLWEIKSKey<TFHEpp::lvl11param> &tlwel1_trlwel1_iks_key,
    std::optional<SecretKey> debug_skey)
    : graph_(graph),
      gate_key_(gate_key),
      tlwel1_trlwel1_iks_key_(tlwel1_trlwel1_iks_key),
      weight_(graph.size(), trivial_TRLWELvl1_zero()),
      queued_inputs_(),
      first_lut_max_depth_(first_lut_max_depth),
      queue_size_(0),
      debug_skey_(std::move(debug_skey))
{
    assert(graph.size() < Lvl1::n);

    for (Graph::State st : graph_.all_states())
        if (st == graph_.initial_state())
            weight_.at(st)[1][0] = (1u << 30);  // 1/4

    size_t snd_depth = std::log2(Lvl1::n / graph_.size());
    assert((1 << snd_depth) <= Lvl1::n / graph_.size());
    queue_size_ = first_lut_max_depth_ + snd_depth;
    queued_inputs_.reserve(queue_size_);
    size_t workspace_size = std::max(first_lut_max_depth_, snd_depth);
    workspace_table1_.reserve(1 << workspace_size);
    workspace_table2_.reserve(1 << workspace_size);
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
    if (queued_inputs_.size() < queue_size_)
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

void lookup_table(std::vector<TRLWELvl1> &table,
                  std::vector<TRGSWLvl1FFT>::const_iterator input_begin,
                  std::vector<TRGSWLvl1FFT>::const_iterator input_end,
                  std::vector<TRLWELvl1> &workspace)
{
    const size_t input_size = std::distance(input_begin, input_end);
    assert(table.size() == (1 << input_size));  // FIXME: relax this condition
    if (input_size == 0)
        return;

    std::vector<TRLWELvl1> &tmp = workspace;
    tmp.clear();
    tmp.resize(1 << (input_size - 1));

    size_t i = 0;
    for (auto it = input_begin; it != input_end; ++it, ++i) {
        execute_parallel(0, 1 << (input_size - i - 1), [&](size_t j) {
            TFHEpp::CMUXFFT<Lvl1>(tmp.at(j), *it, table.at(j * 2 + 1),
                                  table.at(j * 2));
        });
        using std::swap;
        swap(tmp, table);
    }
}

void OnlineDFARunner3::eval_queued_inputs()
{
    const size_t input_size = queued_inputs_.size();
    if (input_size == 0)
        return;
    const std::vector<Graph::State> all_states = graph_.all_states();

    const size_t first_lut_depth = std::min(input_size, first_lut_max_depth_),
                 second_lut_depth =
                     std::max<int>(0, input_size - first_lut_depth);
    assert(first_lut_depth + second_lut_depth == input_size);
    assert(first_lut_depth <= first_lut_max_depth());
    assert(second_lut_depth <= second_lut_max_depth());

    // 1st step: |Q| TRLWE
    //       --> 2^{first_lut_depth} TRLWE
    //       --> 1 TRLWE
    std::vector<TRLWELvl1> &table = workspace_table1_;
    table.clear();
    table.resize(1 << first_lut_depth, trivial_TRLWELvl1_zero());
    execute_parallel(0, 1 << first_lut_depth, [&](size_t input1) {
        for (Graph::State st_from : all_states) {
            Graph::State st_mid = graph_.transition64(
                st_from, static_cast<uint64_t>(input1), first_lut_depth);
            for (size_t input2 = 0; input2 < (1 << second_lut_depth);
                 input2++) {
                Graph::State st_to = graph_.transition64(
                    st_mid, static_cast<uint64_t>(input2), second_lut_depth);
                TRLWELvl1 c;
                TRLWELvl1_mult_X_k(c, weight_.at(st_from),
                                   input2 * graph_.size() + st_to);
                TRLWELvl1_add(table.at(input1), c);
            }
        }
    });
    lookup_table(table, queued_inputs_.begin(),
                 queued_inputs_.begin() + first_lut_depth, workspace_table2_);

    // 2nd step: 1 TRLWE
    //       --> 2^{second_lut_depth} TRLWE
    //       --> 1 TRLWE
    table.resize(1 << second_lut_depth);
    for (size_t i = 1; i < (1 << second_lut_depth); i++) {
        TRLWELvl1_mult_X_k(table.at(i), table.at(0),
                           2 * Lvl1::n - i * graph_.size());
    }
    lookup_table(table, queued_inputs_.begin() + first_lut_depth,
                 queued_inputs_.end(), workspace_table2_);

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
