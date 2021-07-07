#include "online_dfa.hpp"

#include <execution>

#include <spdlog/spdlog.h>
#include <tbb/parallel_for.h>

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
    : runner_(graph.reversed(), std::nullopt, gate_key)
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
    const Graph &graph, size_t queue_size, size_t bootstrapping_freq,
    const GateKey &gate_key,
    const TFHEpp::TLWE2TRLWEIKSKey<TFHEpp::lvl11param> &tlwel1_trlwel1_iks_key,
    std::optional<SecretKey> debug_skey)
    : graph_(graph),
      gate_key_(gate_key),
      tlwel1_trlwel1_iks_key_(tlwel1_trlwel1_iks_key),
      weight_(graph.size(), trivial_TRLWELvl1_zero()),
      queued_inputs_(0),
      queue_size_(queue_size),
      live_states_(),
      memo_transition_(),
      num_eval_(0),
      bootstrapping_freq_(bootstrapping_freq),
      debug_skey_(std::move(debug_skey))
{
    assert(graph.size() < Lvl1::n);

    for (Graph::State st : graph_.all_states())
        if (st == graph_.initial_state())
            weight_.at(st)[1][0] = (1u << 30);  // 1/4

    queued_inputs_.reserve(queue_size_);

    live_states_.push_back(graph_.initial_state());

    for (size_t depth = 0; depth <= queue_size_; depth++) {
        std::vector<Graph::State> memo;
        for (size_t input = 0; input < (1 << depth); input++) {
            for (Graph::State src : graph_.all_states()) {
                // FIXME: Use memo_transition_states_[depth-1]
                Graph::State dst = graph_.transition64(
                    src, static_cast<uint64_t>(input), depth);
                memo.push_back(dst);
            }
        }
        memo_transition_.push_back(memo);
    }
}

TLWELvl1 OnlineDFARunner3::result()
{
    eval_queued_inputs();

    TRLWELvl1 acc = trivial_TRLWELvl1_zero();
    for (Graph::State st : live_states_) {
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
        tbb::parallel_for(0, 1 << (input_size - i - 1), [&](size_t j) {
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
    const std::vector<Graph::State> &memo = memo_transition_.at(input_size);

    // Calculate next live states
    std::vector<Graph::State> next_live_states = [&] {
        std::set<Graph::State> tmp;
        for (size_t input = 0; input < (1 << input_size); input++) {
            for (Graph::State st_from : live_states_) {
                Graph::State st_to = memo.at(input * graph_.size() + st_from);
                tmp.insert(st_to);
            }
        }
        return std::vector<Graph::State>(tmp.begin(), tmp.end());
    }();
    // Map from state of next_live_states to [0, next_live_states.size()] index
    std::vector<size_t> st2idx(graph_.size(), 0);
    for (size_t i = 0; i < next_live_states.size(); i++)
        st2idx.at(next_live_states.at(i)) = i;

    // Determine 1st and 2nd LUT depth
    const size_t second_lut_depth = std::min<size_t>(
        input_size / 2, std::log2(Lvl1::n / next_live_states.size()));
    const size_t first_lut_depth =
        std::max<int>(0, input_size - second_lut_depth);
    assert(first_lut_depth + second_lut_depth == input_size);
    spdlog::debug("LUT {} + {} = {}", first_lut_depth, second_lut_depth,
                  input_size);

    // Prepare workspace avoiding malloc in eval
    std::vector<TRLWELvl1> &table = workspace_table1_,
                           &workspace = workspace_table2_;
    table.clear();
    table.resize(1 << first_lut_depth, trivial_TRLWELvl1_zero());

    // 1st step: |Q| TRLWE
    //       --> 2^{first_lut_depth} TRLWE
    //       --> 1 TRLWE
    tbb::parallel_for(0, 1 << first_lut_depth, [&](size_t input1) {
        for (Graph::State st_from : live_states_) {
            for (size_t input2 = 0; input2 < (1 << second_lut_depth);
                 input2++) {
                Graph::State st_to = memo.at(
                    ((input2 << first_lut_depth) | input1) * graph_.size() +
                    st_from);
                TRLWELvl1 c;
                TRLWELvl1_mult_X_k(
                    c, weight_.at(st_from),
                    input2 * next_live_states.size() + st2idx.at(st_to));
                TRLWELvl1_add(table.at(input1), c);
            }
        }
    });
    lookup_table(table, queued_inputs_.begin(),
                 queued_inputs_.begin() + first_lut_depth, workspace);

    // 2nd step: 1 TRLWE
    //       --> 2^{second_lut_depth} TRLWE
    //       --> 1 TRLWE
    table.resize(1 << second_lut_depth);
    tbb::parallel_for(1, 1 << second_lut_depth, [&](size_t i) {
        TRLWELvl1_mult_X_k(table.at(i), table.at(0),
                           2 * Lvl1::n - i * next_live_states.size());
    });
    lookup_table(table, queued_inputs_.begin() + first_lut_depth,
                 queued_inputs_.end(), workspace);

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

    num_eval_++;
    bool should_bootstrap = (num_eval_ % bootstrapping_freq_ == 0);
    // Split next_trlwe into |Q| TLWE, perform bootstrapping, and convert
    // them to |Q| TRLWE
    std::for_each(
        std::execution::par, next_live_states.begin(), next_live_states.end(),
        [&](Graph::State st) {
            TLWELvl1 tlwe_l1;
            TLWELvl0 tlwe_l0;
            TRLWELvl1 trlwe;

            // Extract
            TFHEpp::SampleExtractIndex<Lvl1>(tlwe_l1, next_trlwe,
                                             st2idx.at(st));
            if (should_bootstrap) {
                // Bootstrap
                TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(tlwe_l0, tlwe_l1,
                                                              gate_key_.ksk);
                TLWELvl0_add(tlwe_l0, trivial_TLWELvl0_minus_1over8());
                TFHEpp::GateBootstrappingTLWE2TRLWEFFT<TFHEpp::lvl01param>(
                    trlwe, tlwe_l0, gate_key_.bkfftlvl01);
                TFHEpp::SampleExtractIndex<Lvl1>(tlwe_l1, trlwe, 0);
                TLWELvl1_add(tlwe_l1, trivial_TLWELvl1_1over8());
            }
            // Convert
            TFHEpp::TLWE2TRLWEIKS<TFHEpp::lvl11param>(weight_.at(st), tlwe_l1,
                                                      tlwel1_trlwel1_iks_key_);
        });

    // Clear the queued inputs. Note that reserved space will NOT freed, which
    // is better.
    queued_inputs_.clear();

    using std::swap;
    swap(live_states_, next_live_states);
}
