#include "online_dfa.hpp"
#include "error.hpp"

#include <execution>

#include <spdlog/spdlog.h>
#include <tbb/parallel_for.h>

/* OnlineDFARunner */
OnlineDFARunner::OnlineDFARunner(const Graph &graph,
                                 std::shared_ptr<GateKey> gate_key,
                                 bool sanitize_result)
    : graph_(graph),
      weight_(graph.size(), trivial_TRLWELvl1_zero()),
      gate_key_(std::move(gate_key)),
      bootstrap_interval_(0),
      num_processed_inputs_(0),
      sanitize_result_(sanitize_result)
{
    assert(gate_key_);

    if (sanitize_result_)
        error::die("Sanitization of results is not implemented (qtrlwe)");

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
    assert(!sanitize_result_);

    TRLWELvl1 acc = trivial_TRLWELvl1_minus_1over8(),
              offset = trivial_TRLWELvl1_1over8();
    for (Graph::State st = 0; st < graph_.size(); st++) {
        if (graph_.is_final_state(st)) {
            TRLWELvl1_add(acc, weight_.at(st));
            TRLWELvl1_add(acc, offset);
        }
    }

    assert(0 &&
           "We need convert {-1/8,1/8} to {0,1/2}, but it is not implemented "
           "here.");

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
OnlineDFARunner2::OnlineDFARunner2(const Graph &graph, size_t boot_interval,
                                   bool is_spec_reversed,
                                   std::shared_ptr<GateKey> gate_key,
                                   bool sanitize_result)
    : runner_(is_spec_reversed ? graph : graph.reversed().minimized(),
              boot_interval, std::nullopt, gate_key, sanitize_result)
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
    Graph graph, size_t max_second_lut_depth, size_t queue_size,
    size_t bootstrapping_freq, const GateKey &gate_key,
    const TFHEpp::TLWE2TRLWEIKSKey<TFHEpp::lvl11param> &tlwel1_trlwel1_iks_key,
    std::optional<SecretKey> debug_skey, bool sanitize_result)
    : graph_(std::move(graph)),
      gate_key_(gate_key),
      tlwel1_trlwel1_iks_key_(tlwel1_trlwel1_iks_key),
      weight_(graph_.size(), trivial_TRLWELvl1_zero()),
      queued_inputs_(0),
      max_second_lut_depth_(max_second_lut_depth),
      queue_size_(queue_size),
      live_states_(),
      memo_transition_(),
      num_eval_(0),
      bootstrapping_freq_(bootstrapping_freq),
      first_lut_depth_(0),
      second_lut_depth_(0),
      debug_skey_(std::move(debug_skey)),
      sanitize_result_(sanitize_result)
{
    if (sanitize_result_)
        error::die("Sanitization of results is not implemented");

    for (Graph::State st : graph_.all_states())
        if (st == graph_.initial_state())
            weight_.at(st)[1][0] = (1u << 31);  // 1/2

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
    assert(!sanitize_result_);

    eval_queued_inputs();

    TRLWELvl1 acc = trivial_TRLWELvl1_zero();
    for (Graph::State st : live_states_) {
        if (graph_.is_final_state(st))
            TRLWELvl1_add(acc, weight_.at(st));
    }
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

    spdlog::debug("live states: {}", live_states_.size());
    spdlog::debug("next live states: {}", next_live_states.size());

    if (next_live_states.size() >= (1 << max_second_lut_depth_))
        error::die(
            "The number of next live states ({}) must be smaller than "
            "2^max_second_lut_depth ({})",
            next_live_states.size(), 1 << max_second_lut_depth_);

    // Determine 1st and 2nd LUT depth
    const size_t second_lut_depth = std::min<size_t>(
        input_size / 2,
        max_second_lut_depth_ - std::log2(next_live_states.size()));
    const size_t first_lut_depth =
        std::max<int>(0, input_size - second_lut_depth);
    assert(first_lut_depth + second_lut_depth == input_size);
    spdlog::debug("LUT {} + {} = {}", first_lut_depth, second_lut_depth,
                  input_size);
    first_lut_depth_ = first_lut_depth;
    second_lut_depth_ = second_lut_depth;

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
    std::for_each(std::execution::par, next_live_states.begin(),
                  next_live_states.end(), [&](Graph::State st) {
                      TLWELvl1 tlwe_l1;
                      TLWELvl0 tlwe_l0;
                      TRLWELvl1 trlwe;

                      // Extract
                      TFHEpp::SampleExtractIndex<Lvl1>(tlwe_l1, next_trlwe,
                                                       st2idx.at(st));
                      if (should_bootstrap) {
                          // Bootstrap
                          TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(
                              tlwe_l0, tlwe_l1, gate_key_.ksk);
                          BS_TLWE_0_1o2_to_TRLWE_0_1o2(trlwe, tlwe_l0,
                                                       gate_key_);
                          TFHEpp::SampleExtractIndex<Lvl1>(tlwe_l1, trlwe, 0);
                      }
                      // Convert
                      TFHEpp::TLWE2TRLWEIKS<TFHEpp::lvl11param>(
                          weight_.at(st), tlwe_l1, tlwel1_trlwel1_iks_key_);
                  });

    // Clear the queued inputs. Note that reserved space will NOT freed, which
    // is better.
    queued_inputs_.clear();

    using std::swap;
    swap(live_states_, next_live_states);
}

/* OnlineDFARunner4 */
OnlineDFARunner4::OnlineDFARunner4(Graph graph, size_t queue_size,
                                   const GateKey &gate_key,
                                   const CircuitKey &circuit_key,
                                   bool sanitize_result)
    : graph_(std::move(graph)),
      gate_key_(gate_key),
      circuit_key_(circuit_key),
      queue_size_(queue_size),
      queued_inputs_(),
      selector_(std::nullopt),
      live_states_({graph_.initial_state()}),
      sanitize_result_(sanitize_result)
{
    if (sanitize_result_)
        error::die("Sanitization of results is not implemented");
}

TLWELvl1 OnlineDFARunner4::result()
{
    assert(!sanitize_result_);

    eval_queued_inputs();
    assert(selector_);
    TLWELvl1 ret;
    TFHEpp::SampleExtractIndex<Lvl1>(ret, *selector_, 0);
    return ret;
}

void OnlineDFARunner4::eval_one(const TRGSWLvl1FFT &input)
{
    queued_inputs_.push_back(input);
    if (queued_inputs_.size() < queue_size_)
        return;
    eval_queued_inputs();
}

void OnlineDFARunner4::eval_queued_inputs()
{
    const size_t input_size = queued_inputs_.size();
    if (input_size == 0)
        return;

    const std::vector<Graph::State> all_states = graph_.all_states();
    const std::vector<Graph::State> live_states = live_states_;
    const std::vector<std::vector<Graph::State>> live_states_at_depth = [&] {
        std::vector<std::vector<Graph::State>> at_depth;
        at_depth.push_back(live_states);

        std::set<Graph::State> tmp1(live_states.begin(), live_states.end()),
            tmp2;
        for (size_t i = 0; i < input_size; i++) {
            tmp2.clear();
            for (Graph::State q : tmp1) {
                tmp2.insert(graph_.next_state(q, true));
                tmp2.insert(graph_.next_state(q, false));
            }
            {
                using std::swap;
                swap(tmp1, tmp2);
            }
            at_depth.emplace_back(tmp1.begin(), tmp1.end());
        }

        return at_depth;
    }();
    const std::vector<Graph::State> next_live_states =
        live_states_at_depth.back();

    // Update live_states_ to next live states.
    // Note that live_states (not suffixed a '_') have current live states.
    live_states_ = next_live_states;

    // Map from next live state to index
    std::vector<int> next_live_to_index(graph_.size(), -1);
    for (size_t i = 0; i < next_live_states.size(); i++)
        next_live_to_index.at(next_live_states.at(i)) = i;

    std::vector<TRLWELvl1> &weight = workspace1_, &out = workspace2_;
    weight.clear();
    out.clear();
    weight.resize(graph_.size(), trivial_TRLWELvl1_zero());
    out.resize(graph_.size());

    const size_t next_width =
        std::floor(std::log2(next_live_states.size())) + 1;
    // Initialize weights.
    // The content of a weight (TRLWE) at index i:
    //   [0]: true iff the state is final
    //   [1..]: index of the state (sum(2^{i-1} * w[i]))
    for (Graph::State q : next_live_states) {
        if (graph_.is_final_state(q))
            weight.at(q)[1][0] = (1u << 31);  // 1/2
        else
            weight.at(q)[1][0] = 0;  // 0

        size_t t = next_live_to_index.at(q);
        for (size_t i = 0; i < next_width; i++)
            if (((t >> i) & 1u) == 0)
                weight.at(q)[1][i + 1] = -(1u << 29);  // -1/8
            else
                weight.at(q)[1][i + 1] = (1u << 29);  // 1/8
    }

    // Propagate weight from back to front
    for (int i = input_size - 1; i >= 0; i--) {
        const auto &states = live_states_at_depth.at(i);
        std::for_each(std::execution::par, states.begin(), states.end(),
                      [&](Graph::State q) {
                          Graph::State q0 = graph_.next_state(q, false),
                                       q1 = graph_.next_state(q, true);
                          const auto &w0 = weight.at(q0), &w1 = weight.at(q1);
                          TFHEpp::CMUXFFT<Lvl1>(out.at(q), queued_inputs_.at(i),
                                                w1, w0);
                      });
        {
            using std::swap;
            swap(out, weight);
        }
    }
    queued_inputs_.clear();

    // Now choose correct weight from the previous block's result, that is,
    // selector.

    // If the number of the live states is 1, all we need is just pick the one.
    if (live_states.size() == 1) {
        selector_ = weight.at(live_states.at(0));
        return;
    }

    // First apply CB to get the selector in TRGSW
    size_t width = std::floor(std::log2(live_states.size())) + 1;
    std::vector<TRGSWLvl1FFT> &cond = workspace3_;
    cond.clear();
    cond.resize(width);
    const TRLWELvl1 &sel = *selector_;
    tbb::parallel_for(0ul, width, [&](size_t i) {
        TLWELvl1 tlwel1;
        TFHEpp::SampleExtractIndex<Lvl1>(tlwel1, sel, i + 1);
        TLWELvl0 tlwel0;
        TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(tlwel0, tlwel1,
                                                      gate_key_.ksk);
        CircuitBootstrappingFFTLvl01(cond.at(i), tlwel0, circuit_key_);
    });
    // Then choose the correct weight specified by the selector
    for (size_t i = 0; i < live_states.size(); i++)
        out.at(i) = weight.at(live_states.at(i));
    {
        using std::swap;
        swap(weight, out);
    }
    weight.resize(1 << width);
    out.resize(1 << width);
    lookup_table(weight, cond.begin(), cond.end(), out);
    selector_ = weight.at(0);
}
