#include "online_dfa.hpp"
#include "error.hpp"
#include "timeit.hpp"

#include <execution>

#include <spdlog/spdlog.h>
#include <tbb/parallel_for.h>

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

void lookup_table_with_timer(
    std::vector<TRLWELvl1> &table,
    std::vector<TRGSWLvl1FFT>::const_iterator input_begin,
    std::vector<TRGSWLvl1FFT>::const_iterator input_end,
    std::vector<TRLWELvl1> &workspace, TimeRecorder &timer)
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
        timer.timeit(
            TimeRecorder::TARGET::CMUX, 1 << (input_size - i - 1), [&] {
                tbb::parallel_for(0, 1 << (input_size - i - 1), [&](size_t j) {
                    TFHEpp::CMUXFFT<Lvl1>(tmp.at(j), *it, table.at(j * 2 + 1),
                                          table.at(j * 2));
                });
            });
        using std::swap;
        swap(tmp, table);
    }
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
      sanitize_result_(sanitize_result),
      timer_()
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
        timer_.timeit(TimeRecorder::TARGET::CMUX, states.size(), [&] {
            std::for_each(std::execution::par, states.begin(), states.end(),
                          [&](Graph::State q) {
                              Graph::State q0 = graph_.next_state(q, false),
                                           q1 = graph_.next_state(q, true);
                              const auto &w0 = weight.at(q0),
                                         &w1 = weight.at(q1);
                              TFHEpp::CMUXFFT<Lvl1>(
                                  out.at(q), queued_inputs_.at(i), w1, w0);
                          });
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
    workspace4_.resize(width);
    tbb::parallel_for(0ul, width, [&](size_t i) {
        TLWELvl1 tlwel1;
        TFHEpp::SampleExtractIndex<Lvl1>(tlwel1, sel, i + 1);
        TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(workspace4_.at(i), tlwel1,
                                                      gate_key_.ksk);
    });
    timer_.timeit(TimeRecorder::TARGET::CIRCUIT_BOOTSTRAPPING, width, [&] {
        tbb::parallel_for(0ul, width, [&](size_t i) {
            CircuitBootstrappingFFTLvl01(cond.at(i), workspace4_.at(i),
                                         circuit_key_);
        });
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
    lookup_table_with_timer(weight, cond.begin(), cond.end(), out, timer_);
    selector_ = weight.at(0);
}
