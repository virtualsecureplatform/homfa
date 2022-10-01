#include "archive.hpp"
#include "graph.hpp"
#include "tfhepp_util.hpp"

#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#include <CLI/CLI.hpp>

#include <execution>
#include <filesystem>
#include <optional>

namespace {
const uint32_t one_over_eight = (1u << 29), minus_one_over_eight = -(1u << 29);

template <class F>
void parallel_for(std::execution::parallel_policy, size_t start, size_t end,
                  F&& f)
{
    tbb::parallel_for(start, end, f);
}

template <class F>
void parallel_for(std::execution::sequenced_policy, size_t start, size_t end,
                  F&& f)
{
    for (size_t i = start; i < end; i++)
        f(i);
}

template <class T>
std::vector<T> unwrap_optional(const std::vector<std::optional<T>>& src,
                               T default_value)
{
    std::vector<T> ret;
    ret.reserve(src.size());
    for (auto&& elm : src)
        ret.push_back(elm.value_or(default_value));
    return ret;
}

bool TLWELvl1_decrypt(const TLWELvl1& src, const SecretKey& skey)
{
    return TFHEpp::bootsSymDecrypt<Lvl1>({src}, skey).at(0);
}
}  // namespace

// Binarized Temporal Tester (?)
class BTT {
public:
    using State = int;
    using CounterNo = size_t;
    using CounterSpec =
        std::tuple<size_t /* bit width */, uint64_t /* init val */>;

    struct Edge {
        State from, to;
        bool input, accept;
        std::vector<CounterNo> reset, inc;
        std::optional<CounterNo> check;

        Edge(State from, bool input, State to, bool accept,
             std::vector<CounterNo> reset, std::vector<CounterNo> inc,
             std::optional<CounterNo> check)
            : from(from),
              to(to),
              input(input),
              accept(accept),
              reset(std::move(reset)),
              inc(std::move(inc)),
              check(std::move(check))
        {
        }
    };

    using Delta = std::vector<Edge>;

private:
    Delta delta_;
    std::vector<CounterSpec> counters_spec_;

public:
    BTT(State init_state, const Delta& delta,
        std::vector<CounterSpec> counters_spec);

    static BTT from_istream(std::istream& is);

    Graph to_graph() const;

    size_t size() const
    {
        return delta_.size() / 2;
    }

    State initial_state() const
    {
        return 0;
    }

    std::vector<CounterSpec> counters_spec() const
    {
        return counters_spec_;
    }

    std::pair<Edge, Edge> get_edge(State state) const
    {
        return std::make_pair(delta_.at(state * 2), delta_.at(state * 2 + 1));
    }
};

BTT::BTT(State init_state, const Delta& delta,
         std::vector<CounterSpec> counters_spec)
    : delta_(delta), counters_spec_(std::move(counters_spec))
{
    // Sanity check
    assert(init_state == 0);  // FIXME: Relax this condition
    assert(delta.size() % 2 == 0);
    for (size_t i = 0; i < delta.size(); i++) {
        assert(delta.at(i).from == i / 2);
        assert(delta.at(i).input == (i % 2 != 0));
    }
}

Graph BTT::to_graph() const
{
    std::set<State> final_states;
    Graph::DFADelta delta;
    for (size_t q = 0; q < size(); q++) {
        auto [e0, e1] = get_edge(q);
        delta.emplace_back(q, e0.to, e1.to);
        final_states.insert(q);
    }
    return Graph{initial_state(), final_states, delta};
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

    template <class ExecutionPolicy>
    void minus_one(ExecutionPolicy&& exec, TLWELvl0 borrow,
                   const GateKey& gate_key)
    {
        std::vector<TLWELvl0> memo;
        memo.reserve(bit_width_ * 2);
        for (size_t i = 0; i < bit_width_; i++) {
            TLWELvl1 t1;
            TFHEpp::SampleExtractIndex<Lvl1>(t1, c_.at(i), 0);
            TLWELvl0 t0;
            TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(t0, t1, gate_key.ksk);
            memo.emplace_back(t0);
            memo.emplace_back(borrow);
            TFHEpp::HomANDNY(borrow, t0, borrow, gate_key);
        }

        parallel_for(exec, 0, bit_width_, [&](size_t i) {
            HomXORwoSE(c_.at(i), memo.at(i * 2), memo.at(i * 2 + 1), gate_key);
        });
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
    Weight(uint64_t state_no, const std::vector<BTT::CounterSpec>& counter_spec)
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

    void get_accept(TLWELvl1& out)
    {
        get_general_bit(out, 0);
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

    template <class ExecutionPolicy>
    void get_counter_condition(ExecutionPolicy&& exec,
                               std::vector<TRGSWLvl1FFT>& out,
                               const GateKey& gate_key,
                               const CircuitKey& circuit_key) const
    {
        out.resize(counter_.size());
        parallel_for(exec, 0, counter_.size(), [&](size_t i) {
            TLWELvl0 temp;
            counter_.at(i).is_zero(exec, temp, gate_key);
            CircuitBootstrappingFFTLvl01(out.at(i), temp, circuit_key);
        });
    }

    template <class ExecutionPolicy>
    std::vector<TRGSWLvl1FFT> get_selectors(ExecutionPolicy&& exec,
                                            size_t log2_num_states,
                                            const GateKey& gate_key,
                                            const CircuitKey& circuit_key) const
    {
        std::vector<TRGSWLvl1FFT> out(log2_num_states);
        parallel_for(exec, 0, log2_num_states, [&](size_t i) {
            TLWELvl1 t1;
            TFHEpp::SampleExtractIndex<Lvl1>(t1, general_, i + 1);
            TLWELvl0 t0;
            TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(t0, t1, gate_key.ksk);
            CircuitBootstrappingFFTLvl01(out.at(i), t0, circuit_key);
        });
        return out;
    }

    // For each counter, get its borrow and subtract it
    template <class ExecutionPolicy>
    void update_counters(ExecutionPolicy&& exec, const GateKey& gate_key)
    {
        parallel_for(exec, 0, counter_.size(), [&](size_t i) {
            TLWELvl1 borrow_l1;
            get_general_bit(borrow_l1, Lvl1::n / 2 + i);
            TLWELvl0 borrow;
            TFHEpp::IdentityKeySwitch<TFHEpp::lvl10param>(borrow, borrow_l1,
                                                          gate_key.ksk);
            counter_.at(i).minus_one(exec, borrow, gate_key);
        });
    }

    std::tuple<bool, size_t, size_t, std::vector<uint64_t>> decrypt(
        const SecretKey& skey) const
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

        std::vector<uint64_t> counter;
        for (auto&& c : counter_)
            counter.push_back(c.decrypt(skey));

        return std::make_tuple(accept, state_no, counter_info, counter);
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

Weight alter_weight(const Weight& src, bool accept,
                    const std::vector<size_t>& reset_counter,
                    const std::vector<size_t>& inc_counter)
{
    Weight ret{src};
    if (accept)
        ret.mark_accept();
    for (size_t i : reset_counter)
        ret.reset_counter(i);
    for (size_t i : inc_counter)
        ret.mark_counter(i);
    return ret;
}

template <class ExecutionPolicy>
void lookup_table(ExecutionPolicy&& exec, std::vector<Weight>& table,
                  std::vector<TRGSWLvl1FFT>::const_iterator input_begin,
                  std::vector<TRGSWLvl1FFT>::const_iterator input_end)
{
    const size_t input_size = std::distance(input_begin, input_end);
    assert(table.size() == (1 << input_size));  // FIXME: relax this condition
    if (input_size == 0)
        return;

    std::vector<Weight> tmp;
    tmp.clear();
    tmp.resize(1 << (input_size - 1), table.at(0) /* dummy */);

    size_t i = 0;
    for (auto it = input_begin; it != input_end; ++it, ++i) {
        parallel_for(exec, 0, 1 << (input_size - i - 1), [&](size_t j) {
            Weight::cmux(tmp.at(j), *it, table.at(j * 2 + 1), table.at(j * 2));
        });
        tmp.swap(table);
    }
}

class BTTRunner {
private:
    BTT btt_;
    Graph graph_;
    const GateKey& gate_key_;
    const CircuitKey& circuit_key_;
    size_t queue_size_;
    std::vector<TRGSWLvl1FFT> queued_inputs_;
    Weight state_;
    std::vector<Graph::State> live_states_;

public:
    BTTRunner(const BTT& btt, size_t queue_size, const GateKey& gate_key,
              const CircuitKey& circuit_key);

    const Graph& graph() const
    {
        return graph_;
    }

    size_t queue_size() const
    {
        return queue_size_;
    }

    size_t num_live_states() const
    {
        return live_states_.size();
    }

    TLWELvl1 result();
    void eval_one(const TRGSWLvl1FFT& input);

    void dump(std::ostream& os, const SecretKey& skey);

private:
    void eval_queued_inputs();

    template <class ExecutionPolicy>
    void eval_block_cmuxs(
        ExecutionPolicy&& exec, std::vector<std::optional<Weight>>& weight,
        const std::vector<TRGSWLvl1FFT>& inputs,
        const std::vector<std::vector<Graph::State>>& live_states_at_depth);
};

BTTRunner::BTTRunner(const BTT& btt, size_t queue_size, const GateKey& gate_key,
                     const CircuitKey& circuit_key)
    : btt_(btt),
      graph_(btt.to_graph()),
      gate_key_(gate_key),
      circuit_key_(circuit_key),
      queue_size_(queue_size),
      queued_inputs_(),
      state_(btt.initial_state(), btt.counters_spec()),
      live_states_({btt.initial_state()})
{
}

TLWELvl1 BTTRunner::result()
{
    eval_queued_inputs();
    TLWELvl1 ret;
    state_.get_accept(ret);
    return ret;
}

void BTTRunner::eval_one(const TRGSWLvl1FFT& input)
{
    queued_inputs_.push_back(input);
    if (queued_inputs_.size() < queue_size_)
        return;
    eval_queued_inputs();
}

void BTTRunner::dump(std::ostream& os, const SecretKey& skey)
{
    auto [accept, state_no, counter_info, counter] = state_.decrypt(skey);
    os << "acc:  " << accept << "\n"
       << "st#:  " << live_states_.at(state_no) << "\n"
       << "info: " << counter_info << "\n";
    for (size_t i = 0; i < counter.size(); i++)
        os << "cnt" << i << ": " << counter.at(i) << "\n";
}

template <class ExecutionPolicy>
void BTTRunner::eval_block_cmuxs(
    ExecutionPolicy&& exec, std::vector<std::optional<Weight>>& weight,
    const std::vector<TRGSWLvl1FFT>& inputs,
    const std::vector<std::vector<Graph::State>>& live_states_at_depth)
{
    std::vector<std::optional<Weight>> temp(weight.size());
    for (int i = inputs.size() - 1; i >= 0; i--) {
        auto&& input = inputs.at(i);
        const auto& states = live_states_at_depth.at(i);
        std::for_each(exec, states.begin(), states.end(), [&](Graph::State q) {
            auto [e0, e1] = btt_.get_edge(q);
            Weight in0 = alter_weight(weight.at(e0.to).value(), e0.accept,
                                      e0.reset, e0.inc);
            Weight in1 = alter_weight(weight.at(e1.to).value(), e1.accept,
                                      e1.reset, e1.inc);
            auto& out = temp.at(q);
            out.emplace(0, state_ /* dummy */);
            Weight::cmux(out.value(), input, in1, in0);
        });
        weight.swap(temp);
    }
}

void BTTRunner::eval_queued_inputs()
{
    auto exec = std::execution::par;

    if (queued_inputs_.size() == 0)
        return;

    // Prepare inputs
    std::vector<TRGSWLvl1FFT> inputs;
    state_.get_counter_condition(exec, inputs, gate_key_, circuit_key_);
    std::copy(queued_inputs_.begin(), queued_inputs_.end(),
              std::back_inserter(inputs));
    queued_inputs_.clear();

    // Calculate live states
    const std::vector<Graph::State> all_states = graph_.all_states();
    const std::vector<Graph::State> live_states = live_states_;
    const std::vector<std::vector<Graph::State>> live_states_at_depth =
        graph_.track_live_states(live_states, inputs.size());
    const std::vector<Graph::State> next_live_states =
        live_states_at_depth.back();

    // Update live_states_ to next live states.
    // Note that live_states (not suffixed a '_') have current live states.
    live_states_ = next_live_states;

    // Map from next live state to index
    std::vector<int> next_live_to_index(graph_.size(), -1);
    for (size_t i = 0; i < next_live_states.size(); i++)
        next_live_to_index.at(next_live_states.at(i)) = i;

    const size_t num_states = btt_.size();

    // Prepare weights
    std::vector<std::optional<Weight>> weight(num_states);
    for (Graph::State q : next_live_states)
        weight.at(q).emplace(next_live_to_index.at(q), state_);

    // Let's do this
    eval_block_cmuxs(exec, weight, inputs, live_states_at_depth);

    // Select the correct candidate
    if (live_states.size() == 1) {
        state_ = weight.at(0).value();
    }
    else {
        // Extract selectors
        const size_t width = std::ceil(std::log2(live_states.size()));
        std::vector<TRGSWLvl1FFT> selectors =
            state_.get_selectors(exec, width, gate_key_, circuit_key_);

        // Select
        std::vector<Weight> weight_nonopt;
        weight_nonopt.reserve(live_states.size());
        for (size_t i = 0; i < live_states.size(); i++)
            weight_nonopt.push_back(weight.at(live_states.at(i)).value());
        weight_nonopt.resize(1 << width, Weight{0, state_} /* dummy */);
        lookup_table(exec, weight_nonopt, selectors.begin(), selectors.end());
        state_ = weight_nonopt.at(0);
    }

    // Update
    state_.update_counters(exec, gate_key_);
}

class BTTPlaintextRunner {
private:
    BTT btt_;
    BTT::State cur_st_;
    bool accept_;
    std::vector<int> counter_;

public:
    BTTPlaintextRunner(BTT btt);

    bool result() const;
    void eval_one(bool input);
    void dump(std::ostream& os) const;
};

BTTPlaintextRunner::BTTPlaintextRunner(BTT btt)
    : btt_(std::move(btt)),
      cur_st_(btt_.initial_state()),
      accept_(false),
      counter_(btt_.counters_spec().size(), 0)
{
}

bool BTTPlaintextRunner::result() const
{
    return accept_;
}

void BTTPlaintextRunner::eval_one(bool input)
{
    auto counters_spec = btt_.counters_spec();
    for (size_t i = 0; i < counters_spec.size(); i++) {
        uint64_t target = 0;
        std::tie(std::ignore, target) = counters_spec.at(i);
        bool satisfied = (counter_.at(i) == target);
        auto [e0, e1] = btt_.get_edge(cur_st_);
        cur_st_ = (satisfied ? e1 : e0).to;
    }

    auto [e0, e1] = btt_.get_edge(cur_st_);
    BTT::Edge e = (input ? e1 : e0);
    cur_st_ = e.to;
    accept_ = e.accept;
    for (BTT::CounterNo i : e.reset)
        counter_.at(i) = 0;
    for (BTT::CounterNo i : e.inc)
        counter_.at(i)++;
}

void BTTPlaintextRunner::dump(std::ostream& os) const
{
    os << "acc:  " << accept_ << "\n"
       << "st#:  " << cur_st_ << "\n";
    for (size_t i = 0; i < counter_.size(); i++)
        os << "cnt" << i << ": " << counter_.at(i) << "\n";
}

void test(const SecretKey& skey, const BKey& bkey)
{
    const size_t alpha = 10;

    BTT btt{
        0,
        {
            // from, input, to, accept, reset, inc, check
            {0, 0, 4, false, {}, {}, 0},
            {0, 1, 4, false, {}, {}, 0},
            {1, 0, 5, false, {}, {}, 0},
            {1, 1, 5, false, {}, {}, 0},
            {2, 0, 7, false, {}, {}, 0},
            {2, 1, 6, false, {}, {}, 0},
            {3, 0, 8, false, {}, {}, 0},
            {3, 1, 8, false, {}, {}, 0},
            {4, 0, 3, false, {}, {}, std::nullopt},
            {4, 1, 1, true, {}, {}, std::nullopt},
            {5, 0, 2, true, {0}, {}, std::nullopt},
            {5, 1, 1, true, {}, {}, std::nullopt},
            {6, 0, 3, false, {}, {}, std::nullopt},
            {6, 1, 1, true, {}, {}, std::nullopt},
            {7, 0, 2, true, {}, {0}, std::nullopt},
            {7, 1, 1, true, {}, {}, std::nullopt},
            {8, 0, 3, false, {}, {}, std::nullopt},
            {8, 1, 1, true, {}, {}, std::nullopt},
        },
        {
            {10, alpha},
        },
    };

    const size_t num_tests = 5;
    std::atomic_bool should_abort{false};
    std::array<std::string, num_tests> error_message;
    parallel_for(std::execution::par, 0, num_tests, [&](size_t i_test) {
        std::cerr << i_test << " ";
        std::default_random_engine engine{std::random_device{}()};
        std::binomial_distribution<> dist;
        BTTPlaintextRunner prunner{btt};
        BTTRunner runner{btt, 1, *bkey.gkey, *bkey.circuit_key};
        std::vector<bool> history;
        for (size_t i = 0; i < 10000; i++) {
            if (should_abort.load())
                return;
            bool input = dist(engine);
            history.push_back(input);
            runner.eval_one(encrypt_bit_to_TRGSWLvl1FFT(input, skey));
            bool result = TLWELvl1_decrypt(runner.result(), skey);
            prunner.eval_one(input);
            bool expected = prunner.result();
            if (result != expected) {
                std::stringstream ss;
                ss << "======= TEST ERROR =======\n";
                for (size_t i = 0; i < history.size(); i++)
                    ss << "Input " << i << ":\t" << history.at(i) << "\n";
                ss << "\nRunner result:\t" << result << "\n";
                runner.dump(ss, skey);
                ss << "\nPRunner result:\t" << expected << "\n";
                prunner.dump(ss);
                ss << "==========================\n";
                error_message.at(i_test) = ss.str();
                should_abort.store(true);
                return;
            }
        }
    });
    if (should_abort.load()) {
        for (size_t i = 0; i < error_message.size(); i++) {
            if (error_message.at(i).empty())
                continue;
            std::cerr << "TEST " << i << "\n";
            std::cerr << error_message.at(i) << "\n";
        }
    }

    /*
    TRGSWLvl1FFT b1 = encrypt_bit_to_TRGSWLvl1FFT(true, skey);
    TRGSWLvl1FFT b0 = encrypt_bit_to_TRGSWLvl1FFT(false, skey);

    BTTRunner runner{btt, 1, *bkey.gkey, *bkey.circuit_key};
    runner.eval_one(b1);
    runner.dump(std::cerr, skey);
    runner.eval_one(b0);
    runner.dump(std::cerr, skey);
    for (size_t i = 0; i < 11; i++) {
        runner.eval_one(b0);
        runner.dump(std::cerr, skey);
    }
    */
}

int main(int argc, char** argv)
{
    CLI::App app{"Benchmark runner"};
    // app.require_subcommand();
    std::optional<int> num_cpu_cores;
    app.add_option("--cpu", num_cpu_cores, "Number of CPU cores we use");
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

    (num_cpu_cores ? tbb::task_arena(*num_cpu_cores) : tbb::task_arena())
        .execute([&] { test(*skey, *bkey); });

    return 0;
}
