#include "archive.hpp"
#include "error.hpp"
#include "offline_dfa.hpp"
#include "online_dfa.hpp"

#include <CLI/CLI.hpp>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>
#include <spot/misc/bddlt.hh>
#include <spot/misc/minato.hh>
#include <spot/tl/parse.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/translate.hh>

const static size_t MIN_INPUT_LENGTH = 8000, MAX_INPUT_LENGTH = 10000;

class TestInputStream : public InputStream<TRGSWLvl1FFT> {
private:
    std::vector<bool> spec_;
    std::vector<bool>::iterator head_;
    TRGSWLvl1FFT c0_, c1_;

public:
    TestInputStream(std::vector<bool> spec, const TRGSWLvl1FFT& c0,
                    const TRGSWLvl1FFT& c1)
        : spec_(spec), head_(spec_.begin()), c0_(c0), c1_(c1)
    {
    }

    size_t size() const override
    {
        return spec_.end() - head_;
    }

    TRGSWLvl1FFT next() override
    {
        assert(size() != 0);
        bool b = *(head_++);
        return b ? c1_ : c0_;
    }
};

class ReversedTestInputStream : public InputStream<TRGSWLvl1FFT> {
private:
    std::vector<bool> spec_;
    std::vector<bool>::reverse_iterator head_;
    TRGSWLvl1FFT c0_, c1_;

public:
    ReversedTestInputStream(std::vector<bool> spec, const TRGSWLvl1FFT& c0,
                            const TRGSWLvl1FFT& c1)
        : spec_(spec), head_(spec_.rbegin()), c0_(c0), c1_(c1)
    {
    }

    size_t size() const override
    {
        return spec_.rend() - head_;
    }

    TRGSWLvl1FFT next() override
    {
        assert(size() != 0);
        bool b = *(head_++);
        return b ? c1_ : c0_;
    }
};

template <class T>
std::string vec2str(const std::vector<T>& src)
{
    std::stringstream ss;
    for (auto&& b : src) {
        ss << b << ",";
    }
    return ss.str();
}

bool check_if_accept(const Graph& graph, const std::vector<bool>& src)
{
    Graph::State q = graph.initial_state();
    for (bool b : src)
        q = graph.next_state(q, b);
    return graph.is_final_state(q);
}

// aut should be deterministic
bool check_if_accept(spot::twa_graph_ptr aut, const std::vector<bool>& src,
                     size_t input_kind_size)
{
    std::unordered_map<int, unsigned> var2idx;
    {
        spot::bdd_dict_ptr dict = aut->get_dict();
        for (size_t i = 0; i < input_kind_size; i++) {
            std::stringstream ss;
            ss << "p" << i;
            const std::string varname = ss.str();
            auto it = dict->var_map.find(spot::formula::ap(varname));
            if (it != dict->var_map.end())
                var2idx.emplace(it->second, i);
        }
    }

    size_t input_size = src.size() / input_kind_size;
    std::set<unsigned> qs;
    qs.insert(aut->get_init_state_number());
    for (size_t i = 0; i < input_size; i++) {
        std::set<unsigned> next_qs;
        for (unsigned q : qs) {
            for (const auto& t : aut->out(q)) {
                bdd n = t.cond;
                size_t offset = i * input_kind_size;
                while (n != bddtrue && n != bddfalse) {
                    size_t index = offset + var2idx.at(bdd_var(n));
                    if (src.at(index))
                        n = bdd_high(n);
                    else
                        n = bdd_low(n);
                }
                if (n == bddtrue)
                    next_qs.insert(t.dst);
            }
        }
        if (next_qs.empty())
            return false;
        qs = std::move(next_qs);
    }

    for (unsigned q : qs)
        if (aut->state_is_accepting(q))
            return true;
    return false;
}

std::vector<bool> int2bvec(size_t i, size_t multiple)
{
    size_t size = log2(i) + 1;
    while (size % multiple != 0)
        size++;
    std::vector<bool> in;
    for (size_t b = 0; b < size; b++)
        in.push_back((i & (1 << b)) != 0);
    return in;
}

template <class RG>
std::vector<bool> random_bvec(RG& rgen, size_t multiple)
{
    std::uniform_int_distribution<> length_dist(MIN_INPUT_LENGTH,
                                                MAX_INPUT_LENGTH);
    std::bernoulli_distribution bool_dist;

    size_t length = length_dist(rgen);
    length += multiple - length % multiple;
    std::vector<bool> ret(length);
    for (size_t i = 0; i < length; i++)
        ret.at(i) = bool_dist(rgen);

    return ret;
}

std::string bvec2str(const std::vector<bool>& src)
{
    std::stringstream ss;
    for (bool b : src) {
        ss << b << ",";
    }
    return ss.str();
}

std::vector<size_t> create_table_to_permutate_input_bits(
    const spot::twa_graph_ptr& aut, size_t num_ap)
{
    spot::bdd_dict_ptr dict = aut->get_dict();
    std::unordered_map<int, size_t> var2idx;
    for (size_t i = 0; i < num_ap; i++) {
        std::stringstream ss;
        ss << "p" << i;
        auto it = dict->var_map.find(spot::formula::ap(ss.str()));
        if (it != dict->var_map.end())
            var2idx.emplace(it->second, i);
    }

    std::vector<size_t> in_idx;
    bdd all = aut->ap_vars();
    while (all != bddtrue) {
        int v = bdd_var(all);
        all = bdd_high(all);
        auto it = var2idx.find(v);
        if (it != var2idx.end())
            in_idx.push_back(it->second);
    }

    while (in_idx.size() < num_ap)  // FIXME: correct?
        in_idx.push_back(0);
    assert(in_idx.size() == num_ap);

    return in_idx;
}

std::vector<bool> permutate_input_bits(const std::vector<size_t>& tbl,
                                       const std::vector<bool>& src)
{
    std::vector<bool> in;
    size_t num_ap = tbl.size();
    size_t input_size = src.size() / num_ap;
    for (size_t i = 0; i < input_size; i++) {
        for (size_t j : tbl) {
            size_t index = i * num_ap + j;
            in.push_back(src.at(index));
        }
    }
    return in;
}

struct Snapshot {
    SecretKey skey;
    BKey bkey;
    TRGSWLvl1FFT c0, c1;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(skey, bkey, c0, c1);
    }
};

enum class METHOD {
    OFFLINE,
    ONLINE_REVERSED,
    ONLINE_QTRLWE2,
};

void test_from_ltl_formula(std::istream& is, size_t num_ap, size_t num_test,
                           METHOD method, const std::string& snapshot_path)
{
    const size_t NUM_INT2BVEC_TEST = 10;

    SecretKey skey;
    BKey bkey{skey};
    TRGSWLvl1FFT c0 = encrypt_bit_to_TRGSWLvl1FFT(false, skey),
                 c1 = encrypt_bit_to_TRGSWLvl1FFT(true, skey);
    write_to_archive(snapshot_path, Snapshot{skey, bkey, c0, c1});

    std::vector<std::vector<bool>> rand_bvec;
    {
        std::mt19937 rgen;
        for (size_t i = 0; i < num_test - NUM_INT2BVEC_TEST; i++)
            rand_bvec.push_back(random_bvec(rgen, num_ap));
    }

    size_t index = 0;
    std::string fml;
    while (std::getline(is, fml)) {
        std::cerr << index << "\t";
        index++;

        Graph gr = Graph::from_ltl_formula(fml, num_ap, false).minimized();
        gr.reserve_states_at_depth(MAX_INPUT_LENGTH);

        spot::twa_graph_ptr aut = ltl_to_monitor(fml, num_ap, false);

        std::vector<size_t> perm_tbl =
            create_table_to_permutate_input_bits(aut, num_ap);

        for (size_t i = 0; i < num_test; i++) {
            std::cerr << ".";
            std::vector<bool> in_src =
                                  i < NUM_INT2BVEC_TEST
                                      ? int2bvec(i + 1, num_ap)
                                      : rand_bvec.at(i - NUM_INT2BVEC_TEST),
                              in = permutate_input_bits(perm_tbl, in_src);

            bool expected = check_if_accept(aut, in_src, num_ap);

            switch (method) {
            case METHOD::OFFLINE: {
                ReversedTestInputStream input_stream{in, c0, c1};
                OfflineDFARunner runner{gr, input_stream, bkey.gkey};
                runner.eval();
                bool got = TFHEpp::tlweSymDecrypt<Lvl1>(runner.result(),
                                                        skey.key.lvl1);
                if (expected != got)
                    error::die("[{}] [{}] [{}] {} != {}", fml, i + 1,
                               bvec2str(in), expected, got);
                break;
            }

            case METHOD::ONLINE_REVERSED: {
                TestInputStream input_stream{in, c0, c1};
                OnlineDFARunner2 runner{gr, bkey.gkey};
                while (input_stream.size() != 0)
                    runner.eval_one(input_stream.next());
                bool got = TFHEpp::tlweSymDecrypt<Lvl1>(runner.result(),
                                                        skey.key.lvl1);
                if (expected != got)
                    error::die("[{}] [{}] [{}] {} != {}", fml, i + 1,
                               bvec2str(in), expected, got);
                break;
            }

            case METHOD::ONLINE_QTRLWE2: {
                TestInputStream input_stream{in, c0, c1};
                OnlineDFARunner3 runner(gr, 15, 1, *bkey.gkey,
                                        *bkey.tlwel1_trlwel1_ikskey,
                                        std::nullopt);
                while (input_stream.size() != 0)
                    runner.eval_one(input_stream.next());
                bool got = TFHEpp::tlweSymDecrypt<Lvl1>(runner.result(),
                                                        skey.key.lvl1);
                if (expected != got)
                    error::die("[{}] [{}] [{}] {} != {}", fml, i + 1,
                               bvec2str(in), expected, got);
                break;
            }
            }
        }

        std::cerr << "\n";
    }
}

int main(int argc, char** argv)
{
    std::unordered_map<std::string, METHOD> str2method = {
        {"offline", METHOD::OFFLINE},
        {"online-reversed", METHOD::ONLINE_REVERSED},
        {"online-qtrlwe2", METHOD::ONLINE_QTRLWE2},
    };

    CLI::App app{"Test crypto random"};
    std::string method, snapshot_path;
    app.add_option("method", method)
        ->required()
        ->check(
            CLI::IsMember({"offline", "online-reversed", "online-qtrlwe2"}));
    app.add_option("key-snapshot-path", snapshot_path)->required();
    CLI11_PARSE(app, argc, argv);

    test_from_ltl_formula(std::cin, 5, 20, str2method.at(method),
                          snapshot_path);
    return 0;
}
