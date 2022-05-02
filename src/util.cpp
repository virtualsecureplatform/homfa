#include "archive.hpp"
#include "error.hpp"
#include "offline_dfa.hpp"
#include "online_dfa.hpp"
#include "utility.hpp"

#include <cassert>
#include <execution>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <queue>
#include <set>
#include <sstream>
#include <thread>

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
#include <CLI/CLI.hpp>
#include <tfhe++.hpp>

std::string concat_paths(const std::string &lhs, const std::string &rhs)
{
    return std::filesystem::path{lhs} / std::filesystem::path{rhs};
}

void do_ltl2spec(const std::string &fml, size_t num_vars,
                 bool make_all_live_states_final)
{
    Graph gr =
        Graph::from_ltl_formula(fml, num_vars, make_all_live_states_final);
    gr.dump(std::cout);
}

void do_spec2spec(const std::optional<std::string> &spec_filename_opt,
                  bool minimized, bool reversed, bool negated)
{
    std::string spec_filename = spec_filename_opt.value_or("-");
    Graph gr = spec_filename == "-" ? Graph::from_istream(std::cin)
                                    : Graph::from_file(spec_filename);
    if (negated)
        gr = gr.negated();
    if (reversed)
        gr = gr.reversed();
    if (minimized)
        gr = gr.minimized();
    gr.dump(std::cout);
}

void do_spec2dot(const std::optional<std::string> &spec_filename_opt)
{
    std::string spec_filename = spec_filename_opt.value_or("-");
    if (spec_filename == "-")
        Graph::from_istream(std::cin).dump_dot(std::cout);
    else
        Graph::from_file(spec_filename).dump_dot(std::cout);
}

int main(int argc, char **argv)
{
    CLI::App app{"Homomorphic Final Answer"};
    app.require_subcommand();

    enum class TYPE {
        LTL2SPEC,
        SPEC2SPEC,
        SPEC2DOT,
    } type;

    bool verbose = false, quiet = false, minimized = false, reversed = false,
         negated, make_all_live_states_final = false, is_spec_reversed = false,
         sanitize_result = false;
    std::optional<std::string> spec, skey, bkey, input, output, output_dir,
        debug_skey;
    std::string formula, online_method = "qtrlwe2";
    std::optional<size_t> num_vars, queue_size, bootstrapping_freq,
        max_second_lut_depth;
    size_t num_ap = 0, output_freq = 1;

    app.add_flag("--verbose", verbose, "");
    app.add_flag("--quiet", quiet, "");
    {
        CLI::App *ltl2spec = app.add_subcommand(
            "ltl2spec", "Convert LTL to spec format for HomFA");
        ltl2spec->parse_complete_callback([&] { type = TYPE::LTL2SPEC; });
        ltl2spec->add_flag("--make-all-live-states-final",
                           make_all_live_states_final);
        ltl2spec->add_option("formula", formula)->required();
        ltl2spec->add_option("#vars", num_vars)->required();
    }
    {
        CLI::App *spec2spec =
            app.add_subcommand("spec2spec", "Convert spec formats for HomFA");
        spec2spec->parse_complete_callback([&] { type = TYPE::SPEC2SPEC; });
        spec2spec->add_flag("--minimized", minimized);
        spec2spec->add_flag("--reversed", reversed);
        spec2spec->add_flag("--negated", negated);
        spec2spec->add_option("SPEC-FILE", spec);
    }
    {
        CLI::App *spec2dot = app.add_subcommand(
            "spec2dot", "Convert spec format for HomFA to dot script");
        spec2dot->parse_complete_callback([&] { type = TYPE::SPEC2DOT; });
        spec2dot->add_option("SPEC-FILE", spec);
    }

    CLI11_PARSE(app, argc, argv);

    if (quiet)
        spdlog::set_level(spdlog::level::err);
    if (verbose)
        spdlog::set_level(spdlog::level::debug);

    switch (type) {
    case TYPE::LTL2SPEC:
        assert(num_vars);
        do_ltl2spec(formula, *num_vars, make_all_live_states_final);
        break;

    case TYPE::SPEC2SPEC:
        do_spec2spec(spec, minimized, reversed, negated);
        break;

    case TYPE::SPEC2DOT:
        do_spec2dot(spec);
        break;
    }

    return 0;
}
