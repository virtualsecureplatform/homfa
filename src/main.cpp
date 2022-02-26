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

void print_result(bool res)
{
    spdlog::info("Result (bool): {}", res);
    std::cout << res;  // FIXME: More appropriate or flexible way?
}

void do_genkey(const std::string &output_filename)
{
    SecretKey skey;
    write_to_archive(output_filename, skey);
}

void do_genbkey(const std::string &skey_filename,
                const std::string &output_filename)
{
    auto skey = read_from_archive<SecretKey>(skey_filename);
    BKey bkey{skey};
    write_to_archive(output_filename, bkey);
}

void do_enc(const std::string &skey_filename, const std::string &input_filename,
            const std::string &output_filename, const size_t num_ap)
{
    auto skey = read_from_archive<SecretKey>(skey_filename);

    std::ofstream ofs{output_filename};
    assert(ofs);
    TRGSWLvl1FFTSerializer ser{ofs};

    each_input_bit(input_filename, num_ap, [&](bool b) {
        ser.save(encrypt_bit_to_TRGSWLvl1FFT(b, skey));
    });
}

void do_run_offline_dfa(const std::string &spec_filename,
                        const std::string &input_filename,
                        const std::string &output_filename,
                        size_t bootstrapping_freq,
                        const std::optional<std::string> &bkey_filename,
                        bool sanitize_result)
{
    ReversedTRGSWLvl1InputStreamFromCtxtFile input_stream{input_filename};

    auto bkey = read_from_archive<BKey>(*bkey_filename);
    OfflineDFARunner runner{Graph::from_file(spec_filename).minimized(),
                            input_stream.size(), bootstrapping_freq, bkey.gkey,
                            sanitize_result};

    spdlog::info("Parameter:");
    spdlog::info("\tMode:\t{}", "Offline FA Runner");
    spdlog::info("\tInput size:\t{}", input_stream.size());
    spdlog::info("\tState size:\t{}", runner.graph().size());
    spdlog::info("\tBootstrapping frequency:\t{}", bootstrapping_freq);
    {
        size_t total_cnt_cmux = 0;
        for (size_t j = 0; j < input_stream.size(); j++)
            total_cnt_cmux += runner.graph().states_at_depth(j).size();
        spdlog::info("\tTotal #CMUX:\t{}", total_cnt_cmux);
    }
    spdlog::info("\tSanitization:\t{}", sanitize_result);
    spdlog::info("");

    size_t input_size = input_stream.size();
    for (size_t i = 0; i < input_size; i++)
        runner.eval_one(input_stream.next());

    write_to_archive(output_filename, runner.result());
}

void do_run_online_dfa(const std::string &spec_filename,
                       const std::string &input_filename,
                       const std::string &output_filename,
                       const std::optional<std::string> &bkey_filename,
                       bool sanitize_result)
{
    TRGSWLvl1InputStreamFromCtxtFile input_stream{input_filename};
    Graph gr = Graph::from_file(spec_filename);
    auto bkey = read_from_archive<BKey>(*bkey_filename);
    OnlineDFARunner runner{gr, bkey.gkey, sanitize_result};

    spdlog::info("Parameter:");
    spdlog::info("\tMode:\t{}", "Online FA Runner1 (qtrlwe)");
    spdlog::info("\tState size:\t{}", gr.size());
    // spdlog::info("\tBootstrap interval:\t{}", bootstrap_interval_);
    spdlog::info("\tSanitization:\t{}", sanitize_result);
    spdlog::info("");

    for (size_t i = 0; input_stream.size() != 0; i++) {
        spdlog::debug("Processing input {}", i);
        runner.eval_one(input_stream.next());
    }

    write_to_archive(output_filename, runner.result());
}

void do_run_online_dfa2(const std::string &spec_filename,
                        const std::string &input_filename,
                        const std::optional<std::string> &output_filename,
                        const std::optional<std::string> &output_dirname,
                        size_t output_freq, size_t bootstrapping_freq,
                        bool is_spec_reversed,
                        const std::optional<std::string> &bkey_filename,
                        bool sanitize_result)
{
    assert((output_filename && !output_dirname) ||
           (!output_filename && output_dirname));

    TRGSWLvl1InputStreamFromCtxtFile input_stream{input_filename};
    auto bkey = read_from_archive<BKey>(*bkey_filename);
    OnlineDFARunner2 runner{Graph::from_file(spec_filename), bootstrapping_freq,
                            is_spec_reversed, bkey.gkey, sanitize_result};

    spdlog::info("Parameter:");
    spdlog::info("\tMode:\t{}", "Online FA Runner2 (reversed)");
    spdlog::info("\tInput size:\t{} (hidden)", input_stream.size());
    spdlog::info("\tState size:\t{}", runner.graph().size());
    if (output_filename)
        spdlog::info("\tOutput file name:\t{}", *output_filename);
    if (output_dirname) {
        spdlog::info("\tOutput directory:\t{}", *output_dirname);
        spdlog::info("\tOutput frequency:\t{}", output_freq);
    }
    spdlog::info("\tBootstrapping frequency:\t{}", bootstrapping_freq);
    spdlog::info("\tSanitization:\t{}", sanitize_result);
    spdlog::info("");

    if (output_dirname)
        std::filesystem::create_directory(*output_dirname);

    size_t input_stream_size_hidden = input_stream.size();
    for (size_t i = 0; input_stream.size() != 0; i++) {
        spdlog::debug("Processing input {}", i);
        runner.eval_one(input_stream.next());

        if (output_dirname && i % output_freq == output_freq - 1) {
            const std::string path =
                concat_paths(*output_dirname, fmt::format("{}.out", i + 1));
            write_to_archive(path, runner.result());
        }
    }

    if (output_filename)
        write_to_archive(*output_filename, runner.result());
    else {
        const std::string path = concat_paths(
            *output_dirname, fmt::format("{}.out", input_stream_size_hidden));
        write_to_archive(path, runner.result());
    }
}

void do_run_online_dfa3(const std::string &spec_filename,
                        const std::string &input_filename,
                        const std::optional<std::string> &output_filename,
                        const std::optional<std::string> &output_dirname,
                        size_t output_freq, size_t queue_size,
                        size_t bootstrapping_freq,
                        const std::string &bkey_filename,
                        const std::optional<size_t> &max_second_lut_depth,
                        const std::optional<std::string> &debug_skey_filename,
                        bool sanitize_result)
{
    TRGSWLvl1InputStreamFromCtxtFile input_stream{input_filename};
    Graph gr = Graph::from_file(spec_filename);

    auto bkey = read_from_archive<BKey>(bkey_filename);
    assert(bkey.gkey && bkey.tlwel1_trlwel1_ikskey);

    std::optional<SecretKey> debug_skey;
    if (debug_skey_filename)
        debug_skey.emplace(read_from_archive<SecretKey>(*debug_skey_filename));

    OnlineDFARunner3 runner{gr,         max_second_lut_depth.value_or(8),
                            queue_size, bootstrapping_freq,
                            *bkey.gkey, *bkey.tlwel1_trlwel1_ikskey,
                            debug_skey, sanitize_result};

    spdlog::info("Parameter:");
    spdlog::info("\tMode:\t{}", "Online FA Runner3 (qtrlwe2)");
    spdlog::info("\tInput size:\t{} (hidden)", input_stream.size());
    spdlog::info("\tState size:\t{}", gr.size());
    spdlog::info("\tQueue size:\t{}", runner.queue_size());
    if (output_filename)
        spdlog::info("\tOutput file name:\t{}", *output_filename);
    if (output_dirname) {
        spdlog::info("\tOutput directory:\t{}", *output_dirname);
        spdlog::info("\tOutput frequency:\t{}", output_freq);
    }
    spdlog::info("\tBootstrapping frequency:\t{}", bootstrapping_freq);
    spdlog::info("\tSanitization:\t{}", sanitize_result);
    spdlog::info("");

    if (output_dirname && output_freq % queue_size != 0)
        spdlog::warn("Output frequency ({}) is not the same as queue size ({})",
                     output_freq, queue_size);

    if (output_dirname)
        std::filesystem::create_directory(*output_dirname);

    size_t input_stream_size_hidden = input_stream.size();
    for (size_t i = 0; input_stream.size() != 0; i++) {
        spdlog::debug("Processing input {}", i);
        runner.eval_one(input_stream.next());

        if (output_dirname && i % output_freq == output_freq - 1) {
            const std::string path =
                concat_paths(*output_dirname, fmt::format("{}.out", i + 1));
            write_to_archive(path, runner.result());
        }
    }

    if (output_filename)
        write_to_archive(*output_filename, runner.result());
    else {
        const std::string path = concat_paths(
            *output_dirname, fmt::format("{}.out", input_stream_size_hidden));
        write_to_archive(path, runner.result());
    }
}

void do_run_online_dfa4(const std::string &spec_filename,
                        const std::string &input_filename,
                        const std::string &output_filename, size_t queue_size,
                        const std::string &bkey_filename, bool sanitize_result)
{
    TRGSWLvl1InputStreamFromCtxtFile input_stream{input_filename};
    Graph gr = Graph::from_file(spec_filename);

    auto bkey = read_from_archive<BKey>(bkey_filename);
    assert(bkey.gkey);

    OnlineDFARunner4 runner{gr, queue_size, *bkey.gkey, *bkey.circuit_key,
                            sanitize_result};

    spdlog::info("Parameter:");
    spdlog::info("\tMode:\t{}", "Online FA Runner4 (block-backstream)");
    spdlog::info("\tInput size:\t{} (hidden)", input_stream.size());
    spdlog::info("\tState size:\t{}", gr.size());
    spdlog::info("\tQueue size:\t{}", runner.queue_size());
    spdlog::info("\tSanitization:\t{}", sanitize_result);
    spdlog::info("");

    for (size_t i = 0; input_stream.size() != 0; i++) {
        spdlog::debug("Processing input {}", i);
        runner.eval_one(input_stream.next());
    }
    TLWELvl1 res = runner.result();

    write_to_archive(output_filename, res);
}

void do_dec(const std::string &skey_filename, const std::string &input_filename)
{
    auto skey = read_from_archive<SecretKey>(skey_filename);
    auto enc_res = read_from_archive<TLWELvl1>(input_filename);
    bool res = decrypt_TLWELvl1_to_bit(enc_res, skey);
    print_result(res);
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

void do_att2spec(const std::optional<std::string> &att_filename_opt)
{
    std::string att_filename = att_filename_opt.value_or("-");
    if (att_filename == "-")
        Graph::from_att_istream(std::cin).dump(std::cout);
    else
        Graph::from_att_file(att_filename).dump(std::cout);
}

void do_spec2att(const std::optional<std::string> &spec_filename_opt)
{
    std::string spec_filename = spec_filename_opt.value_or("-");
    if (spec_filename == "-")
        Graph::from_istream(std::cin).dump_att(std::cout);
    else
        Graph::from_file(spec_filename).dump_att(std::cout);
}

void do_run_dfa_plain(const std::string &spec_filename,
                      const std::string &input_filename, size_t num_ap)
{
    Graph gr = Graph::from_file(spec_filename);
    auto dfa_state = gr.initial_state();

    each_input_bit(input_filename, num_ap,
                   [&](bool b) { dfa_state = gr.next_state(dfa_state, b); });

    bool res = gr.is_final_state(dfa_state);
    print_result(res);
}

namespace {
void dumpBasicInfo(int argc, char **argv)
{
    // Show build config
    spdlog::info("Built with:");
#if defined(HOMFA_BUILD_DEBUG)
    spdlog::info("\tType: debug");
#else
    spdlog::info("\tType: release");
#endif
#if defined(HOMFA_GIT_REVISION)
    spdlog::info("\tGit revision: " HOMFA_GIT_REVISION);
#else
    spdlog::info("\tGit revision: unknown");
#endif
#if defined(HOMFA_ENABLE_PROFILE)
    spdlog::info("\tProfiling: enabled");
#else
    spdlog::info("\tProfiling: disabled");
#endif

    // Show execution setting
    spdlog::info("Executed with:");
    {
        std::stringstream ss;
        for (int i = 0; i < argc; i++)
            ss << argv[i] << " ";
        spdlog::info("\tArgs: {}", ss.str());
    }
    {
        std::stringstream ss;
        if (char *envvar = std::getenv("CPUPROFILE"); envvar != nullptr)
            ss << "CPUPROFILE=" << envvar << " ";
        if (char *envvar = std::getenv("HEAPPROFILE"); envvar != nullptr)
            ss << "HEAPPROFILE=" << envvar << " ";
        spdlog::info("\tEnv var: {}", ss.str());
    }
    spdlog::info("\tConcurrency:\t{}", std::thread::hardware_concurrency());
}
}  // namespace

int main(int argc, char **argv)
{
    error::initialize("homfa");
    dumpBasicInfo(argc, argv);

    CLI::App app{"Homomorphic Final Answer"};
    app.require_subcommand();

    enum class TYPE {
        GENKEY,
        GENBKEY,
        ENC,
        RUN_OFFLINE_DFA,
        RUN_ONLINE_DFA,
        DEC,
        LTL2SPEC,
        SPEC2SPEC,
        SPEC2DOT,
        ATT2SPEC,
        SPEC2ATT,
        RUN_DFA_PLAIN,
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
        CLI::App *genkey = app.add_subcommand("genkey", "Generate secret key");
        genkey->parse_complete_callback([&] { type = TYPE::GENKEY; });
        genkey->add_option("--out", output)->required();
    }
    {
        CLI::App *genbkey = app.add_subcommand(
            "genbkey", "Generate bootstrapping key from secret key");
        genbkey->parse_complete_callback([&] { type = TYPE::GENBKEY; });
        genbkey->add_option("--key", skey)
            ->required()
            ->check(CLI::ExistingFile);
        genbkey->add_option("--out", output)->required();
    }
    {
        CLI::App *enc = app.add_subcommand("enc", "Encrypt input file");
        enc->parse_complete_callback([&] { type = TYPE::ENC; });
        enc->add_option("--ap", num_ap)->required()->check(CLI::PositiveNumber);
        enc->add_option("--key", skey)->required()->check(CLI::ExistingFile);
        enc->add_option("--in", input)->required()->check(CLI::ExistingFile);
        enc->add_option("--out", output)->required();
    }
    {
        CLI::App *run =
            app.add_subcommand("run-offline-dfa", "Run offline DFA");
        run->parse_complete_callback([&] { type = TYPE::RUN_OFFLINE_DFA; });
        run->add_option("--bkey", bkey)->check(CLI::ExistingFile);
        run->add_option("--spec", spec)->required()->check(CLI::ExistingFile);
        run->add_option("--in", input)->required()->check(CLI::ExistingFile);
        run->add_option("--out", output)->required();
        run->add_option("--bootstrapping-freq", bootstrapping_freq)
            ->check(CLI::PositiveNumber);
        run->add_option("--sanitize-result", sanitize_result);
    }
    {
        CLI::App *run = app.add_subcommand("run-online-dfa", "Run online DFA");
        run->parse_complete_callback([&] { type = TYPE::RUN_ONLINE_DFA; });
        run->add_option("--bkey", bkey)->check(CLI::ExistingFile);
        run->add_option("--spec", spec)->required()->check(CLI::ExistingFile);
        run->add_option("--in", input)->required()->check(CLI::ExistingFile);
        run->add_option("--out", output);
        run->add_option("--out-dir", output_dir);
        run->add_option("--out-freq", output_freq)->check(CLI::PositiveNumber);
        run->add_option("--method", online_method)
            ->check(CLI::IsMember(
                {"qtrlwe", "reversed", "qtrlwe2", "block-backstream"}));
        run->add_option("--queue-size", queue_size)->check(CLI::PositiveNumber);
        run->add_option("--bootstrapping-freq", bootstrapping_freq)
            ->check(CLI::PositiveNumber);
        run->add_option("--debug-secret-key", debug_skey)
            ->check(CLI::ExistingFile);
        run->add_option("--max-second-lut-depth", max_second_lut_depth)
            ->check(CLI::PositiveNumber);
        run->add_flag("--spec-reversed", is_spec_reversed);
        run->add_option("--sanitize-result", sanitize_result);
    }
    {
        CLI::App *dec = app.add_subcommand("dec", "Decrypt input file");
        dec->parse_complete_callback([&] { type = TYPE::DEC; });
        dec->add_option("--key", skey)->required()->check(CLI::ExistingFile);
        dec->add_option("--in", input)->required()->check(CLI::ExistingFile);
    }
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
    {
        CLI::App *spec2att = app.add_subcommand(
            "spec2att", "Convert spec format for HomFA to AT&T format");
        spec2att->parse_complete_callback([&] { type = TYPE::SPEC2ATT; });
        spec2att->add_option("SPEC-FILE", spec);
    }
    {
        CLI::App *att2spec = app.add_subcommand(
            "att2spec", "Convert AT&T format to spec format for HomFA");
        att2spec->parse_complete_callback([&] { type = TYPE::ATT2SPEC; });
        att2spec->add_option("ATT-SPEC-FILE", spec);
    }
    {
        CLI::App *run_plain =
            app.add_subcommand("run-dfa-plain", "Run DFA on plain text");
        run_plain->parse_complete_callback([&] { type = TYPE::RUN_DFA_PLAIN; });
        run_plain->add_option("--ap", num_ap)
            ->required()
            ->check(CLI::PositiveNumber);
        run_plain->add_option("--spec", spec)
            ->required()
            ->check(CLI::ExistingFile);
        run_plain->add_option("--in", input)
            ->required()
            ->check(CLI::ExistingFile);
    }

    CLI11_PARSE(app, argc, argv);

    if (quiet)
        spdlog::set_level(spdlog::level::err);
    if (verbose)
        spdlog::set_level(spdlog::level::debug);

    switch (type) {
    case TYPE::GENKEY:
        assert(output);
        do_genkey(*output);
        break;

    case TYPE::GENBKEY:
        assert(skey && output);
        do_genbkey(*skey, *output);
        break;

    case TYPE::ENC:
        assert(skey && input && output);
        do_enc(*skey, *input, *output, num_ap);
        break;

    case TYPE::RUN_OFFLINE_DFA:
        assert(spec && input && output);
        do_run_offline_dfa(*spec, *input, *output,
                           bootstrapping_freq.value_or(8000), bkey,
                           sanitize_result);
        break;

    case TYPE::RUN_ONLINE_DFA:
        assert(spec && input);

        if (!((output && !output_dir) || (!output && output_dir)))
            error::die("Use --out or --out-dir");

        if (online_method == "qtrlwe") {
            spdlog::warn("FIXME: not support --out-dir and --output-freq");
            assert(output);
            do_run_online_dfa(*spec, *input, *output, bkey, sanitize_result);
        }
        else if (online_method == "reversed") {
            do_run_online_dfa2(*spec, *input, output, output_dir, output_freq,
                               bootstrapping_freq.value_or(8000),
                               is_spec_reversed, bkey, sanitize_result);
        }
        else if (online_method == "block-backstream") {
            do_run_online_dfa4(*spec, *input, *output, queue_size.value_or(100),
                               *bkey, sanitize_result);
        }
        else {
            assert(online_method == "qtrlwe2");
            assert(bkey);
            do_run_online_dfa3(
                *spec, *input, output, output_dir, output_freq,
                queue_size.value_or(15), bootstrapping_freq.value_or(1), *bkey,
                max_second_lut_depth, debug_skey, sanitize_result);
        }
        break;

    case TYPE::DEC:
        assert(skey && input);
        do_dec(*skey, *input);
        break;

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

    case TYPE::SPEC2ATT:
        do_spec2att(spec);
        break;

    case TYPE::ATT2SPEC:
        do_att2spec(spec);
        break;

    case TYPE::RUN_DFA_PLAIN:
        assert(spec && input);
        do_run_dfa_plain(*spec, *input, num_ap);
        break;
    }

    return 0;
}
