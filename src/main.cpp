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

namespace {
enum class VERBOSITY { VERBOSE, NORMAL, QUIET };

enum class TYPE {
    UNSPECIFIED,

    GENKEY,
    GENBKEY,

    ENC,
    DEC,

    RUN_OFFLINE,
    RUN_REVERSE,
    RUN_BLOCK,
    RUN_FLUT,
    RUN_PLAIN,

    BENCH_OFFLINE,
    BENCH_REVERSE,
    BENCH_BLOCK,
    BENCH_FLUT,
    BENCH_PLAIN,

    LTL2SPEC,
    SPEC2SPEC,
    SPEC2DOT,
    ATT2SPEC,
    SPEC2ATT,
};

struct Args {
    VERBOSITY verbosity = VERBOSITY::NORMAL;
    TYPE type = TYPE::UNSPECIFIED;

    bool minimized = false, reversed = false, negated = false,
         make_all_live_states_final = false, is_spec_reversed = false,
         sanitize_result = false;
    std::optional<std::string> spec, skey, bkey, input, output, output_dir,
        debug_skey, formula, online_method;
    std::optional<size_t> num_vars, queue_size, bootstrapping_freq,
        max_second_lut_depth, num_ap, output_freq;
};

void register_general_options(CLI::App &app, Args &args)
{
    app.add_flag_callback("--verbose",
                          [&] { args.verbosity = VERBOSITY::VERBOSE; });
    app.add_flag_callback("--quiet",
                          [&] { args.verbosity = VERBOSITY::QUIET; });
}

void register_genkey(CLI::App &app, Args &args)
{
    CLI::App *genkey = app.add_subcommand("genkey", "Generate secret key");
    genkey->parse_complete_callback([&args] { args.type = TYPE::GENKEY; });
    genkey->add_option("--out", args.output)->required();
}

void register_genbkey(CLI::App &app, Args &args)
{
    CLI::App *genbkey = app.add_subcommand(
        "genbkey", "Generate bootstrapping key from secret key");
    genbkey->parse_complete_callback([&args] { args.type = TYPE::GENBKEY; });
    genbkey->add_option("--key", args.skey)
        ->required()
        ->check(CLI::ExistingFile);
    genbkey->add_option("--out", args.output)->required();
}

void register_enc(CLI::App &app, Args &args)
{
    CLI::App *enc = app.add_subcommand("enc", "Encrypt input file");
    enc->parse_complete_callback([&args] { args.type = TYPE::ENC; });
    enc->add_option("--ap", args.num_ap)
        ->required()
        ->check(CLI::PositiveNumber);
    enc->add_option("--key", args.skey)->required()->check(CLI::ExistingFile);
    enc->add_option("--in", args.input)->required()->check(CLI::ExistingFile);
    enc->add_option("--out", args.output)->required();
}

void register_dec(CLI::App &app, Args &args)
{
    CLI::App *dec = app.add_subcommand("dec", "Decrypt input file");
    dec->parse_complete_callback([&args] { args.type = TYPE::DEC; });
    dec->add_option("--key", args.skey)->required()->check(CLI::ExistingFile);
    dec->add_option("--in", args.input)->required()->check(CLI::ExistingFile);
}

void add_run_common_options(CLI::App *run, Args &args, bool benchmark)
{
    if (benchmark) {
        run->add_option("--ap", args.num_ap)
            ->required()
            ->check(CLI::PositiveNumber);
    }
    else {  // not benchmark
        run->add_option("--bkey", args.bkey)
            ->required()
            ->check(CLI::ExistingFile);
        run->add_option("--out", args.output)->required();
    }

    run->add_option("--spec", args.spec)->required()->check(CLI::ExistingFile);
    run->add_option("--in", args.input)->required()->check(CLI::ExistingFile);
    run->add_option("--debug-secret-key", args.debug_skey)
        ->check(CLI::ExistingFile);
}

void register_offline(CLI::App &app, Args &args, bool benchmark)
{
    CLI::App *run = app.add_subcommand("offline", "Run OFFLINE algorithm");
    add_run_common_options(run, args, benchmark);
    run->parse_complete_callback([&args, benchmark] {
        args.type = benchmark ? TYPE::BENCH_OFFLINE : TYPE::RUN_OFFLINE;
    });
    run->add_option("--bootstrapping-freq", args.bootstrapping_freq)
        ->required()
        ->check(CLI::PositiveNumber);
}

void register_reverse(CLI::App &app, Args &args, bool benchmark)
{
    CLI::App *run = app.add_subcommand("reverse", "Run REVERSE algorithm");
    run->alias("reversed");
    add_run_common_options(run, args, benchmark);
    run->parse_complete_callback([&args, benchmark] {
        args.type = benchmark ? TYPE::BENCH_REVERSE : TYPE::RUN_REVERSE;
    });
    run->add_option("--out-freq", args.output_freq)
        ->required()
        ->check(CLI::PositiveNumber);
    run->add_option("--bootstrapping-freq", args.bootstrapping_freq)
        ->required()
        ->check(CLI::PositiveNumber);
    run->add_flag("--spec-reversed", args.is_spec_reversed);
}

void register_block(CLI::App &app, Args &args, bool benchmark)
{
    CLI::App *run = app.add_subcommand("block", "Run BLOCK algorithm");
    run->alias("bbs");
    run->alias("block-backstream");
    add_run_common_options(run, args, benchmark);
    run->parse_complete_callback([&args, benchmark] {
        args.type = benchmark ? TYPE::BENCH_BLOCK : TYPE::RUN_BLOCK;
    });
    run->add_option("--out-freq", args.output_freq)
        ->required()
        ->check(CLI::PositiveNumber);
    run->add_option("--queue-size", args.queue_size)
        ->required()
        ->check(CLI::PositiveNumber);
}

void register_flut(CLI::App &app, Args &args, bool benchmark)
{
    CLI::App *run = app.add_subcommand("flut", "Run FLUT algorithm");
    run->alias("qtrlwe2");
    add_run_common_options(run, args, benchmark);
    run->parse_complete_callback([&args, benchmark] {
        args.type = benchmark ? TYPE::BENCH_FLUT : TYPE::RUN_FLUT;
    });
    run->add_option("--out-freq", args.output_freq)
        ->required()
        ->check(CLI::PositiveNumber);
    run->add_option("--queue-size", args.queue_size)
        ->required()
        ->check(CLI::PositiveNumber);
    run->add_option("--max-second-lut-depth", args.max_second_lut_depth)
        ->required()
        ->check(CLI::PositiveNumber);
    run->add_option("--bootstrapping-freq", args.bootstrapping_freq)
        ->required()
        ->check(CLI::PositiveNumber);
}

void register_plain(CLI::App &app, Args &args, bool benchmark)
{
    CLI::App *run = app.add_subcommand("plain", "Run DFA on plain text");
    run->parse_complete_callback([&args, benchmark] {
        args.type = benchmark ? TYPE::BENCH_PLAIN : TYPE::RUN_PLAIN;
    });
    run->add_option("--ap", args.num_ap)
        ->required()
        ->check(CLI::PositiveNumber);
    run->add_option("--spec", args.spec)->required()->check(CLI::ExistingFile);
    run->add_option("--in", args.input)->required()->check(CLI::ExistingFile);
}

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

void do_run_offline(const std::string &spec_filename,
                    const std::string &input_filename,
                    const std::string &output_filename,
                    size_t bootstrapping_freq, const std::string &bkey_filename,
                    bool sanitize_result)
{
    ReversedTRGSWLvl1InputStreamFromCtxtFile input_stream{input_filename};

    auto bkey = read_from_archive<BKey>(bkey_filename);
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

/*
void do_run_online_dfa(const std::string &spec_filename,
                       const std::string &input_filename,
                       const std::string &output_filename,
                       const std::string &bkey_filename, bool sanitize_result)
{
    TRGSWLvl1InputStreamFromCtxtFile input_stream{input_filename};
    Graph gr = Graph::from_file(spec_filename);
    auto bkey = read_from_archive<BKey>(bkey_filename);
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
*/

void do_run_reverse(const std::string &spec_filename,
                    const std::string &input_filename,
                    const std::optional<std::string> &output_filename,
                    const std::optional<std::string> &output_dirname,
                    size_t output_freq, size_t bootstrapping_freq,
                    bool is_spec_reversed, const std::string &bkey_filename,
                    bool sanitize_result)
{
    assert((output_filename && !output_dirname) ||
           (!output_filename && output_dirname));

    TRGSWLvl1InputStreamFromCtxtFile input_stream{input_filename};
    auto bkey = read_from_archive<BKey>(bkey_filename);
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

void do_run_flut(const std::string &spec_filename,
                 const std::string &input_filename,
                 const std::optional<std::string> &output_filename,
                 const std::optional<std::string> &output_dirname,
                 size_t output_freq, size_t queue_size,
                 size_t bootstrapping_freq, const std::string &bkey_filename,
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

void do_run_block(const std::string &spec_filename,
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

void dumpBasicInfo(int argc, char **argv)
{
    spdlog::info(R"(===================================)");

    // Logo: thanks to:
    // https://patorjk.com/software/taag/#p=display&f=Standard&t=HomFA
    spdlog::info(R"(  _   _                 _____ _    )");
    spdlog::info(R"( | | | | ___  _ __ ___ |  ___/ \   )");
    spdlog::info(R"( | |_| |/ _ \| '_ ` _ \| |_ / _ \  )");
    spdlog::info(R"( |  _  | (_) | | | | | |  _/ ___ \ )");
    spdlog::info(R"( |_| |_|\___/|_| |_| |_|_|/_/   \_\)");
    spdlog::info(R"(                                   )");

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

    spdlog::info(R"(===================================)");
}
}  // namespace

int main(int argc, char **argv)
{
    error::initialize("homfa");

    Args args;
    CLI::App app{
        "HomFA -- Oblivious Online LTL Monitor via Fully Homomorphic "
        "Encryption"};
    register_general_options(app, args);
    register_genkey(app, args);
    register_genbkey(app, args);
    register_enc(app, args);
    register_dec(app, args);
    {
        CLI::App *run = app.add_subcommand("run", "Run DFA over ciphertexts");
        register_offline(*run, args, false);
        register_reverse(*run, args, false);
        register_block(*run, args, false);
        register_flut(*run, args, false);
        register_plain(*run, args, false);
    }
    {
        CLI::App *ltl2spec = app.add_subcommand(
            "ltl2spec", "Convert LTL to spec format for HomFA");
        ltl2spec->parse_complete_callback([&] { args.type = TYPE::LTL2SPEC; });
        ltl2spec->add_flag("--make-all-live-states-final",
                           args.make_all_live_states_final);
        ltl2spec->add_option("formula", args.formula)->required();
        ltl2spec->add_option("#vars", args.num_vars)->required();
    }
    {
        CLI::App *spec2spec =
            app.add_subcommand("spec2spec", "Convert spec formats for HomFA");
        spec2spec->parse_complete_callback(
            [&] { args.type = TYPE::SPEC2SPEC; });
        spec2spec->add_flag("--minimized", args.minimized);
        spec2spec->add_flag("--reversed", args.reversed);
        spec2spec->add_flag("--negated", args.negated);
        spec2spec->add_option("SPEC-FILE", args.spec);
    }
    {
        CLI::App *spec2dot = app.add_subcommand(
            "spec2dot", "Convert spec format for HomFA to dot script");
        spec2dot->parse_complete_callback([&] { args.type = TYPE::SPEC2DOT; });
        spec2dot->add_option("SPEC-FILE", args.spec);
    }
    {
        CLI::App *spec2att = app.add_subcommand(
            "spec2att", "Convert spec format for HomFA to AT&T format");
        spec2att->parse_complete_callback([&] { args.type = TYPE::SPEC2ATT; });
        spec2att->add_option("SPEC-FILE", args.spec);
    }
    {
        CLI::App *att2spec = app.add_subcommand(
            "att2spec", "Convert AT&T format to spec format for HomFA");
        att2spec->parse_complete_callback([&] { args.type = TYPE::ATT2SPEC; });
        att2spec->add_option("ATT-SPEC-FILE", args.spec);
    }

    CLI11_PARSE(app, argc, argv);

    switch (args.verbosity) {
    case VERBOSITY::QUIET:
        spdlog::set_level(spdlog::level::err);
        break;
    case VERBOSITY::VERBOSE:
        spdlog::set_level(spdlog::level::debug);
        break;
    default:;
    }

    dumpBasicInfo(argc, argv);

    switch (args.type) {
    case TYPE::GENKEY:
        do_genkey(args.output.value());
        break;

    case TYPE::GENBKEY:
        do_genbkey(args.skey.value(), args.output.value());
        break;

    case TYPE::ENC:
        do_enc(args.skey.value(), args.input.value(), args.output.value(),
               args.num_ap.value());
        break;

    case TYPE::DEC:
        do_dec(args.skey.value(), args.input.value());
        break;

    case TYPE::RUN_OFFLINE:
        do_run_offline(args.spec.value(), args.input.value(),
                       args.output.value(), args.bootstrapping_freq.value(),
                       args.bkey.value(), args.sanitize_result);
        break;

    case TYPE::RUN_REVERSE:
        if (!((args.output && !args.output_dir) ||
              (!args.output && args.output_dir)))
            error::die("Use --out or --out-dir");
        do_run_reverse(args.spec.value(), args.input.value(), args.output,
                       args.output_dir, args.output_freq.value(),
                       args.bootstrapping_freq.value(), args.is_spec_reversed,
                       args.bkey.value(), args.sanitize_result);
        break;

    case TYPE::RUN_BLOCK:
        if (!((args.output && !args.output_dir) ||
              (!args.output && args.output_dir)))
            error::die("Use --out or --out-dir");
        do_run_block(args.spec.value(), args.input.value(), args.output.value(),
                     args.queue_size.value(), args.bkey.value(),
                     args.sanitize_result);
        break;

    case TYPE::RUN_FLUT:
        if (!((args.output && !args.output_dir) ||
              (!args.output && args.output_dir)))
            error::die("Use --out or --out-dir");
        do_run_flut(args.spec.value(), args.input.value(), args.output,
                    args.output_dir, args.output_freq.value(),
                    args.queue_size.value(), args.bootstrapping_freq.value(),
                    args.bkey.value(), args.max_second_lut_depth.value(),
                    args.debug_skey, args.sanitize_result);
        break;

    case TYPE::RUN_PLAIN:
        do_run_dfa_plain(args.spec.value(), args.input.value(),
                         args.num_ap.value());
        break;

    case TYPE::LTL2SPEC:
        do_ltl2spec(args.formula.value(), args.num_vars.value(),
                    args.make_all_live_states_final);
        break;

    case TYPE::SPEC2SPEC:
        do_spec2spec(args.spec, args.minimized, args.reversed, args.negated);
        break;

    case TYPE::SPEC2DOT:
        do_spec2dot(args.spec);
        break;

    case TYPE::SPEC2ATT:
        do_spec2att(args.spec);
        break;

    case TYPE::ATT2SPEC:
        do_att2spec(args.spec);
        break;

    case TYPE::BENCH_OFFLINE:
    case TYPE::BENCH_REVERSE:
    case TYPE::BENCH_BLOCK:
    case TYPE::BENCH_FLUT:
    case TYPE::BENCH_PLAIN:
        error::die("Not implemented for now");

    case TYPE::UNSPECIFIED:
        error::die("Please specify subcommand");
    }

    return 0;
}
