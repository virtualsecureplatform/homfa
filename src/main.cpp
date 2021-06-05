#include "archive.hpp"
#include "error.hpp"
#include "offline_dfa.hpp"
#include "online_dfa.hpp"

#include <cassert>
#include <execution>
#include <fstream>
#include <iostream>
#include <queue>
#include <set>
#include <sstream>
#include <thread>

#include <spdlog/spdlog.h>
#include <CLI/CLI.hpp>
#include <tfhe++.hpp>

void do_genkey(const std::string &output_filename)
{
    SecretKey skey;
    write_to_archive(output_filename, skey);
}

void do_genbkey(const std::string &skey_filename,
                const std::string &output_filename)
{
    auto skey = read_from_archive<SecretKey>(skey_filename);
    CloudKey bkey{skey};
    write_to_archive(output_filename, bkey);
}

void do_enc(const std::string &skey_filename, const std::string &input_filename,
            const std::string &output_filename)
{
    auto skey = read_from_archive<SecretKey>(skey_filename);

    // FIXME: We essentially don't need cuFHE for NTT.
    auto gkey = std::make_shared<TFHEpp::GateKeywoFFT>(skey);
    cufhe::Initialize(*gkey);

    std::ifstream ifs{input_filename};
    assert(ifs);
    EncryptedInput ret;
    cufhe::cuFHETRGSWNTTlvl1 tmp;
    while (ifs) {
        int ch = ifs.get();
        if (ch == EOF)
            break;
        for (int i = 0; i < 8; i++) {
            bool b = ((static_cast<uint8_t>(ch) >> i) & 1u) != 0;
            TRGSWLvl1 c;
            encrypt_bit_to_TRGSWLvl1(c, b, skey);

            ret.trgsw_fft.emplace_back();
            fft_TRGSWLvl1(ret.trgsw_fft.back(), c);

            ret.trgsw_ntt.emplace_back();
            // ntt_TRGSWLvl1(ret.trgsw_ntt.back(), c); // FIXME
            cufhe::TRGSW2NTT(tmp, c, 0);
            ret.trgsw_ntt.back() = tmp.trgswhost;
        }
    }

    write_to_archive(output_filename, ret);

    cufhe::CleanUp();
}

void do_run_offline_dfa(
    const std::string &spec_filename, const std::string &input_filename,
    const std::string &output_filename, const std::optional<int> &gpu,
    const std::optional<std::string> &bkey_filename = std::nullopt)
{
    ReversedTRGSWLvl1InputStreamFromCtxtFile input_stream{input_filename};

    Graph gr{spec_filename};
    gr.reserve_states_at_depth(input_stream.size());

    CloudKey bkey;
    if (bkey_filename)
        bkey = read_from_archive<CloudKey>(*bkey_filename);

    bool num_gpus = gpu.value_or(0);
    if (num_gpus != 0) {
        cufhe::SetGPUNum(num_gpus);
        cufhe::Initialize(*bkey.gk);
    }

    OfflineDFARunner runner{gr, input_stream, num_gpus != 0, bkey.gkfft};
    runner.eval();

    write_to_archive(output_filename, runner.result());
}

void do_run_online_dfa(
    const std::string &spec_filename, const std::string &input_filename,
    const std::string &output_filename,
    const std::optional<std::string> &bkey_filename = std::nullopt)
{
    TRGSWLvl1InputStreamFromCtxtFile input_stream{input_filename};
    Graph gr{spec_filename};
    CloudKey bkey;
    if (bkey_filename)
        bkey = read_from_archive<CloudKey>(*bkey_filename);
    OnlineDFARunner runner{gr, bkey.gkfft};

    for (size_t i = 0; input_stream.size() != 0; i++) {
        spdlog::debug("Processing input {}", i);

        TRGSWLvl1FFT in;
        input_stream.next(in);
        runner.eval_one(in);
    }

    write_to_archive(output_filename, runner.result());
}

void do_dec(const std::string &skey_filename, const std::string &input_filename)
{
    auto skey = read_from_archive<SecretKey>(skey_filename);
    auto enc_res = read_from_archive<TLWELvl1>(input_filename);
    bool res = TFHEpp::tlweSymDecrypt<Lvl1>(enc_res, skey.key.lvl1);
    spdlog::info("Result (bool): {}", res);
}

int main(int argc, char **argv)
{
    CLI::App app{"Homomorphic Final Answer"};
    app.require_subcommand();

    enum class TYPE {
        GENKEY,
        GENBKEY,
        ENC,
        RUN_OFFLINE_DFA,
        RUN_ONLINE_DFA,
        DEC,
    } type;

    bool verbose = false, quiet = false;
    std::optional<std::string> spec, skey, bkey, input, output;
    std::optional<int> gpu;

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
        run->add_option("--gpu", gpu);
    }
    {
        CLI::App *run = app.add_subcommand("run-online-dfa", "Run online DFA");
        run->parse_complete_callback([&] { type = TYPE::RUN_ONLINE_DFA; });
        run->add_option("--bkey", bkey)->check(CLI::ExistingFile);
        run->add_option("--spec", spec)->required()->check(CLI::ExistingFile);
        run->add_option("--in", input)->required()->check(CLI::ExistingFile);
        run->add_option("--out", output)->required();
    }
    {
        CLI::App *dec = app.add_subcommand("dec", "Decrypt input file");
        dec->parse_complete_callback([&] { type = TYPE::DEC; });
        dec->add_option("--key", skey)->required()->check(CLI::ExistingFile);
        dec->add_option("--in", input)->required()->check(CLI::ExistingFile);
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
        do_enc(*skey, *input, *output);
        break;

    case TYPE::RUN_OFFLINE_DFA:
        assert(spec && input && output);
        assert(!gpu || bkey);
        // FIXME: gpu
        do_run_offline_dfa(*spec, *input, *output, gpu, bkey);
        break;

    case TYPE::RUN_ONLINE_DFA:
        assert(spec && input && output);
        do_run_online_dfa(*spec, *input, *output, bkey);
        break;

    case TYPE::DEC:
        assert(skey && input);
        do_dec(*skey, *input);
        break;
    }

    return 0;
}
