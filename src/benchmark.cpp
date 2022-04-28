#include "archive.hpp"
#include "error.hpp"
#include "offline_dfa.hpp"
#include "online_dfa.hpp"
#include "timeit.hpp"
#include "utility.hpp"

#include <chrono>
#include <iostream>
#include <optional>

#include <CLI/CLI.hpp>

std::string current_time_str()
{
    auto now = std::chrono::system_clock::now();
    std::time_t t1 = std::chrono::system_clock::to_time_t(now);
    std::tm* t = std::localtime(&t1);
    std::stringstream ss;
    ss << t->tm_year + 1900 << "-" << t->tm_mon << "-" << t->tm_mday << "_"
       << t->tm_hour << "-" << t->tm_min << "-" << t->tm_sec;
    return ss.str();
}

template <class T1, class T2>
void print(T1 key, T2 value)
{
    std::cout << key << "," << value << std::endl;
}

template <class Key, class Func>
void print_elapsed(Key key, Func func)
{
    auto elapsed = timeit(func);
    print(key, elapsed.count());
}

template <class Runner>
void enc_run_dec_loop(const SecretKey& skey, const std::string& input_filename,
                      size_t num_ap, Runner&& runner)
{
    each_input_bit(input_filename, num_ap, [&](bool input) {
        // Encrypt
        TRGSWLvl1FFT enc_input;
        print_elapsed("enc", [&] {
            enc_input = encrypt_bit_to_TRGSWLvl1FFT(input, skey);
        });

        // Run
        bool output_exists = false;
        print_elapsed("run", [&] { output_exists = runner.run(enc_input); });

        // Decrypt if output exists
        if (output_exists) {
            bool result = false;
            print_elapsed("dec", [&] {
                result = decrypt_TLWELvl1_to_bit(runner.result(), skey);
            });
            print("result", result);
        }
    });
}

class OfflineBenchRunner {
private:
    OfflineDFARunner runner_;
    TLWELvl1 result_;
    size_t remaining_input_size_;

public:
    OfflineBenchRunner(const std::string& spec_filename, size_t input_size,
                       size_t boot_interval, const BKey& bkey,
                       bool sanitize_result)
        : runner_(Graph::from_file(spec_filename), input_size, boot_interval,
                  bkey.gkey, sanitize_result),
          result_(),
          remaining_input_size_(input_size)
    {
    }

    size_t num_states() const
    {
        return runner_.graph().size();
    }

    bool run(const TRGSWLvl1FFT& input)
    {
        runner_.eval_one(input);
        --remaining_input_size_;
        if (remaining_input_size_ == 0)
            result_ = runner_.result();
        return false;
    }

    TLWELvl1 result() const
    {
        return result_;
    }
};

class OnlineDFA2BenchRunner {
private:
    OnlineDFARunner2 runner_;
    size_t output_freq_, num_processed_;
    TLWELvl1 result_;

public:
    OnlineDFA2BenchRunner(const std::string& spec_filename, size_t output_freq,
                          size_t bootstrapping_freq, bool spec_reversed,
                          const BKey& bkey, bool sanitize_result)
        : runner_(Graph::from_file(spec_filename), bootstrapping_freq,
                  spec_reversed, bkey.gkey, sanitize_result),
          output_freq_(output_freq),
          num_processed_(0)
    {
    }

    const OnlineDFARunner2& runner() const
    {
        return runner_;
    }

    size_t num_states() const
    {
        return runner_.graph().size();
    }

    bool run(const TRGSWLvl1FFT& input)
    {
        runner_.eval_one(input);
        num_processed_++;
        if (num_processed_ % output_freq_ != 0)
            return false;
        result_ = runner_.result();
        return true;
    }

    TLWELvl1 result() const
    {
        return result_;
    }
};

class OnlineDFA3BenchRunner {
private:
    OnlineDFARunner3 runner_;
    size_t output_freq_, queue_size_, bootstrapping_freq_, num_processed_;
    TLWELvl1 result_;

public:
    OnlineDFA3BenchRunner(const std::string& spec_filename, size_t output_freq,
                          size_t max_second_lut_depth, size_t queue_size,
                          size_t bootstrapping_freq, const BKey& bkey,
                          bool sanitize_result)
        : runner_(Graph::from_file(spec_filename), max_second_lut_depth,
                  queue_size, bootstrapping_freq, *bkey.gkey,
                  *bkey.tlwel1_trlwel1_ikskey, std::nullopt, sanitize_result),
          output_freq_(output_freq),
          queue_size_(queue_size),
          bootstrapping_freq_(bootstrapping_freq),
          num_processed_(0)
    {
    }

    size_t num_states() const
    {
        return runner_.graph().size();
    }

    bool run(const TRGSWLvl1FFT& input)
    {
        print("num_live_states", runner_.num_live_states());
        print("first_lut_depth", runner_.first_lut_depth());
        print("second_lut_depth", runner_.second_lut_depth());
        runner_.eval_one(input);
        num_processed_++;
        if (num_processed_ % output_freq_ != 0)
            return false;
        result_ = runner_.result();
        return true;
    }

    TLWELvl1 result() const
    {
        return result_;
    }
};

class OnlineDFA4BenchRunner {
private:
    OnlineDFARunner4 runner_;
    size_t output_freq_, queue_size_, num_processed_;
    TLWELvl1 result_;

public:
    OnlineDFA4BenchRunner(const std::string& spec_filename, size_t output_freq,
                          size_t queue_size, const BKey& bkey,
                          bool sanitize_result)
        : runner_(Graph::from_file(spec_filename), queue_size, *bkey.gkey,
                  *bkey.circuit_key, sanitize_result),
          output_freq_(output_freq),
          queue_size_(queue_size),
          num_processed_(0)
    {
    }

    const OnlineDFARunner4& runner() const
    {
        return runner_;
    }

    size_t num_states() const
    {
        return runner_.graph().size();
    }

    bool run(const TRGSWLvl1FFT& input)
    {
        print("num_live_states", runner_.num_live_states());
        runner_.eval_one(input);
        num_processed_++;
        if (num_processed_ % output_freq_ != 0)
            return false;
        result_ = runner_.result();
        return true;
    }

    TLWELvl1 result() const
    {
        return result_;
    }
};

void do_plain(const std::string& spec_filename,
              const std::string& input_filename, size_t output_freq,
              size_t num_ap)
{
    print("config-method", "plain");
    print("config-spec", spec_filename);
    print("config-input", input_filename);
    print("config-num_ap", num_ap);

    Graph gr = Graph::from_file(spec_filename);
    print("config-spec_num_states", gr.size());

    size_t input_size = 0;
    each_input_bit(input_filename, num_ap, [&](bool) { input_size++; });
    print("config-input_size", input_size);

    Graph::State cur = gr.initial_state();
    size_t i = 0;
    each_input_bit(input_filename, num_ap, [&](bool b) {
        cur = gr.next_state(cur, b);
        print("state", cur);
        i++;
        if (i % output_freq == 0) {
            if (gr.is_final_state(cur))
                print("result", 1);
            else
                print("result", 0);
        }
    });
}

void do_offline(const std::string& spec_filename,
                const std::string& input_filename, size_t bootstrapping_freq,
                size_t num_ap, bool sanitize_result)
{
    print("config-method", "offline");
    print("config-spec", spec_filename);
    print("config-input", input_filename);
    print("config-bootstrapping_freq", bootstrapping_freq);
    print("config-num_ap", num_ap);
    print("config-sanitize_result", sanitize_result);

    std::optional<SecretKey> skey_opt;
    std::optional<BKey> bkey_opt;

    auto skey_elapsed = timeit([&] { skey_opt.emplace(); });
    const SecretKey& skey = skey_opt.value();
    auto bkey_elapsed = timeit([&] { bkey_opt.emplace(skey, false); });
    const BKey& bkey = bkey_opt.value();

    print("skey", skey_elapsed.count());
    print("bkey", bkey_elapsed.count());

    std::vector<bool> input_bits;
    each_input_bit(input_filename, num_ap,
                   [&](bool b) { input_bits.push_back(b); });

    std::optional<OfflineBenchRunner> runner_opt;
    print_elapsed("init", [&] {
        runner_opt.emplace(spec_filename, input_bits.size(), bootstrapping_freq,
                           bkey, sanitize_result);
    });
    OfflineBenchRunner& runner = runner_opt.value();

    print("config-spec_num_states", runner.num_states());
    print("config-input_size", input_bits.size());

    for (auto it = input_bits.rbegin(); it != input_bits.rend(); it++) {
        bool input = *it;

        // Encrypt
        TRGSWLvl1FFT enc_input;
        print_elapsed("enc", [&] {
            enc_input = encrypt_bit_to_TRGSWLvl1FFT(input, skey);
        });

        // Run
        print_elapsed("run", [&] { runner.run(enc_input); });
    }
    // Decrypt
    bool result = false;
    print_elapsed("dec", [&] {
        result = decrypt_TLWELvl1_to_bit(runner.result(), skey);
    });
    print("result", result);
}

void do_reversed(const std::string& spec_filename,
                 const std::string& input_filename, size_t output_freq,
                 size_t bootstrapping_freq, size_t num_ap, bool spec_reversed,
                 bool sanitize_result)
{
    print("config-method", "reversed");
    print("config-spec", spec_filename);
    print("config-input", input_filename);
    print("config-output_freq", output_freq);
    print("config-bootstrapping_freq", bootstrapping_freq);
    print("config-num_ap", num_ap);
    print("config-sanitize_result", sanitize_result);

    std::optional<SecretKey> skey_opt;
    std::optional<BKey> bkey_opt;

    auto skey_elapsed = timeit([&] { skey_opt.emplace(); });
    const SecretKey& skey = skey_opt.value();
    auto bkey_elapsed = timeit([&] { bkey_opt.emplace(skey, false); });
    const BKey& bkey = bkey_opt.value();

    print("skey", skey_elapsed.count());
    print("bkey", bkey_elapsed.count());

    std::optional<OnlineDFA2BenchRunner> runner_opt;
    print_elapsed("init", [&] {
        runner_opt.emplace(spec_filename, output_freq, bootstrapping_freq,
                           spec_reversed, bkey, sanitize_result);
    });
    OnlineDFA2BenchRunner& runner = runner_opt.value();

    print("config-spec_num_states", runner.num_states());

    size_t input_size = 0;
    each_input_bit(input_filename, num_ap, [&](bool) { input_size++; });
    print("config-input_size", input_size);

    enc_run_dec_loop(skey, input_filename, num_ap, runner);
    runner.runner().timer().dumpCSV(std::cout);
}

void do_qtrlwe2(const std::string& spec_filename,
                const std::string& input_filename, size_t output_freq,
                size_t max_second_lut_depth, size_t queue_size,
                size_t bootstrapping_freq, size_t num_ap, bool sanitize_result)
{
    print("config-method", "qtrlwe2");
    print("config-spec", spec_filename);
    print("config-input", input_filename);
    print("config-output_freq", output_freq);
    print("config-max_second_lut_depth", max_second_lut_depth);
    print("config-queue_size", queue_size);
    print("config-bootstrapping_freq", bootstrapping_freq);
    print("config-num_ap", num_ap);
    print("config-sanitize_result", sanitize_result);

    std::optional<SecretKey> skey_opt;
    std::optional<BKey> bkey_opt;

    auto skey_elapsed = timeit([&] { skey_opt.emplace(); });
    const SecretKey& skey = skey_opt.value();
    auto bkey_elapsed = timeit([&] { bkey_opt.emplace(skey, false); });
    const BKey& bkey = bkey_opt.value();

    print("skey", skey_elapsed.count());
    print("bkey", bkey_elapsed.count());

    std::optional<OnlineDFA3BenchRunner> runner_opt;
    print_elapsed("init", [&] {
        runner_opt.emplace(spec_filename, output_freq, max_second_lut_depth,
                           queue_size, bootstrapping_freq, bkey,
                           sanitize_result);
    });
    OnlineDFA3BenchRunner& runner = runner_opt.value();

    print("config-spec_num_states", runner.num_states());

    size_t input_size = 0;
    each_input_bit(input_filename, num_ap, [&](bool) { input_size++; });
    print("config-input_size", input_size);

    enc_run_dec_loop(skey, input_filename, num_ap, runner);
}

void do_bbs(const std::string& spec_filename, const std::string& input_filename,
            size_t output_freq, size_t queue_size, size_t num_ap,
            bool sanitize_result)
{
    print("config-method", "bbs");
    print("config-spec", spec_filename);
    print("config-input", input_filename);
    print("config-output_freq", output_freq);
    print("config-queue_size", queue_size);
    print("config-num_ap", num_ap);
    print("config-sanitize_reuslt", sanitize_result);

    std::optional<SecretKey> skey_opt;
    std::optional<BKey> bkey_opt;

    auto skey_elapsed = timeit([&] { skey_opt.emplace(); });
    const SecretKey& skey = skey_opt.value();
    auto bkey_elapsed = timeit([&] { bkey_opt.emplace(skey, true); });
    const BKey& bkey = bkey_opt.value();

    print("skey", skey_elapsed.count());
    print("bkey", bkey_elapsed.count());

    std::optional<OnlineDFA4BenchRunner> runner_opt;
    print_elapsed("init", [&] {
        runner_opt.emplace(spec_filename, output_freq, queue_size, bkey,
                           sanitize_result);
    });
    OnlineDFA4BenchRunner& runner = runner_opt.value();

    print("config-spec_num_states", runner.num_states());

    size_t input_size = 0;
    each_input_bit(input_filename, num_ap, [&](bool) { input_size++; });
    print("config-input_size", input_size);

    enc_run_dec_loop(skey, input_filename, num_ap, runner);
    runner.runner().timer().dumpCSV(std::cout);
}

int main(int argc, char** argv)
{
    CLI::App app{"Benchmark runner"};
    app.require_subcommand();

    enum class TYPE {
        PLAIN,
        OFFLINE,
        REVERSED,
        QTRLWE2,
        BBS,
    } type;
    std::string spec_filename, input_filename;
    size_t output_freq, num_ap, max_second_lut_depth, queue_size,
        bootstrapping_freq;
    bool verbose = false, spec_reversed = false, sanitize_result = false;

    app.add_flag("--verbose", verbose, "");
    {
        CLI::App* plain = app.add_subcommand("plain", "Run in plaintext");
        plain->parse_complete_callback([&] { type = TYPE::PLAIN; });
        plain->add_option("--ap", num_ap)
            ->required()
            ->check(CLI::PositiveNumber);
        plain->add_option("--spec", spec_filename)
            ->required()
            ->check(CLI::ExistingFile);
        plain->add_option("--in", input_filename)
            ->required()
            ->check(CLI::ExistingFile);
        plain->add_option("--out-freq", output_freq)
            ->required()
            ->check(CLI::PositiveNumber);
    }
    {
        CLI::App* offline = app.add_subcommand("offline", "Run offline");
        offline->parse_complete_callback([&] { type = TYPE::OFFLINE; });
        offline->add_option("--ap", num_ap)
            ->required()
            ->check(CLI::PositiveNumber);
        offline->add_option("--bootstrapping-freq", bootstrapping_freq)
            ->required()
            ->check(CLI::PositiveNumber);
        offline->add_option("--spec", spec_filename)
            ->required()
            ->check(CLI::ExistingFile);
        offline->add_option("--in", input_filename)
            ->required()
            ->check(CLI::ExistingFile);
        offline->add_flag("--sanitize-result", sanitize_result);
    }
    {
        CLI::App* rev = app.add_subcommand("reversed", "Run online-reversed");
        rev->parse_complete_callback([&] { type = TYPE::REVERSED; });
        rev->add_option("--ap", num_ap)->required()->check(CLI::PositiveNumber);
        rev->add_option("--out-freq", output_freq)
            ->required()
            ->check(CLI::PositiveNumber);
        rev->add_option("--bootstrapping-freq", bootstrapping_freq)
            ->required()
            ->check(CLI::PositiveNumber);
        rev->add_option("--spec", spec_filename)
            ->required()
            ->check(CLI::ExistingFile);
        rev->add_option("--in", input_filename)
            ->required()
            ->check(CLI::ExistingFile);
        rev->add_flag("--spec-reversed", spec_reversed);
        rev->add_flag("--sanitize-result", sanitize_result);
    }
    {
        CLI::App* qtrlwe2 = app.add_subcommand("qtrlwe2", "Run online-qtrlwe2");
        qtrlwe2->parse_complete_callback([&] { type = TYPE::QTRLWE2; });
        qtrlwe2->add_option("--ap", num_ap)
            ->required()
            ->check(CLI::PositiveNumber);
        qtrlwe2->add_option("--out-freq", output_freq)
            ->required()
            ->check(CLI::PositiveNumber);
        qtrlwe2->add_option("--queue-size", queue_size)
            ->required()
            ->check(CLI::PositiveNumber);
        qtrlwe2->add_option("--bootstrapping-freq", bootstrapping_freq)
            ->required()
            ->check(CLI::PositiveNumber);
        qtrlwe2->add_option("--spec", spec_filename)
            ->required()
            ->check(CLI::ExistingFile);
        qtrlwe2->add_option("--in", input_filename)
            ->required()
            ->check(CLI::ExistingFile);
        qtrlwe2->add_option("--max-second-lut-depth", max_second_lut_depth)
            ->required()
            ->check(CLI::PositiveNumber);
        qtrlwe2->add_flag("--sanitize-result", sanitize_result);
    }
    {
        CLI::App* bbs =
            app.add_subcommand("bbs", "Run online-block-backstream");
        bbs->parse_complete_callback([&] { type = TYPE::BBS; });
        bbs->add_option("--ap", num_ap)->required()->check(CLI::PositiveNumber);
        bbs->add_option("--out-freq", output_freq)
            ->required()
            ->check(CLI::PositiveNumber);
        bbs->add_option("--queue-size", queue_size)
            ->required()
            ->check(CLI::PositiveNumber);
        bbs->add_option("--spec", spec_filename)
            ->required()
            ->check(CLI::ExistingFile);
        bbs->add_option("--in", input_filename)
            ->required()
            ->check(CLI::ExistingFile);
        bbs->add_flag("--sanitize-result", sanitize_result);
    }

    CLI11_PARSE(app, argc, argv);

    print("time", current_time_str());

    if (verbose)
        spdlog::set_level(spdlog::level::debug);

    switch (type) {
    case TYPE::PLAIN:
        do_plain(spec_filename, input_filename, output_freq, num_ap);
        break;

    case TYPE::OFFLINE:
        do_offline(spec_filename, input_filename, bootstrapping_freq, num_ap,
                   sanitize_result);
        break;

    case TYPE::REVERSED:
        do_reversed(spec_filename, input_filename, output_freq,
                    bootstrapping_freq, num_ap, spec_reversed, sanitize_result);
        break;

    case TYPE::QTRLWE2:
        do_qtrlwe2(spec_filename, input_filename, output_freq,
                   max_second_lut_depth, queue_size, bootstrapping_freq, num_ap,
                   sanitize_result);
        break;

    case TYPE::BBS:
        do_bbs(spec_filename, input_filename, output_freq, queue_size, num_ap,
               sanitize_result);
        break;
    }

    return 0;
}
