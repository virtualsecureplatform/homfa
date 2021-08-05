#ifndef HOMFA_TFHEPP_UTIL_HPP
#define HOMFA_TFHEPP_UTIL_HPP

#include <fstream>

#include <ThreadPool.h>
#include <tfhe++.hpp>

using Lvl0 = TFHEpp::lvl0param;
using TLWELvl0 = TFHEpp::TLWE<Lvl0>;
using Lvl1 = TFHEpp::lvl1param;
using TLWELvl1 = TFHEpp::TLWE<Lvl1>;
using TRGSWLvl1FFT = TFHEpp::TRGSWFFT<Lvl1>;
using TRLWELvl1 = TFHEpp::TRLWE<Lvl1>;
using PolyLvl1 = TFHEpp::Polynomial<Lvl1>;
using SecretKey = TFHEpp::SecretKey;
using GateKey = TFHEpp::GateKey;
using CircuitKey = TFHEpp::CircuitKey<TFHEpp::lvl02param, TFHEpp::lvl21param>;

class TRGSWLvl1FFTSerializer {
    static_assert(TRGSWLvl1FFT{}.size() == 2 * Lvl1::l);
    static_assert(TRGSWLvl1FFT{}[0].size() == 2);
    static_assert(TRGSWLvl1FFT{}[0][0].size() == Lvl1::n);

private:
    std::ostream &os_;

private:
    void save_binary(const void *data, size_t size);
    void save_double(double src);

public:
    TRGSWLvl1FFTSerializer(std::ostream &os);

    void save(const TRGSWLvl1FFT &src);
};

class TRGSWLvl1FFTDeserializer {
    static_assert(TRGSWLvl1FFT{}.size() == 2 * Lvl1::l);
    static_assert(TRGSWLvl1FFT{}[0].size() == 2);
    static_assert(TRGSWLvl1FFT{}[0][0].size() == Lvl1::n);

public:
    const static size_t BLOCK_SIZE =
        (2 * Lvl1::l) * 2 * Lvl1::n * sizeof(double);

private:
    std::istream &is_;

private:
    void load_binary(void *const data, size_t size);
    void load_double(double &src);

public:
    TRGSWLvl1FFTDeserializer(std::istream &is);

    size_t tell() const;
    bool is_beg() const;
    bool is_end() const;
    void seek(int off, std::ios_base::seekdir dir);
    void load(TRGSWLvl1FFT &out);
};

template <class T>
class InputStream {
public:
    InputStream()
    {
    }
    virtual ~InputStream()
    {
    }

    virtual size_t size() const = 0;
    virtual T next() = 0;
};

class TRGSWLvl1InputStreamFromCtxtFile : public InputStream<TRGSWLvl1FFT> {
private:
    std::ifstream ifs_;
    TRGSWLvl1FFTDeserializer deser_;
    ThreadPool pool_;
    std::future<bool> running_;
    TRGSWLvl1FFT loaded_;
    size_t current_size_;

private:
    void start_loading_next();

public:
    TRGSWLvl1InputStreamFromCtxtFile(const std::string &filename);

    size_t size() const override;
    TRGSWLvl1FFT next() override;
};

class ReversedTRGSWLvl1InputStreamFromCtxtFile
    : public InputStream<TRGSWLvl1FFT> {
private:
    std::ifstream ifs_;
    TRGSWLvl1FFTDeserializer deser_;
    ThreadPool pool_;
    std::future<bool> running_;
    TRGSWLvl1FFT loaded_;
    size_t current_size_;

private:
    void start_loading_next();

public:
    ReversedTRGSWLvl1InputStreamFromCtxtFile(const std::string &filename);

    size_t size() const override;
    TRGSWLvl1FFT next() override;
};

struct RedundantTRLWELvl1 {
    enum {
        TRIVIAL_0,
        TRIVIAL_1,
        NON_TRIVIAL,
    } s;
    TRLWELvl1 c;
};

// Bootstrapping key in the broad sense
struct BKey {
    std::shared_ptr<GateKey> gkey;
    std::shared_ptr<TFHEpp::TLWE2TRLWEIKSKey<TFHEpp::lvl11param>>
        tlwel1_trlwel1_ikskey;
    std::shared_ptr<CircuitKey> circuit_key;

    BKey()
    {
    }

    BKey(const SecretKey &skey)
        : gkey(std::make_shared<GateKey>(skey)),
          tlwel1_trlwel1_ikskey(
              std::make_shared<TFHEpp::TLWE2TRLWEIKSKey<TFHEpp::lvl11param>>()),
          circuit_key(std::make_shared<CircuitKey>(skey))
    {
        TFHEpp::tlwe2trlweikskkgen<TFHEpp::lvl11param>(*tlwel1_trlwel1_ikskey,
                                                       skey);
    }

    template <class Archive>
    void serialize(Archive &ar)
    {
        ar(gkey, tlwel1_trlwel1_ikskey, circuit_key);
    }
};

TLWELvl0 trivial_TLWELvl0_minus_1over8();
TLWELvl0 trivial_TLWELvl0_1over8();
TLWELvl1 trivial_TLWELvl1_minus_1over8();
TLWELvl1 trivial_TLWELvl1_1over8();
TRLWELvl1 trivial_TRLWELvl1(const PolyLvl1 &src);
TRLWELvl1 trivial_TRLWELvl1_zero();
TRLWELvl1 trivial_TRLWELvl1_minus_1over8();
TRLWELvl1 trivial_TRLWELvl1_1over8();
TRLWELvl1 trivial_TRLWELvl1_1over2();
void TLWELvl0_add(TLWELvl0 &lhs, const TLWELvl0 &rhs);
void TLWELvl1_add(TLWELvl1 &lhs, const TLWELvl1 &rhs);
void TRLWELvl1_add(TRLWELvl1 &out, const TRLWELvl1 &src);
void TRLWELvl1_mult_X_k(TRLWELvl1 &out, const TRLWELvl1 &src, size_t k);
uint32_t phase_of_TLWELvl1(const TLWELvl1 &src, const SecretKey &skey);
PolyLvl1 phase_of_TRLWELvl1(const TRLWELvl1 &src, const SecretKey &skey);
void do_SEI_IKS_GBTLWE2TRLWE(TRLWELvl1 &w, const GateKey &gk);
void do_SEI_IKS_GBTLWE2TRLWE_2(TRLWELvl1 &w, const GateKey &gk);
void do_SEI_IKS_GBTLWE2TRLWE_3(TRLWELvl1 &w, const GateKey &gk);
void BS_TLWE_0_1o2_to_TRLWE_0_1o2(TRLWELvl1 &out, TLWELvl0 &src,
                                  const GateKey &gk);
void BS_TLWE_0_1o2_to_TRLWE_m1o8_1o8(TRLWELvl1 &out, TLWELvl0 &src,
                                     const GateKey &gk);
TRGSWLvl1FFT encrypt_bit_to_TRGSWLvl1FFT(bool b, const SecretKey &skey);
bool decrypt_TLWELvl1_to_bit(const TLWELvl1 &c, const SecretKey &skey);
PolyLvl1 uint2weight(uint64_t n);
bool between_25_75(uint32_t n);
void dump_weight(std::ostream &os, const PolyLvl1 &w);
std::string weight2bitstring(const PolyLvl1 &w);
void CircuitBootstrappingFFTLvl01(TRGSWLvl1FFT &out, const TLWELvl0 &src,
                                  const CircuitKey &circuit_key);

#endif
