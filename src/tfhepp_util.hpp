#ifndef HOMFA_TFHEPP_UTIL_HPP
#define HOMFA_TFHEPP_UTIL_HPP

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
    std::vector<TRGSWLvl1FFT> data_;
    std::vector<TRGSWLvl1FFT>::iterator head_;

public:
    TRGSWLvl1InputStreamFromCtxtFile(const std::string &filename);

    size_t size() const override;
    TRGSWLvl1FFT next() override;
};

class ReversedTRGSWLvl1InputStreamFromCtxtFile
    : public InputStream<TRGSWLvl1FFT> {
private:
    std::vector<TRGSWLvl1FFT> data_;
    std::vector<TRGSWLvl1FFT>::reverse_iterator head_;

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

TLWELvl0 trivial_TLWELvl0_minus_1over8();
TLWELvl0 trivial_TLWELvl0_1over8();
TLWELvl1 trivial_TLWELvl1_minus_1over8();
TLWELvl1 trivial_TLWELvl1_1over8();
TRLWELvl1 trivial_TRLWELvl1(const PolyLvl1 &src);
TRLWELvl1 trivial_TRLWELvl1_zero();
TRLWELvl1 trivial_TRLWELvl1_minus_1over8();
TRLWELvl1 trivial_TRLWELvl1_1over8();
void TLWELvl0_add(TLWELvl0 &lhs, const TLWELvl0 &rhs);
void TLWELvl1_add(TLWELvl1 &lhs, const TLWELvl1 &rhs);
void TRLWELvl1_add(TRLWELvl1 &out, const TRLWELvl1 &src);
void TRLWELvl1_mult_X_k(TRLWELvl1 &out, const TRLWELvl1 &src, size_t k);
uint32_t phase_of_TLWELvl1(const TLWELvl1 &src, const SecretKey &skey);
PolyLvl1 phase_of_TRLWELvl1(const TRLWELvl1 &src, const SecretKey &skey);
void do_SEI_IKS_GBTLWE2TRLWE(TRLWELvl1 &w, const GateKey &gk);
TRGSWLvl1FFT encrypt_bit_to_TRGSWLvl1FFT(bool b, const SecretKey &skey);
PolyLvl1 uint2weight(uint64_t n);
bool between_25_75(uint32_t n);
void dump_weight(std::ostream &os, const PolyLvl1 &w);
std::string weight2bitstring(const PolyLvl1 &w);

#endif
