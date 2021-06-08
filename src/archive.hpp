#ifndef HOMFA_ARCHIVE_HPP
#define HOMFA_ARCHIVE_HPP

#include "error.hpp"

#include <fstream>

#include <cereal/archives/portable_binary.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>

template <class T>
void read_from_archive(T &res, std::istream &is)
{
    cereal::PortableBinaryInputArchive ar{is};
    ar(res);
}

template <class T>
void read_from_archive(T &res, const std::string &path)
{
    try {
        std::ifstream ifs{path, std::ios::binary};
        assert(ifs && "Can't open the file to read from; maybe not found?");
        read_from_archive<T>(res, ifs);
    }
    catch (std::exception &ex) {
        error::die("Invalid archive: {}", path);
    }
}

template <class T>
T read_from_archive(std::istream &is)
{
    T ret;
    read_from_archive(ret, is);
    return ret;
}

template <class T>
T read_from_archive(const std::string &path)
{
    T ret;
    read_from_archive(ret, path);
    return ret;
}

template <class T>
void write_to_archive(std::ostream &os, const T &src)
{
    cereal::PortableBinaryOutputArchive ar{os};
    ar(src);
}

template <class T>
void write_to_archive(const std::string &path, const T &src)
{
    try {
        std::ofstream ofs{path, std::ios::binary};
        assert(ofs && "Can't open the file to write in; maybe not allowed?");
        return write_to_archive(ofs, src);
    }
    catch (std::exception &ex) {
        spdlog::error(ex.what());
        error::die("Unable to write into archive: {}", path);
    }
}

#endif
