#pragma once
#include "ObjLoader.h"
#include <vector>
#include <string>
#include <fstream>
#include <stdexcept>
#include <cstdint>
#include <cstring>
#include <algorithm>

// ─────────────────────────────────────────────────────────────
//  Writes VTK XML ImageData (.vti)
//  Scalar type : Float32 (signed distance field)
//  Encoding    : base64 inline binary  (no compression needed)
//  Compatible with ParaView, ITK-SNAP, VTK 6+
// ─────────────────────────────────────────────────────────────

static std::string base64Encode(const uint8_t* data, size_t len) {
    static const char T[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((len + 2) / 3) * 4);
    for (size_t i = 0; i < len; i += 3) {
        uint32_t b = (uint32_t)data[i] << 16;
        if (i + 1 < len) b |= (uint32_t)data[i + 1] << 8;
        if (i + 2 < len) b |= (uint32_t)data[i + 2];
        out += T[(b >> 18) & 63]; out += T[(b >> 12) & 63];
        out += (i + 1 < len) ? T[(b >> 6) & 63] : '=';
        out += (i + 2 < len) ? T[(b >> 0) & 63] : '=';
    }
    return out;
}

inline void writeVTK(const std::string& path,
    const std::vector<float>& vol,
    int                        res,
    const Mesh& mesh)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot write VTI file: " + path);

    // Spacing preserving aspect ratio
    Vec3  bmin = mesh.bmin, bmax = mesh.bmax;
    float maxExt = 0.f;
    for (int i = 0; i < 3; ++i) maxExt = std::max(maxExt, bmax[i] - bmin[i]);
    float pad = maxExt * 0.02f;
    for (int i = 0; i < 3; ++i) { bmin[i] -= pad; bmax[i] += pad; }
    float sx = (bmax.x - bmin.x) / res;
    float sy = (bmax.y - bmin.y) / res;
    float sz = (bmax.z - bmin.z) / res;

    // Compute range for metadata
    float vmin = vol[0], vmax = vol[0];
    for (float v : vol) { vmin = std::min(vmin, v); vmax = std::max(vmax, v); }

    // Build binary payload: 4-byte length prefix + float data
    uint32_t dataBytes = (uint32_t)(vol.size() * sizeof(float));
    std::vector<uint8_t> payload(4 + dataBytes);
    std::memcpy(payload.data(), &dataBytes, 4);
    std::memcpy(payload.data() + 4, vol.data(), dataBytes);
    std::string encoded = base64Encode(payload.data(), payload.size());

    f << "<?xml version=\"1.0\"?>\n"
        << "<VTKFile type=\"ImageData\" version=\"0.1\" byte_order=\"LittleEndian\">\n"
        << "  <ImageData WholeExtent=\"0 " << res - 1 << " 0 " << res - 1 << " 0 " << res - 1 << "\""
        << " Origin=\"" << bmin.x << " " << bmin.y << " " << bmin.z << "\""
        << " Spacing=\"" << sx << " " << sy << " " << sz << "\">\n"
        << "    <Piece Extent=\"0 " << res - 1 << " 0 " << res - 1 << " 0 " << res - 1 << "\">\n"
        << "      <PointData Scalars=\"ImageScalars\">\n"
        << "        <DataArray type=\"Float32\" Name=\"ImageScalars\""
        << " format=\"binary\" NumberOfComponents=\"1\""
        << " RangeMin=\"" << vmin << "\" RangeMax=\"" << vmax << "\">\n"
        << "          " << encoded << "\n"
        << "        </DataArray>\n"
        << "      </PointData>\n"
        << "    </Piece>\n"
        << "  </ImageData>\n"
        << "</VTKFile>\n";

    f.close();
    if (!f) throw std::runtime_error("Error writing VTI file: " + path);
}