#pragma once
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>

struct Vec3 {
    float x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return { x + o.x, y + o.y, z + o.z }; }
    Vec3 operator-(const Vec3& o) const { return { x - o.x, y - o.y, z - o.z }; }
    Vec3 operator*(float s)       const { return { x * s,   y * s,   z * s }; }
    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3  cross(const Vec3& o) const {
        return { y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x };
    }
    float lengthSq() const { return x * x + y * y + z * z; }
    float length()   const { return std::sqrt(lengthSq()); }
    Vec3  normalized() const {
        float l = length();
        return l > 1e-12f ? Vec3{ x / l,y / l,z / l } : Vec3{ 0,0,0 };
    }
    float& operator[](int i) { return (&x)[i]; }
    float  operator[](int i) const { return (&x)[i]; }
};

struct Triangle {
    Vec3 v0, v1, v2;
    int  i0, i1, i2;   // 원본 vertex 인덱스 (pseudonormal 계산용)
};

struct Mesh {
    std::vector<Triangle> triangles;
    std::vector<Vec3>     vertices;  // 원본 정점 목록
    Vec3 bmin{ 1e30f, 1e30f, 1e30f };
    Vec3 bmax{ -1e30f,-1e30f,-1e30f };
};

inline Mesh loadOBJ(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Cannot open OBJ file: " + path);

    std::vector<Vec3> verts;
    std::vector<Triangle> tris;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string tok; ss >> tok;

        if (tok == "v") {
            Vec3 v; ss >> v.x >> v.y >> v.z;
            verts.push_back(v);
        }
        else if (tok == "f") {
            std::vector<int> idx;
            std::string s;
            while (ss >> s) {
                int vi = std::stoi(s.substr(0, s.find('/')));
                vi = (vi < 0) ? (int)verts.size() + vi : vi - 1;
                idx.push_back(vi);
            }
            for (int i = 1; i + 1 < (int)idx.size(); ++i) {
                Triangle tri;
                tri.v0 = verts[idx[0]]; tri.v1 = verts[idx[i]]; tri.v2 = verts[idx[i + 1]];
                tri.i0 = idx[0];        tri.i1 = idx[i];         tri.i2 = idx[i + 1];
                tris.push_back(tri);
            }
        }
    }
    if (tris.empty()) throw std::runtime_error("OBJ file contains no triangles.");

    Mesh mesh;
    mesh.triangles = std::move(tris);
    mesh.vertices = std::move(verts);
    for (const auto& tri : mesh.triangles)
        for (const auto& v : { tri.v0, tri.v1, tri.v2 })
            for (int i = 0; i < 3; ++i) {
                mesh.bmin[i] = std::min(mesh.bmin[i], v[i]);
                mesh.bmax[i] = std::max(mesh.bmax[i], v[i]);
            }
    return mesh;
}