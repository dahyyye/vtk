#pragma once
#include "Bvh.h"
#include <vector>
#include <algorithm>
#include <functional>
#include <cmath>

// ─────────────────────────────────────────────────────────────
//  SDF Voxelizer
//
//  부호 결정:
//   1차) Angle-weighted pseudonormal (BVH.signAt)
//        face / edge / vertex 케이스 모두 처리
//   2차) dot ≈ 0인 퇴화 케이스 → 6방향 다수결 패리티
//
//  사후 처리:
//   - 부호 일관성 스무딩 (3×3×3 이웃 다수결)
//     → 고립된 sign-flip 아티팩트 제거
// ─────────────────────────────────────────────────────────────

// 패리티 테스트 (방향 하나)
static bool parityTest(const Vec3& p, const Vec3& dir, const BVH& bvh) {
    std::vector<float> hits;
    bvh.collectHits(p, dir, 1e-5f, hits);
    std::sort(hits.begin(), hits.end());
    std::vector<float> deduped;
    for (float h : hits)
        if (deduped.empty() || h - deduped.back() > 1e-4f)
            deduped.push_back(h);
    return (deduped.size() & 1) != 0;
}

// 6방향 다수결 패리티 (±X, ±Y, ±Z)
static bool parityVote(const Vec3& p, const BVH& bvh) {
    int cnt = 0;
    if (parityTest(p, { 1,0,0 }, bvh)) ++cnt;
    if (parityTest(p, { -1,0,0 }, bvh)) ++cnt;
    if (parityTest(p, { 0, 1,0 }, bvh)) ++cnt;
    if (parityTest(p, { 0,-1,0 }, bvh)) ++cnt;
    if (parityTest(p, { 0,0, 1 }, bvh)) ++cnt;
    if (parityTest(p, { 0,0,-1 }, bvh)) ++cnt;
    return cnt >= 4;
}

// 부호 일관성 스무딩: 3×3×3 이웃의 부호 다수결로 고립된 오류 제거
static void smoothSigns(std::vector<float>& vol, int res) {
    std::vector<float> out = vol;
    for (int iz = 1; iz < res - 1; ++iz)
        for (int iy = 1; iy < res - 1; ++iy)
            for (int ix = 1; ix < res - 1; ++ix) {
                size_t c = (size_t)ix + (size_t)iy * res + (size_t)iz * res * res;
                int neg = 0, pos = 0;
                for (int dz = -1; dz <= 1; ++dz)
                    for (int dy = -1; dy <= 1; ++dy)
                        for (int dx = -1; dx <= 1; ++dx) {
                            size_t nb = (size_t)(ix + dx) + (size_t)(iy + dy) * res + (size_t)(iz + dz) * res * res;
                            if (vol[nb] < 0) ++neg; else ++pos;
                        }
                // 자신과 다른 방향이 압도적이면 플립
                float absDist = std::abs(vol[c]);
                if (vol[c] < 0 && pos>23) out[c] = absDist; // 27개 이웃 중 24개 이상이 양수
                if (vol[c] > 0 && neg > 23) out[c] = -absDist;
            }
    vol = std::move(out);
}

inline std::vector<float> voxelize(
    const Mesh& mesh,
    const BVH& bvh,
    int         res,
    const std::function<void(int, int)>& progress = nullptr)
{
    std::vector<float> vol(static_cast<size_t>(res) * res * res, 0.f);

    Vec3  bmin = mesh.bmin, bmax = mesh.bmax;
    float maxExt = 0.f;
    for (int i = 0; i < 3; ++i) maxExt = std::max(maxExt, bmax[i] - bmin[i]);
    float pad = maxExt * 0.02f;
    for (int i = 0; i < 3; ++i) { bmin[i] -= pad; bmax[i] += pad; }

    float dx = (bmax.x - bmin.x) / res;
    float dy = (bmax.y - bmin.y) / res;
    float dz = (bmax.z - bmin.z) / res;

    for (int iz = 0; iz < res; ++iz) {
        if (progress) progress(iz, res);
        float wz = bmin.z + (iz + 0.5f) * dz;

        for (int iy = 0; iy < res; ++iy) {
            float wy = bmin.y + (iy + 0.5f) * dy;

            for (int ix = 0; ix < res; ++ix) {
                float wx = bmin.x + (ix + 0.5f) * dx;
                Vec3 p = { wx,wy,wz };

                int   closestTri = -1;
                float distSq = bvh.nearestDistSq(p, closestTri);
                float dist = std::sqrt(distSq);

                float s = 1.f;
                if (closestTri >= 0) {
                    float dot = bvh.signAt(p, closestTri);
                    if (dot < -1e-6f) s = -1.f;  // 안쪽
                    else if (dot > 1e-6f) s = 1.f;  // 바깥
                    else s = parityVote(p, bvh) ? -1.f : 1.f; // 퇴화 케이스
                }

                size_t idx = (size_t)ix + (size_t)iy * res + (size_t)iz * res * res;
                vol[idx] = s * dist;
            }
        }
    }

    // 고립된 sign-flip 아티팩트 제거
    smoothSigns(vol, res);

    return vol;
}