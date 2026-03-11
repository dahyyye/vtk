#include "ObjLoader.h"
#include "Bvh.h"
#include "Voxelizer.h"
#include "VTKWriter.h"

#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <limits>

static void showProgress(int done, int total) {
    constexpr int BAR = 40;
    float pct = static_cast<float>(done) / total;
    int   filled = static_cast<int>(pct * BAR);
    std::cout << "\r  [";
    for (int i = 0; i < BAR; ++i)
        std::cout << (i < filled ? '#' : '.');
    std::cout << "] " << std::setw(3) << static_cast<int>(pct * 100) << "%" << std::flush;
}

static std::string fileStem(const std::string& path) {
    size_t sepPos = path.find_last_of("/\\");
    std::string name = (sepPos == std::string::npos) ? path : path.substr(sepPos + 1);
    size_t dotPos = name.rfind('.');
    return (dotPos == std::string::npos) ? name : name.substr(0, dotPos);
}

static long long getFileSize(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    return f ? (long long)f.tellg() : -1LL;
}

static int pickResolution() {
    const int choices[] = { 32, 64, 128, 256, 512 };
    constexpr int N = 5;
    std::cout << "\n+-----------------------------------+\n"
        << "|   Select voxel grid resolution    |\n"
        << "+-----------------------------------+\n";
    for (int i = 0; i < N; ++i)
        std::cout << "|  [" << (i + 1) << "]  "
        << std::setw(3) << choices[i]
        << " x " << choices[i] << " x " << choices[i]
        << "               |\n";
    std::cout << "+-----------------------------------+\n"
        << "  Choice (1-" << N << "): ";
    int sel = 0;
    while (true) {
        std::cin >> sel;
        if (sel >= 1 && sel <= N) break;
        std::cout << "  Invalid. Enter 1-" << N << ": ";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return choices[sel - 1];
}

int main(int argc, char* argv[]) {
    std::cout << "====================================\n"
        << "  OBJ  =>  BVH  =>  VTK Volume Tool\n"
        << "====================================\n";

    std::string objPath;
    if (argc >= 2) {
        objPath = argv[1];
    }
    else {
        std::cout << "\nEnter OBJ file path:\n  > ";
        std::getline(std::cin >> std::ws, objPath);
        if (!objPath.empty() && objPath.front() == '"')
            objPath = objPath.substr(1, objPath.size() - 2);
    }

    int res = 0;
    if (argc >= 3) {
        try { res = std::stoi(argv[2]); }
        catch (...) { res = 0; }
    }
    if (res <= 0) res = pickResolution();

    // [1/4] Load OBJ
    Mesh mesh;
    try {
        std::cout << "\n[1/4] Loading OBJ ...  ";
        auto t0 = std::chrono::steady_clock::now();
        mesh = loadOBJ(objPath);
        double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        std::cout << "done  (" << mesh.triangles.size() << " triangles,  "
            << std::fixed << std::setprecision(1) << ms << " ms)\n";
        std::cout << "       Bounds  X [" << mesh.bmin.x << ", " << mesh.bmax.x << "]\n"
            << "               Y [" << mesh.bmin.y << ", " << mesh.bmax.y << "]\n"
            << "               Z [" << mesh.bmin.z << ", " << mesh.bmax.z << "]\n";
    }
    catch (const std::exception& e) {
        std::cerr << "\nERROR: " << e.what() << '\n';
        return 1;
    }

    // [2/4] Build BVH
    BVH bvh;
    try {
        std::cout << "\n[2/4] Building BVH ... ";
        auto t0 = std::chrono::steady_clock::now();
        bvh.build(mesh.triangles);
        double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        std::cout << "done  (" << bvh.nodes.size() << " nodes,  "
            << std::fixed << std::setprecision(1) << ms << " ms)\n";
    }
    catch (const std::exception& e) {
        std::cerr << "\nERROR: " << e.what() << '\n';
        return 1;
    }

    // [3/4] Voxelize
    std::vector<float> vol;
    try {
        std::cout << "\n[3/4] Voxelizing  " << res << "^3 ...\n";
        auto t0 = std::chrono::steady_clock::now();
        vol = voxelize(mesh, bvh, res,
            [](int z, int total) { showProgress(z, total); });
        showProgress(res, res);
        double sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        long long total_vox = (long long)res * res * res;
        long long filled = 0;
        for (float v : vol) if (v < 0.f) ++filled;
        std::cout << "\n       done  (" << std::fixed << std::setprecision(2) << sec << " s)\n"
            << "       Inside ratio: " << std::setprecision(1)
            << 100.0 * filled / total_vox << " %\n";
    }
    catch (const std::exception& e) {
        std::cerr << "\nERROR: " << e.what() << '\n';
        return 1;
    }

    // [4/4] Write VTK
    std::string outPath = fileStem(objPath) + "_" + std::to_string(res) + ".vti";
    try {
        std::cout << "\n[4/4] Writing VTK ... ";
        auto t0 = std::chrono::steady_clock::now();
        writeVTK(outPath, vol, res, mesh);
        double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        long long bytes = getFileSize(outPath);
        std::cout << "done  (" << std::fixed << std::setprecision(1) << ms << " ms,  "
            << bytes / (1024 * 1024) << " MB)\n";
    }
    catch (const std::exception& e) {
        std::cerr << "\nERROR: " << e.what() << '\n';
        return 1;
    }

    std::cout << "\n  Output: " << outPath << "\n"
        << "====================================\n"
        << "  Done! Open with ParaView or ITK-SNAP.\n"
        << "====================================\n\n";

    if (argc < 2) {
        std::cout << "Press ENTER to exit...";
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cin.get();
    }
    return 0;
}