#pragma once
#ifndef _RANDOMENGINE_H_
#define _RANDOMENGINE_H_

#include <vector>
#include <memory>

class LdsGenerator
{
public:
    LdsGenerator() = default;
    LdsGenerator(int pixelNum, int dim);
    void Build(int pixelNum, int dim);
    float Get(int dim, int thread = 0);
    void Reset();
    void Reset(int dim, int thread);

private:
    std::vector<std::vector<uint32_t>> mDirNumV_;    // [dim][k]
    std::vector<std::vector<uint32_t>> mIndex;       // [dim][pixelNum]    ָ�ڼ���sobol��
    std::vector<std::vector<uint32_t>> mDigit32;     // [dim][pixelNum]    ��������2^32Ϊsobol��
    std::vector<uint32_t> mBits;
    float mFactor = 1.0f / (1ull << 32);

    std::vector<uint32_t> mASet = {
        0, 0, 1, 1, 2, 1, 4, 2, 4, 7, 11, 13, 14, 1, 13, 16, 19, 22, 25, 1, 4, 7, 8 };
    std::vector<std::vector<uint32_t>> mMSet = {
        { 0 },
        { 1 },
        { 1, 3 },
        { 1, 3, 1 },
        { 1, 1, 1 },
        { 1, 1, 3, 3 },
        { 1, 3, 5, 13 },
        { 1, 1, 5, 5, 17 },
        { 1, 1, 5, 5, 5 },
        { 1, 1, 7, 11, 19 },
        { 1, 1, 5, 1, 1 },
        { 1, 1, 1, 3, 11 },
        { 1, 3, 5, 5, 31 },
        { 1, 3, 3, 9, 7, 49 },
        { 1, 1, 1, 15, 21, 21 },
        { 1, 3, 1, 13, 27, 49 },
        { 1, 1, 1, 15, 7, 5 },
        { 1, 3, 1, 15, 13, 25 },
        { 1, 1, 5, 5, 19, 61 },
        { 1, 3, 7, 11, 23, 15, 103 },
        { 1, 3, 7, 13, 13, 15, 69 },
        { 1, 1, 3, 13, 7, 35, 63 },
        { 1, 5, 9, 1, 25, 53 }
    };
};

template<typename RandomEngine>
class UniformSampler{
    // ���Ȳ�����
public:
    std::shared_ptr<RandomEngine> RandomCreator;
    size_t maxSize = 0, dim = 0;
public:
    UniformSampler(size_t maxSize, size_t dim);
    ~UniformSampler() = default;
};

using SobolSampler = UniformSampler<LdsGenerator>;

#endif	// !_RANDOMENGINE_H_