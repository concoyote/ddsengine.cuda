#pragma once

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <device_atomic_functions.h>
#include <curand_kernel.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/gather.h>
#include <thrust/device_ptr.h>

#include "helper_math.h"
#include <string>
#include <vector>
#include <thrust/execution_policy.h>

#define GLM_FORCE_CUDA
#include "glm/glm.hpp"
#include "glm/common.hpp"
#include "glm/ext.hpp"
#include "glm/vec3.hpp"
#include "glm/geometric.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"
#include "glm/gtx/matrix_decompose.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/compatibility.hpp"

struct AABB 
{
    glm::vec3 min;
    glm::vec3 max;
};

struct Mesh
{
    AABB* bbox;
    glm::vec3* vertices;
    glm::vec3* normals;
    glm::vec4* colors;
    unsigned short* indices;
    int* numvertices;
    int* numindices;
    glm::mat4* matrix;
};

void LoadGLB(const std::string& filename, std::vector<Mesh*>& meshes);
void CopyMeshToGPU(Mesh**& d_meshes, const std::vector<Mesh*>& meshes);

#define CUDA_CHECK(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char* file, int line, bool abort = true)
{
    if (code != cudaSuccess)
    {
        fprintf(stderr, "CUDA Error: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort) exit(code);
    }
}
