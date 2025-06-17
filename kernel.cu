#define ASIO_STANDALONE
#define _WEBSOCKETPP_CPP11_TYPE_TRAITS_
#define _WEBSOCKETPP_CPP11_MEMORY_
#define _WEBSOCKETPP_CPP11_FUNCTIONAL_
#define _WEBSOCKETPP_CPP11_SYSTEM_ERROR_
#define _WEBSOCKETPP_CPP11_RANDOM_DEVICE_
#define _WEBSOCKETPP_CPP11_THREAD_
#define _WEBSOCKETPP_CPP11_CHRONO_
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <cub/cub.cuh>

#include "glb.h"
#include <iostream>
#include "tinygltf/stb_image_write.h"
#include "./libjpeg-turbo/include/turbojpeg.h"

#include <fstream>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <cfloat>
#include <string>

struct Ray
{
    glm::vec3 origin;
    glm::vec3 direction;
};

typedef websocketpp::server<websocketpp::config::asio> server;

#define SPP 4
std::vector<Mesh*> gMeshes;
Mesh** d_meshes = nullptr;
int width = 64;
int height = 64;

float* d_depths = nullptr;;
int* d_indices = nullptr;;

unsigned char* d_colors = nullptr;;
unsigned char* h_colors = nullptr;;
float* d_depth = nullptr;

cudaEvent_t start, stop;
float iTime = 0.0f;

tjhandle tjCompressor = nullptr;
unsigned char* jpegBuffer = nullptr;
std::string base64;

bool bEventDown = false;
bool bEventRotation = true;
float eclientX;
float eclientY;

glm::mat4 Projection;
glm::mat4 View;
glm::mat4 World;

glm::mat4* d_Projection;
glm::mat4* d_View;
glm::mat4* d_World;

static const std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
std::string base64_encode(const unsigned char* bytes_to_encode, unsigned int in_len) 
{
    std::string ret;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    while (in_len--) 
    {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3) 
        {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for (i = 0; (i < 4); i++) ret += base64_chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i)
    {
        for (j = i; j < 3; j++) char_array_3[j] = '\0';

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++) ret += base64_chars[char_array_4[j]];

        while ((i++ < 3)) ret += '=';
    }

    return ret;
}

__device__ bool intersectAABB(const Ray& ray, const AABB* box)
{
    float tmin = -FLT_MAX, tmax = FLT_MAX;

    for (int i = 0; i < 3; ++i) 
    {
        float origin = (&ray.origin.x)[i];
        float direction = (&ray.direction.x)[i];
        float bmin = (&box->min.x)[i];
        float bmax = (&box->max.x)[i];

        if (fabs(direction) < 1e-8f) 
        {
            if (origin < bmin || origin > bmax) return false;
        }
        else 
        {
            float t1 = (bmin - origin) / direction;
            float t2 = (bmax - origin) / direction;
            if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
            if (tmin > tmax) return false;
        }
    }

    return true;
}

__device__ bool intersectAABB(const Ray& ray, const AABB& box, float& tmin_out, float& tmax_out)
{
    float tmin = -FLT_MAX, tmax = FLT_MAX;

    for (int i = 0; i < 3; ++i) 
    {
        float origin = (&ray.origin.x)[i];
        float direction = (&ray.direction.x)[i];
        float bmin = (&box.min.x)[i];
        float bmax = (&box.max.x)[i];

        if (fabs(direction) < 1e-8f) 
        {
            if (origin < bmin || origin > bmax) return false;
        }
        else 
        {
            float t1 = (bmin - origin) / direction;
            float t2 = (bmax - origin) / direction;
            if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
            if (t1 > tmin) tmin = t1;
            if (t2 < tmax) tmax = t2;
            if (tmin > tmax) return false;
        }
    }

    if (tmin > tmin_out) return false;

    tmin_out = tmin;
    tmax_out = tmax;
    return true;
}

__device__ Ray glup_primary_ray(const glm::vec2& gl_FragCoord, const glm::vec2& iResolution, const glm::mat4* Projection, const glm::mat4* View, const glm::mat4* World)
{
    const glm::mat4 inv = glm::inverse(*Projection * *View * *World);
    const glm::vec2 pos = (1.0f * (gl_FragCoord / iResolution)) - 0.5f;
    const glm::vec4 near_4 = inv * (glm::vec4(pos, -1.0, 1.0));
    const glm::vec4 far_4 = inv * (glm::vec4(pos, 1.0, 1.0));
    const glm::vec3 rayOrigin = glm::vec3(near_4.x / near_4.w, near_4.y / near_4.w, near_4.z / near_4.w);
    const glm::vec3 far3 = glm::vec3(far_4.x / far_4.w, far_4.y / far_4.w, far_4.z / far_4.w);
    const glm::vec3 rayDir = far3 - rayOrigin;

    Ray ray;
    ray.origin = rayOrigin;
    ray.direction = normalize(rayDir);
    return ray;
}

__device__ bool rayTriangleIntersect(const glm::vec3& ro, const glm::vec3& rd, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, float& t, glm::vec3& bary)
{
    const float EPSILON = 1e-8f;
    glm::vec3 edge1 = v1 - v0;
    glm::vec3 edge2 = v2 - v0;
    glm::vec3 h = glm::cross(rd, edge2);
    float a = glm::dot(edge1, h);
    if (fabs(a) < EPSILON) return false;

    float f = 1.0f / a;
    glm::vec3 s = ro - v0;
    float u = f * glm::dot(s, h);
    if (u < 0.0f || u > 1.0f) return false;

    glm::vec3 q = glm::cross(s, edge1);
    float v = f * glm::dot(rd, q);
    if (v < 0.0f || u + v > 1.0f) return false;

    t = f * glm::dot(edge2, q);
    if (t > EPSILON) 
    {
        bary = glm::vec3(1.0f - u - v, u, v);
        return true;
    }

    return false;
}

__global__ void renderBVH(Mesh** meshes, int maxmesh, unsigned char* colors, const glm::mat4* Projection, const glm::mat4* View, const glm::mat4* World, int width, int height)
{
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height) return;

    const int idx = y * width + x;

    const float sampleOffsets[SPP][2] = { {0.25f, 0.25f}, {0.75f, 0.25f}, {0.25f, 0.75f}, {0.75f, 0.75f} };
    glm::vec3 finalColor(0.0f);
    float finalDepth = FLT_MAX;

    for (int s = 0; s < SPP; s++) 
    {
        const float sampleX = x + sampleOffsets[s][0];
        const float sampleY = y + sampleOffsets[s][1];
        const glm::vec2 iResolution(width, height);
        const glm::vec2 p(sampleX, height - sampleY);

        Ray ray = glup_primary_ray(p, iResolution, Projection, View, World);

        glm::vec3 accumColor(0.1f);
        float accumAlpha = 0.0f;
        float sampleDepth = FLT_MAX;
        float t_min = 0.001f;
        const int maxBounces = 5;

        for (int bounce = 0; bounce < maxBounces; bounce++) 
        {
            float closestT = FLT_MAX;
            int hitMeshIndex = -1;
            int hitTriangle = -1;
            glm::vec3 hitBary;

            for (int nummesh = 0; nummesh < maxmesh; nummesh++) 
            {
                const Mesh* mesh = meshes[nummesh];
                if (!mesh) continue;

                if (!intersectAABB(ray, mesh->bbox)) continue;

                const int numTriangles = *mesh->numindices / 3;
                for (int i = 0; i < numTriangles; i++)
                {
                    const glm::vec3 v0 = glm::vec4(mesh->vertices[mesh->indices[i * 3 + 0]], 1.0f);
                    const glm::vec3 v1 = glm::vec4(mesh->vertices[mesh->indices[i * 3 + 1]], 1.0f);
                    const glm::vec3 v2 = glm::vec4(mesh->vertices[mesh->indices[i * 3 + 2]], 1.0f);

                    float t = 0.0f;
                    glm::vec3 bary;
                    if (rayTriangleIntersect(ray.origin, ray.direction, v0, v1, v2, t, bary)) 
                    {
                        if (t < closestT && t > t_min) 
                        {
                            closestT = t;
                            hitMeshIndex = nummesh;
                            hitTriangle = i;
                            hitBary = bary;
                        }
                    }
                }
            }

            if (closestT == FLT_MAX) break;
            if (bounce == 0) sampleDepth = closestT;

            const Mesh* hitMesh = meshes[hitMeshIndex];
			if (!hitMesh) continue;

            const glm::vec4 c0 = hitMesh->colors[hitMesh->indices[hitTriangle * 3 + 0]];
            const glm::vec4 c1 = hitMesh->colors[hitMesh->indices[hitTriangle * 3 + 1]];
            const glm::vec4 c2 = hitMesh->colors[hitMesh->indices[hitTriangle * 3 + 2]];
            const glm::vec4 hitColor4 = hitBary.x * c0 + hitBary.y * c1 + hitBary.z * c2;
            const glm::vec3 hitColor = glm::vec3(hitColor4);
            const float hitAlpha = hitColor4.a;

            accumColor = hitColor * hitAlpha + accumColor;
            accumAlpha = hitAlpha + accumAlpha * (1.0f - hitAlpha);

            const glm::vec3 lightPos(500, 500, 500);
            const glm::vec3 lightDir = glm::normalize(lightPos);

            const glm::vec3 n0 = hitMesh->normals[hitMesh->indices[hitTriangle * 3 + 0]];
            const glm::vec3 n1 = hitMesh->normals[hitMesh->indices[hitTriangle * 3 + 1]];
            const glm::vec3 n2 = hitMesh->normals[hitMesh->indices[hitTriangle * 3 + 2]];

            const glm::vec3 n = hitBary.x * n0 + hitBary.y * n1 + hitBary.z * n2;
            const float diffuse = max(dot(glm::normalize(glm::mat3(*World) * n), lightDir), 0.2f);

            accumColor *= diffuse;

            if (accumAlpha > 0.95f) break;
            ray.origin += closestT * ray.direction + 0.001f * ray.direction;
        }

        finalColor += accumColor;
        finalDepth = fmin(finalDepth, sampleDepth);

    }

    finalColor /= float(SPP);
    finalColor = glm::clamp(finalColor, 0.0f, 1.0f);

    colors[idx * 3 + 0] = (unsigned char)(finalColor.r * 255.0f);
    colors[idx * 3 + 1] = (unsigned char)(finalColor.g * 255.0f);
    colors[idx * 3 + 2] = (unsigned char)(finalColor.b * 255.0f);
}

bool ProcessCuda()
{
    cudaEventRecord(start);

    cudaMemcpy(d_Projection, &Projection, sizeof(glm::mat4), cudaMemcpyHostToDevice);
    cudaMemcpy(d_View, &View, sizeof(glm::mat4), cudaMemcpyHostToDevice);
    cudaMemcpy(d_World, &World, sizeof(glm::mat4), cudaMemcpyHostToDevice);

    int maxmesh = gMeshes.size();
    dim3 blockSize(16, 16);
    dim3 numBlocks((width + blockSize.x - 1) / blockSize.x, (height + blockSize.y - 1) / blockSize.y);
    renderBVH << < numBlocks, blockSize >> > (d_meshes, maxmesh, d_colors, d_Projection, d_View, d_World, width, height);
    cudaDeviceSynchronize();
    CUDA_CHECK(cudaGetLastError());

    cudaEventRecord(stop);
    cudaEventSynchronize(stop);

    float milliseconds = 0;
    cudaEventElapsedTime(&milliseconds, start, stop);
    float fps = 1000.0f / milliseconds;
    std::cout << "T: " << milliseconds << " ms" << " - FPS: " << fps << std::endl;

    cudaMemcpy(h_colors, d_colors, width * height * 3 * sizeof(unsigned char), cudaMemcpyDeviceToHost);

    unsigned long jpegSize = 0;
    int tjStatus = tjCompress2(tjCompressor, h_colors, width, 0, height, TJPF_RGB, &jpegBuffer, &jpegSize, TJSAMP_420, 80, TJFLAG_NOREALLOC | TJFLAG_FASTDCT);
    if (tjStatus != 0) 
    {
        std::cout << "ERROR JPEG: " << tjGetErrorStr() << std::endl;
        return false;
    }

    base64 = base64_encode(jpegBuffer, jpegSize);

    return true;
}

int Initialisation(std::string filename) //./mesh/Office/Batiment Office.glb
{
    LoadGLB(filename, gMeshes);
    if (gMeshes.size() <= 0)
    {
        std::cout << "ERROR: reading file GLB." << std::endl;
        return -1;
    }
    CopyMeshToGPU(d_meshes, gMeshes);
    CUDA_CHECK(cudaGetLastError());

    h_colors = new unsigned char[width * height * 3];

    cudaMalloc(&d_colors, width * height * sizeof(unsigned char) * 3);
    CUDA_CHECK(cudaGetLastError());

    cudaMalloc(&d_depth, width * height * sizeof(float));
    CUDA_CHECK(cudaGetLastError());

    cudaEventCreate(&start);
    CUDA_CHECK(cudaGetLastError());

    cudaEventCreate(&stop);
    CUDA_CHECK(cudaGetLastError());

    glm::vec3 cameraPos = glm::vec3(0, 0, 20);
    Projection = glm::perspective(glm::radians(60.0f), (float)width / (float)height, 0.5f, 1800.0f);
    View = glm::lookAt(cameraPos, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
    World = glm::mat4(1.0f);

    cudaMalloc(&d_Projection, sizeof(glm::mat4));
    CUDA_CHECK(cudaGetLastError());

    cudaMalloc(&d_View, sizeof(glm::mat4));
    CUDA_CHECK(cudaGetLastError());

    cudaMalloc(&d_World, sizeof(glm::mat4));
	CUDA_CHECK(cudaGetLastError());

    tjCompressor = tjInitCompress();
    unsigned long maxJpegSize = tjBufSize(width, height, TJSAMP_420);
    jpegBuffer = new unsigned char[maxJpegSize];

    std::cout << "Initialisation OK." << std::endl;
    return 0;
}

std::vector<std::string> split(const std::string& str, char delimiter = '|')
{
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}

glm::vec3 screenToArcball(const glm::vec2& point, const glm::vec2& screenSize) 
{
    float x = (2.0f * point.x - screenSize.x) / screenSize.x;
    float y = (screenSize.y - 2.0f * point.y) / screenSize.y;
    float z = 0.0f;

    float lengthSquared = x * x + y * y;
    if (lengthSquared <= 1.0f) 
    {
        z = std::sqrt(1.0f - lengthSquared);
    }
    else 
    {
        float norm = 1.0f / std::sqrt(lengthSquared);
        x *= norm;
        y *= norm;
    }

    return glm::vec3(x, y, z);
}

glm::quat computeArcballQuaternion(const glm::vec2& prevMouse, const glm::vec2& currMouse, const glm::vec2& screenSize) 
{
    glm::vec3 va = screenToArcball(prevMouse, screenSize);
    glm::vec3 vb = screenToArcball(currMouse, screenSize);

    glm::vec3 axis = glm::cross(va, vb);

    float dotValue = glm::dot(va, vb);
    dotValue = std::min(1.0f, std::max(-1.0f, dotValue));
    float angle = std::acos(dotValue);

    if (glm::length(axis) < 0.0001f) return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    axis = glm::normalize(axis);
    return glm::angleAxis(angle, axis);
}

void on_message(server* s, websocketpp::connection_hdl hdl, server::message_ptr msg) 
{
    try 
    {
        std::vector<std::string> action = split(msg->get_payload());

        if (action[0] == "start")
        {
            width = atof(action[1].c_str());
            if (width <= 0)
            {
                width = 64;
                std::cout << "Width must be greater than 0, using default value of 64." << std::endl;
            }

            height = atof(action[2].c_str());
            if (height <= 0)
            {
                height = 64;
                std::cout << "Height must be greater than 0, using default value of 64." << std::endl;
            }

            std::string filename = action[3];
            if (Initialisation(filename) < 0)
            {
                std::cout << "Failed to initialize." << std::endl;
                return;
            }

            if (ProcessCuda())
            {
                websocketpp::lib::error_code ec;
                s->send(hdl, base64, websocketpp::frame::opcode::text, ec);
                if (ec) {
                    std::cerr << "ERROR SEND: " << ec.message() << std::endl;

                }
            }
        }
        else if (action[0] == "render")
        {
            if (ProcessCuda())
            {
                websocketpp::lib::error_code ec;
                s->send(hdl, base64, websocketpp::frame::opcode::text, ec);
                if (ec) {
                    std::cerr << "ERROR SEND: " << ec.message() << std::endl;

                }
            }
        }
        else if (action[0] == "mousedown")
        {
            bEventDown = true;
            eclientX = atof(action[1].c_str());
            eclientY = atof(action[2].c_str());
        }
        else if (action[0] == "mouseup")
        {
            bEventDown = false;
        }
        else if (action[0] == "dblclick")
        {
            bEventRotation = !bEventRotation;
        }
        else if (action[0] == "mousemove")
        {
            if (bEventDown)
            {
                float newX = atof(action[1].c_str());
                float newY = atof(action[2].c_str());

                if (bEventRotation)
                {
                    glm::vec2 previousMousePos(eclientX, eclientY);
                    glm::vec2 currentMousePos(newX, newY);
                    glm::vec2 screenSize(width, height);
                    glm::quat arcballQuat = computeArcballQuaternion(previousMousePos, currentMousePos, screenSize);
                    World = glm::toMat4(arcballQuat) * World;
                }
                else
                {
                    float coeff = 0.01;
                    View = glm::translate(glm::mat4(1.0f), glm::vec3(-(eclientX - newX) * coeff, (eclientY - newY) * coeff, 0.0)) * View;
                }

                eclientX = newX;
                eclientY = newY;
            }
        }
        else if (action[0] == "wheel")
        {
            float deltaX = atof(action[2].c_str());
            float deltaY = atof(action[1].c_str());

            float coeff = 0.01;
            View = glm::translate(glm::mat4(1.0f), glm::vec3(0.0, 0.0, -deltaY * coeff)) * View;
        }
    }
    catch (const websocketpp::lib::error_code& e) {
        std::cerr << "Exception: " << e.message() << std::endl;
    }
}

int main()
{
    server ws_server;

    ws_server.clear_access_channels(websocketpp::log::alevel::all);
    ws_server.clear_error_channels(websocketpp::log::elevel::all);

    ws_server.set_open_handler([](websocketpp::connection_hdl hdl) {});
    ws_server.set_message_handler(std::bind(&on_message, &ws_server, std::placeholders::_1, std::placeholders::_2));

    ws_server.init_asio();
    ws_server.listen(9002);
    ws_server.start_accept();

    std::cout << "SERVER LISTEN ON: ws://localhost:9002" << std::endl;
    ws_server.run();

    cudaFree(d_depth);
    cudaFree(d_colors);
    delete[] h_colors;
    cudaFree(d_meshes);
    return 0;
}
