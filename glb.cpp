#include "glb.h"
#include <iostream>
#include <vector>

#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define TINYGLTF_NOEXCEPTION
#define JSON_NOEXCEPTION
#include "./tinygltf/tiny_gltf.h"

std::vector<glm::mat4> popping;
glm::mat4 currentmatrix;

void push(glm::mat4 p) { popping.push_back(p); }
glm::mat4 pop() { if (!popping.empty()) { glm::mat4 m = popping.back(); popping.pop_back(); return m; } return glm::mat4(1.0); }

size_t ComponentTypeByteSize(int type) 
{
    switch (type) 
    {
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
    case TINYGLTF_COMPONENT_TYPE_BYTE:
        return sizeof(char);
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
    case TINYGLTF_COMPONENT_TYPE_SHORT:
        return sizeof(short);
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
    case TINYGLTF_COMPONENT_TYPE_INT:
        return sizeof(int);
    case TINYGLTF_COMPONENT_TYPE_FLOAT:
        return sizeof(float);
    case TINYGLTF_COMPONENT_TYPE_DOUBLE:
        return sizeof(float);
    default:
        return 0;
    }
}

glm::vec4 getPixelColor(glm::vec2 uv, unsigned char* textureData, int textureWidth, int textureHeight, int channels) 
{
    int pixelX = static_cast<int>(uv.x * textureWidth);
    int pixelY = static_cast<int>(uv.y * textureHeight);
    int index = (pixelY * textureWidth + pixelX) * channels;

    glm::vec4 color = glm::vec4(textureData[index + 0] / 255.0, textureData[index + 1] / 255.0, textureData[index + 2] / 255.0, textureData[index + 3] / 255.0);
    return color;
}

void ComputeMatrixNode(tinygltf::Model& model, tinygltf::Node& node)
{
    push(currentmatrix);

    if (node.matrix.size() > 0)
    {
        glm::mat4 tmp = glm::mat4(1.0);
        tmp = glm::mat4(node.matrix[0], node.matrix[1], node.matrix[2], node.matrix[3], node.matrix[4], node.matrix[5], node.matrix[6], node.matrix[7], node.matrix[8], node.matrix[9], node.matrix[10], node.matrix[11], node.matrix[12], node.matrix[13], node.matrix[14], node.matrix[15]);
        glm::vec3 scale, translation, skew;
        glm::quat rotation;
        glm::vec4 perspective;
        glm::decompose(tmp, scale, rotation, translation, skew, perspective);

        node.translation.push_back(translation.x);
        node.translation.push_back(translation.y);
        node.translation.push_back(translation.z);

        node.scale.push_back(scale.x);
        node.scale.push_back(scale.y);
        node.scale.push_back(scale.z);

        node.rotation.push_back(rotation.w);
        node.rotation.push_back(rotation.x);
        node.rotation.push_back(rotation.y);
        node.rotation.push_back(rotation.z);
        node.matrix.clear();
    }

    glm::mat4 trans = glm::mat4(1.0);
    if (node.translation.size() > 0)
    {
        glm::mat4 t = glm::translate(glm::mat4(1.0), glm::vec3(node.translation[0], node.translation[1], node.translation[2]));
        trans *= t;
    }
    else
    {
        node.translation.resize(3);
        node.translation[0] = 0.0;
        node.translation[1] = 0.0;
        node.translation[2] = 0.0;
    }

    if (node.rotation.size() > 0)
    {
        glm::quat q(node.rotation[3], node.rotation[0], node.rotation[1], node.rotation[2]);
        trans *= glm::toMat4(q);
    }
    else
    {
        node.rotation.resize(4);
        node.rotation[0] = 0.0;
        node.rotation[1] = 0.0;
        node.rotation[2] = 0.0;
        node.rotation[3] = 1.0;
    }

    if (node.scale.size() > 0)
    {
        glm::mat4 s = glm::scale(glm::mat4(1.0), glm::vec3(node.scale[0], node.scale[1], node.scale[2]));
        trans *= s;
    }
    else
    {
        node.scale.resize(3);
        node.scale[0] = 1.0;
        node.scale[1] = 1.0;
        node.scale[2] = 1.0;
    }

    currentmatrix *= trans;
    node.matrix_transformation = currentmatrix;

    // Draw child nodes.
    for (size_t i = 0; i < node.children.size(); i++)
    {
        ComputeMatrixNode(model, model.nodes[node.children[i]]);
    }

    currentmatrix = pop();
}

void ComputeMatrix(tinygltf::Model& model)
{
    int scene_to_display = model.defaultScene > -1 ? model.defaultScene : 0;
    const tinygltf::Scene& scene = model.scenes[scene_to_display];
    for (size_t i = 0; i < scene.nodes.size(); i++)
    {
        if (scene.nodes[i] < 0) continue;

        popping.clear();
        currentmatrix = glm::mat4(1.0f);
        push(currentmatrix);

        ComputeMatrixNode(model, model.nodes[scene.nodes[i]]);

        currentmatrix = pop();
    }
}

void LoadGLB(const std::string& filename, std::vector<Mesh*>& meshes) {

    tinygltf::Model model;
    tinygltf::TinyGLTF loader;
    std::string err, warn;

    // Charger le fichier GLB
    bool success = loader.LoadBinaryFromFile(&model, &err, &warn, filename);
    if (!success) {
        std::cout << "ERROR GLB: " << err << std::endl;
        return;
    }

    ComputeMatrix(model);

    glm::vec4 color = glm::vec4(1.0f);
    for (const auto& node : model.nodes)
    {
        if (node.mesh < 0) continue;

        auto mesh = model.meshes[node.mesh];

        for (auto primitive : mesh.primitives)
        {
            if (primitive.indices < 0) continue;
            
            Mesh* mMesh = new Mesh();
            mMesh->bbox = new AABB();
            mMesh->matrix = new glm::mat4(1.0);
            mMesh->numvertices = new int;
            mMesh->numindices = new int;

            tinygltf::Image* image = nullptr;
            memcpy(mMesh->matrix, &node.matrix_transformation, sizeof(glm::mat4));

            if (primitive.material >= 0)
            {
                const auto& material = model.materials[primitive.material];
                if (material.baseColoFactor.size() == 4) 
                {
                    color.x = material.baseColoFactor[0];
                    color.y = material.baseColoFactor[1];
                    color.z = material.baseColoFactor[2];
                    color.w = material.baseColoFactor[3];
                }

                if (model.textures.size() > 0)
                {
                    int index = material.pbrMetallicRoughness.baseColorTexture.index;
                    if (index > -1) 
                    {
                        tinygltf::Texture& tex = model.textures[index];
                        image = &model.images[tex.source];
                    }
                }
            }

            {
                if (primitive.mode != TINYGLTF_MODE_TRIANGLES) 
                {
                    std::cout << "not supported mode: " << primitive.mode << std::endl;
                    continue;
                }

                const tinygltf::Accessor& indexAccessor = model.accessors[primitive.indices];
                if (indexAccessor.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT) 
                {
                    std::cout << "indices not supported: " << indexAccessor.componentType << std::endl;
                    continue;
                }

                const tinygltf::BufferView& indexBufferView = model.bufferViews[indexAccessor.bufferView];
                const tinygltf::Buffer& indexBuffer = model.buffers[indexBufferView.buffer];
                if (indexAccessor.type != TINYGLTF_TYPE_SCALAR) 
                {
                    std::cout << "vertices not supported" << std::endl;
                    continue;
                }

                const auto byteStride = indexAccessor.ByteStride(indexBufferView);
                const tinygltf::Buffer& buffer = model.buffers[indexBufferView.buffer];
                int size = 1;
                if (indexAccessor.type == TINYGLTF_TYPE_SCALAR) 
                {
                    size = 1;
                }
                else if (indexAccessor.type == TINYGLTF_TYPE_VEC2) 
                {
                    size = 2;
                }
                else if (indexAccessor.type == TINYGLTF_TYPE_VEC3) 
                {
                    size = 3;
                }
                else if (indexAccessor.type == TINYGLTF_TYPE_VEC4) 
                {
                    size = 4;
                }
                else 
                {
                }

                if (!indexAccessor.sparse.isSparse)
                {
                    std::vector<unsigned char> positions;
                    const unsigned char* ptr = indexBuffer.data.data() + indexBufferView.byteOffset;
                    const unsigned char* ptrindex = ptr;

                    *mMesh->numindices = indexAccessor.count;
                    mMesh->indices = new  unsigned short[*mMesh->numindices];
                    for (int pn = 0; pn < (indexBufferView.byteLength / byteStride); pn++)
                    {
                        ptrindex = ptr + byteStride * pn;
                        ptrindex += indexAccessor.byteOffset;
                        for (int ns = 0; ns < size * sizeof(unsigned short); ns++)
                        {
                            positions.push_back(*ptrindex);
                            ptrindex++;
                        }
                    }

                    std::vector<unsigned short> positionsS;
                    positionsS.resize(indexAccessor.count);
                    memcpy(positionsS.data(), positions.data(), indexAccessor.count * sizeof(unsigned short));

                    for (size_t p = 0; p < positionsS.size(); p++)
                        mMesh->indices[p] = positionsS[p];
                }
                else
                {
                    unsigned char* tmp_buffer = new unsigned char[indexBufferView.byteLength];
                    memcpy(tmp_buffer, indexBuffer.data.data() + indexBufferView.byteOffset, indexBufferView.byteLength);

                    const size_t size_of_object_in_buffer = ComponentTypeByteSize(indexAccessor.componentType);
                    const size_t size_of_sparse_indices = ComponentTypeByteSize(indexAccessor.sparse.indices.componentType);

                    const auto& indices_buffer_view = model.bufferViews[indexAccessor.sparse.indices.bufferView];
                    const auto& indices_buffer = model.buffers[indices_buffer_view.buffer];

                    const auto& values_buffer_view = model.bufferViews[indexAccessor.sparse.values.bufferView];
                    const auto& values_buffer = model.buffers[values_buffer_view.buffer];

                    for (size_t sparse_index = 0; sparse_index < indexAccessor.sparse.count; ++sparse_index)
                    {
                        int index = 0;
                        switch (indexAccessor.sparse.indices.componentType) 
                        {
                        case TINYGLTF_COMPONENT_TYPE_BYTE:
                        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
                            index = (int)*(unsigned char*)(indices_buffer.data.data() + indices_buffer_view.byteOffset + indexAccessor.sparse.indices.byteOffset + (sparse_index * size_of_sparse_indices));
                            break;
                        case TINYGLTF_COMPONENT_TYPE_SHORT:
                        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
                            index = (int)*(unsigned short*)(indices_buffer.data.data() + indices_buffer_view.byteOffset + indexAccessor.sparse.indices.byteOffset + (sparse_index * size_of_sparse_indices));
                            break;
                        case TINYGLTF_COMPONENT_TYPE_INT:
                        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
                            index = (int)*(unsigned int*)(indices_buffer.data.data() + indices_buffer_view.byteOffset + indexAccessor.sparse.indices.byteOffset + (sparse_index * size_of_sparse_indices));
                            break;
                        }

                        const unsigned char* read_from = values_buffer.data.data() + (values_buffer_view.byteOffset + indexAccessor.sparse.values.byteOffset) + (sparse_index * (size_of_object_in_buffer * indexAccessor.type));
                        unsigned char* write_to = tmp_buffer + index * (size_of_object_in_buffer * indexAccessor.type);
                        memcpy(write_to, read_from, size_of_object_in_buffer * indexAccessor.type);
                    }

                    int byteStride = indexAccessor.ByteStride(values_buffer_view);

                    std::vector<unsigned char> positions;
                    const unsigned char* ptr = tmp_buffer;
                    const unsigned char* ptrindex = ptr;

                    *mMesh->numindices = indexAccessor.count;
                    mMesh->indices = new  unsigned short[*mMesh->numindices];
                    for (int pn = 0; pn < (values_buffer_view.byteLength / byteStride); pn++)
                    {
                        ptrindex = ptr + byteStride * pn;
                        ptrindex += indexAccessor.byteOffset;
                        for (int ns = 0; ns < size * sizeof(unsigned short); ns++)
                        {
                            positions.push_back(*ptrindex);
                            ptrindex++;
                        }
                    }

                    memcpy(mMesh->indices, positions.data(), indexAccessor.count * sizeof(unsigned short));

                    delete[] tmp_buffer;
                }
            }

            for (std::map<std::string, int>::const_iterator it = primitive.attributes.begin(); it != primitive.attributes.end(); it++)
            {
                if (it->first.compare("POSITION") == 0)
                {
                    const tinygltf::Accessor& accessor = model.accessors[it->second];

                    tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
                    if (bufferView.target == 0) continue;

                    const tinygltf::Buffer& buffer = model.buffers[bufferView.buffer];
                    int size = 1;
                    if (accessor.type == TINYGLTF_TYPE_SCALAR) 
                    {
                        size = 1;
                    }
                    else if (accessor.type == TINYGLTF_TYPE_VEC2) 
                    {
                        size = 2;
                    }
                    else if (accessor.type == TINYGLTF_TYPE_VEC3) 
                    {
                        size = 3;
                    }
                    else if (accessor.type == TINYGLTF_TYPE_VEC4) 
                    {
                        size = 4;
                    }
                    else 
                    {

                    }

                    if (!accessor.sparse.isSparse)
                    {
                        int byteStride = accessor.ByteStride(model.bufferViews[accessor.bufferView]);
                        const tinygltf::Buffer& buffer = model.buffers[bufferView.buffer];

                        std::vector<unsigned char> positions;
                        const unsigned char* ptr = &buffer.data.at(0) + bufferView.byteOffset;
                        const unsigned char* ptrindex = ptr;

                        *mMesh->numvertices = accessor.count;
                        mMesh->vertices = new glm::vec3[*mMesh->numvertices];

                        mMesh->colors = new glm::vec4[accessor.count];
                        for (int i = 0; i < accessor.count; i++) mMesh->colors[i] = color;

                        for (int pn = 0; pn < (bufferView.byteLength / byteStride); pn++)
                        {
                            ptrindex = ptr + byteStride * pn;
                            ptrindex += accessor.byteOffset;
                            for (int ns = 0; ns < size * sizeof(float); ns++)
                            {
                                positions.push_back(*ptrindex);
                                ptrindex++;
                            }
                        }

                        memcpy(mMesh->vertices, positions.data(), *mMesh->numvertices * sizeof(glm::vec3));
                    }
                    else
                    {
                        unsigned char* tmp_buffer = new unsigned char[bufferView.byteLength];
                        memcpy(tmp_buffer, buffer.data.data() + bufferView.byteOffset, bufferView.byteLength);

                        const size_t size_of_object_in_buffer = ComponentTypeByteSize(accessor.componentType);
                        const size_t size_of_sparse_indices = ComponentTypeByteSize(accessor.sparse.indices.componentType);

                        const auto& indices_buffer_view = model.bufferViews[accessor.sparse.indices.bufferView];
                        const auto& indices_buffer = model.buffers[indices_buffer_view.buffer];

                        const auto& values_buffer_view = model.bufferViews[accessor.sparse.values.bufferView];
                        const auto& values_buffer = model.buffers[values_buffer_view.buffer];

                        for (size_t sparse_index = 0; sparse_index < accessor.sparse.count; ++sparse_index)
                        {
                            int index = 0;
                            switch (accessor.sparse.indices.componentType) 
                            {
                            case TINYGLTF_COMPONENT_TYPE_BYTE:
                            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
                                index = (int)*(unsigned char*)(indices_buffer.data.data() + indices_buffer_view.byteOffset + accessor.sparse.indices.byteOffset + (sparse_index * size_of_sparse_indices));
                                break;
                            case TINYGLTF_COMPONENT_TYPE_SHORT:
                            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
                                index = (int)*(unsigned short*)(indices_buffer.data.data() + indices_buffer_view.byteOffset + accessor.sparse.indices.byteOffset + (sparse_index * size_of_sparse_indices));
                                break;
                            case TINYGLTF_COMPONENT_TYPE_INT:
                            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
                                index = (int)*(unsigned int*)(indices_buffer.data.data() + indices_buffer_view.byteOffset + accessor.sparse.indices.byteOffset + (sparse_index * size_of_sparse_indices));
                                break;
                            }

                            const unsigned char* read_from = values_buffer.data.data() + (values_buffer_view.byteOffset + accessor.sparse.values.byteOffset) + (sparse_index * (size_of_object_in_buffer * accessor.type));
                            unsigned char* write_to = tmp_buffer + index * (size_of_object_in_buffer * accessor.type);
                            memcpy(write_to, read_from, size_of_object_in_buffer * accessor.type);
                        }

                        int byteStride = accessor.ByteStride(values_buffer_view);
                        std::vector<unsigned char> positions;
                        const unsigned char* ptr = tmp_buffer;
                        const unsigned char* ptrindex = ptr;

                        *mMesh->numvertices = accessor.count;
                        mMesh->vertices = new glm::vec3[*mMesh->numvertices];
                        mMesh->colors = new glm::vec4[accessor.count];
                        for (int i = 0; i < accessor.count; i++) mMesh->colors[i] = color;

                        for (int pn = 0; pn < (values_buffer_view.byteLength / byteStride); pn++)
                        {
                            ptrindex = ptr + byteStride * pn;
                            ptrindex += accessor.byteOffset;
                            for (int ns = 0; ns < size * sizeof(float); ns++)
                            {
                                positions.push_back(*ptrindex);
                                ptrindex++;
                            }
                        }

                        memcpy(mMesh->vertices, positions.data(), *mMesh->numvertices * sizeof(glm::vec3));

                        delete[] tmp_buffer;
                    }
                }
                else if (it->first.compare("NORMAL") == 0)
                {
                    const tinygltf::Accessor& accessor = model.accessors[it->second];

                    tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
                    if (bufferView.target == 0) continue;

                    const tinygltf::Buffer& buffer = model.buffers[bufferView.buffer];
                    int size = 1;
                    if (accessor.type == TINYGLTF_TYPE_SCALAR) 
                    {
                        size = 1;
                    }
                    else if (accessor.type == TINYGLTF_TYPE_VEC2) 
                    {
                        size = 2;
                    }
                    else if (accessor.type == TINYGLTF_TYPE_VEC3) 
                    {
                        size = 3;
                    }
                    else if (accessor.type == TINYGLTF_TYPE_VEC4) 
                    {
                        size = 4;
                    }
                    else 
                    {

                    }

                    if (!accessor.sparse.isSparse)
                    {
                        int byteStride = accessor.ByteStride(model.bufferViews[accessor.bufferView]);
                        const tinygltf::Buffer& buffer = model.buffers[bufferView.buffer];

                        std::vector<unsigned char> positions;
                        const unsigned char* ptr = &buffer.data.at(0) + bufferView.byteOffset;
                        const unsigned char* ptrindex = ptr;

                        mMesh->normals = new glm::vec3[accessor.count];
                        for (int pn = 0; pn < (bufferView.byteLength / byteStride); pn++)
                        {
                            ptrindex = ptr + byteStride * pn;
                            ptrindex += accessor.byteOffset;
                            for (int ns = 0; ns < size * sizeof(float); ns++)
                            {
                                positions.push_back(*ptrindex);
                                ptrindex++;
                            }
                        }

                        memcpy(mMesh->normals, positions.data(), accessor.count * sizeof(float3));
                    }
                    else
                    {
                        unsigned char* tmp_buffer = new unsigned char[bufferView.byteLength];
                        memcpy(tmp_buffer, buffer.data.data() + bufferView.byteOffset, bufferView.byteLength);

                        const size_t size_of_object_in_buffer = ComponentTypeByteSize(accessor.componentType);
                        const size_t size_of_sparse_indices = ComponentTypeByteSize(accessor.sparse.indices.componentType);

                        const auto& indices_buffer_view = model.bufferViews[accessor.sparse.indices.bufferView];
                        const auto& indices_buffer = model.buffers[indices_buffer_view.buffer];

                        const auto& values_buffer_view = model.bufferViews[accessor.sparse.values.bufferView];
                        const auto& values_buffer = model.buffers[values_buffer_view.buffer];

                        for (size_t sparse_index = 0; sparse_index < accessor.sparse.count; ++sparse_index)
                        {
                            int index = 0;
                            switch (accessor.sparse.indices.componentType) 
                            {
                            case TINYGLTF_COMPONENT_TYPE_BYTE:
                            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
                                index = (int)*(unsigned char*)(indices_buffer.data.data() + indices_buffer_view.byteOffset + accessor.sparse.indices.byteOffset + (sparse_index * size_of_sparse_indices));
                                break;
                            case TINYGLTF_COMPONENT_TYPE_SHORT:
                            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
                                index = (int)*(unsigned short*)(indices_buffer.data.data() + indices_buffer_view.byteOffset + accessor.sparse.indices.byteOffset + (sparse_index * size_of_sparse_indices));
                                break;
                            case TINYGLTF_COMPONENT_TYPE_INT:
                            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
                                index = (int)*(unsigned int*)(indices_buffer.data.data() + indices_buffer_view.byteOffset + accessor.sparse.indices.byteOffset + (sparse_index * size_of_sparse_indices));
                                break;
                            }

                            const unsigned char* read_from = values_buffer.data.data() + (values_buffer_view.byteOffset + accessor.sparse.values.byteOffset) + (sparse_index * (size_of_object_in_buffer * accessor.type));
                            unsigned char* write_to = tmp_buffer + index * (size_of_object_in_buffer * accessor.type);
                            memcpy(write_to, read_from, size_of_object_in_buffer * accessor.type);
                        }

                        int byteStride = accessor.ByteStride(values_buffer_view);
                        std::vector<unsigned char> positions;
                        const unsigned char* ptr = tmp_buffer;
                        const unsigned char* ptrindex = ptr;

                        mMesh->normals = new glm::vec3[accessor.count];
                        for (int pn = 0; pn < (values_buffer_view.byteLength / byteStride); pn++)
                        {
                            ptrindex = ptr + byteStride * pn;
                            ptrindex += accessor.byteOffset;
                            for (int ns = 0; ns < size * sizeof(float); ns++)
                            {
                                positions.push_back(*ptrindex);
                                ptrindex++;
                            }
                        }

                        memcpy(mMesh->normals, positions.data(), accessor.count * sizeof(float3));

                        delete[] tmp_buffer;
                    }
                }
                else if (it->first.compare("TEXCOORD_0") == 0)
                {
                    const tinygltf::Accessor& accessor = model.accessors[it->second];

                    tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
                    if (bufferView.target == 0) continue;

                    const tinygltf::Buffer& buffer = model.buffers[bufferView.buffer];
                    int size = 1;
                    if (accessor.type == TINYGLTF_TYPE_SCALAR) 
                    {
                        size = 1;
                    }
                    else if (accessor.type == TINYGLTF_TYPE_VEC2) 
                    {
                        size = 2;
                    }
                    else if (accessor.type == TINYGLTF_TYPE_VEC3) 
                    {
                        size = 3;
                    }
                    else if (accessor.type == TINYGLTF_TYPE_VEC4) 
                    {
                        size = 4;
                    }
                    else 
                    {

                    }

                    if (!accessor.sparse.isSparse)
                    {
                        int byteStride = accessor.ByteStride(model.bufferViews[accessor.bufferView]);
                        const tinygltf::Buffer& buffer = model.buffers[bufferView.buffer];

                        std::vector<unsigned char> positions;
                        const unsigned char* ptr = &buffer.data.at(0) + bufferView.byteOffset;
                        const unsigned char* ptrindex = ptr;

                        glm::vec2* uvs = new glm::vec2[accessor.count];
                        for (int pn = 0; pn < (bufferView.byteLength / byteStride); pn++)
                        {
                            ptrindex = ptr + byteStride * pn;
                            ptrindex += accessor.byteOffset;
                            for (int ns = 0; ns < size * sizeof(float); ns++)
                            {
                                positions.push_back(*ptrindex);
                                ptrindex++;
                            }
                        }

                        memcpy(uvs, positions.data(), accessor.count * sizeof(float2));

                        mMesh->colors = new glm::vec4[accessor.count];
                        for (int i = 0; i < accessor.count; i++) mMesh->colors[i] = color;

                        if (image)
                        {
                            for (size_t ip = 0; ip < accessor.count; ++ip)
                            {
                                mMesh->colors[ip] = getPixelColor(uvs[ip], image->image.data(), image->width, image->height, image->component);
                            }
                        }

                        delete[] uvs;
                    }
                    else
                    {
                        unsigned char* tmp_buffer = new unsigned char[bufferView.byteLength];
                        memcpy(tmp_buffer, buffer.data.data() + bufferView.byteOffset, bufferView.byteLength);

                        const size_t size_of_object_in_buffer = ComponentTypeByteSize(accessor.componentType);
                        const size_t size_of_sparse_indices = ComponentTypeByteSize(accessor.sparse.indices.componentType);

                        const auto& indices_buffer_view = model.bufferViews[accessor.sparse.indices.bufferView];
                        const auto& indices_buffer = model.buffers[indices_buffer_view.buffer];

                        const auto& values_buffer_view = model.bufferViews[accessor.sparse.values.bufferView];
                        const auto& values_buffer = model.buffers[values_buffer_view.buffer];

                        for (size_t sparse_index = 0; sparse_index < accessor.sparse.count; ++sparse_index)
                        {
                            int index = 0;
                            switch (accessor.sparse.indices.componentType) 
                            {
                            case TINYGLTF_COMPONENT_TYPE_BYTE:
                            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
                                index = (int)*(unsigned char*)(indices_buffer.data.data() + indices_buffer_view.byteOffset + accessor.sparse.indices.byteOffset + (sparse_index * size_of_sparse_indices));
                                break;
                            case TINYGLTF_COMPONENT_TYPE_SHORT:
                            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
                                index = (int)*(unsigned short*)(indices_buffer.data.data() + indices_buffer_view.byteOffset + accessor.sparse.indices.byteOffset + (sparse_index * size_of_sparse_indices));
                                break;
                            case TINYGLTF_COMPONENT_TYPE_INT:
                            case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
                                index = (int)*(unsigned int*)(indices_buffer.data.data() + indices_buffer_view.byteOffset + accessor.sparse.indices.byteOffset + (sparse_index * size_of_sparse_indices));
                                break;
                            }

                            const unsigned char* read_from = values_buffer.data.data() + (values_buffer_view.byteOffset + accessor.sparse.values.byteOffset) + (sparse_index * (size_of_object_in_buffer * accessor.type));
                            unsigned char* write_to = tmp_buffer + index * (size_of_object_in_buffer * accessor.type);
                            memcpy(write_to, read_from, size_of_object_in_buffer * accessor.type);
                        }

                        int byteStride = accessor.ByteStride(values_buffer_view);
                        std::vector<unsigned char> positions;
                        const unsigned char* ptr = tmp_buffer;
                        const unsigned char* ptrindex = ptr;

                        glm::vec2* uvs = new glm::vec2[accessor.count];
                        for (int pn = 0; pn < (values_buffer_view.byteLength / byteStride); pn++)
                        {
                            ptrindex = ptr + byteStride * pn;
                            ptrindex += accessor.byteOffset;
                            for (int ns = 0; ns < size * sizeof(float); ns++)
                            {
                                positions.push_back(*ptrindex);
                                ptrindex++;
                            }
                        }

                        memcpy(uvs, positions.data(), accessor.count * sizeof(float2));

                        mMesh->colors = new glm::vec4[accessor.count];
                        for (int i = 0; i < accessor.count; i++) mMesh->colors[i] = color;

                        if (image)
                        {
                            for (size_t ip = 0; ip < accessor.count; ++ip) 
                            {
                                mMesh->colors[ip] = getPixelColor(uvs[ip], image->image.data(), image->width, image->height, 4);
                            }
                        }

                        delete[] uvs;


                        delete[] tmp_buffer;
                    }
                }
            }

            meshes.push_back(mMesh);
        }
    }

    std::cout << "Loading complete, \"" << filename << "\" number mesh: " << std::to_string(meshes.size()) << std::endl;
}


void InitMesh(std::vector<Mesh*> meshes)
{
    for (int i = 0; i < meshes.size(); i++)
    {
        Mesh* mesh = meshes[i];
        if (mesh->numindices == 0) continue;

        glm::vec3 min = glm::vec3(FLT_MAX, FLT_MAX, FLT_MAX);
        glm::vec3 max = glm::vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        for (int j = 0; j < *mesh->numvertices; j++)
        {
            glm::vec3 v = *mesh->matrix * glm::vec4(mesh->vertices[j], 1.0);
            mesh->vertices[j] = v;

            glm::vec3 n = glm::mat3(*mesh->matrix) * mesh->normals[j];
            mesh->normals[j] = glm::normalize(n);

            min = glm::min(min, v);
            max = glm::max(max, v);
        }
        
        mesh->bbox->min = min;
        mesh->bbox->max = max;
    }
}

void CopyMeshToGPU(Mesh**& d_meshes, const std::vector<Mesh*>& meshes)
{
    InitMesh(meshes);

    cudaMalloc(&d_meshes, meshes.size() * sizeof(Mesh*));
    CUDA_CHECK(cudaGetLastError());

    Mesh** meshArray = new Mesh * [meshes.size()];

    for (size_t i = 0; i < meshes.size(); i++) 
    {
        Mesh* mesh = meshes[i];
        if (!mesh) continue;
        if (*mesh->numindices <= 0) continue;
        
        int totalVertexSize = *mesh->numvertices * sizeof(glm::vec3);
        glm::vec3* d_allVertices = nullptr;
        cudaMalloc(&d_allVertices, totalVertexSize);
        CUDA_CHECK(cudaGetLastError());

        cudaMemcpy(d_allVertices, mesh->vertices, totalVertexSize, cudaMemcpyHostToDevice);
        CUDA_CHECK(cudaGetLastError());

        int totalNormalSize = *mesh->numvertices * sizeof(glm::vec3);
        glm::vec3* d_allNormals = nullptr;
        cudaMalloc(&d_allNormals, totalNormalSize);
        CUDA_CHECK(cudaGetLastError());

        cudaMemcpy(d_allNormals, mesh->normals, totalNormalSize, cudaMemcpyHostToDevice);
        CUDA_CHECK(cudaGetLastError());

        int totalColorSize = *mesh->numvertices * sizeof(glm::vec4);
        glm::vec4* d_allColor = nullptr;
        cudaMalloc(&d_allColor, totalColorSize);
        CUDA_CHECK(cudaGetLastError());

        cudaMemcpy(d_allColor, mesh->colors, totalColorSize, cudaMemcpyHostToDevice);
        CUDA_CHECK(cudaGetLastError());

        int totalIndexSize = *mesh->numindices * sizeof(unsigned short);
        unsigned short* d_allIndices = nullptr;
        cudaMalloc(&d_allIndices, totalIndexSize);
        CUDA_CHECK(cudaGetLastError());

        cudaMemcpy(d_allIndices, mesh->indices, totalIndexSize, cudaMemcpyHostToDevice);
        CUDA_CHECK(cudaGetLastError());


        AABB* d_allbbox = nullptr;
        cudaMalloc(&d_allbbox, sizeof(AABB));
        CUDA_CHECK(cudaGetLastError());

        cudaMemcpy(d_allbbox, mesh->bbox, sizeof(AABB), cudaMemcpyHostToDevice);
        CUDA_CHECK(cudaGetLastError());


        glm::mat4* d_allmatrix = nullptr;
        cudaMalloc(&d_allmatrix, sizeof(glm::mat4));
        CUDA_CHECK(cudaGetLastError());

        cudaMemcpy(d_allmatrix, mesh->matrix, sizeof(glm::mat4), cudaMemcpyHostToDevice);
        CUDA_CHECK(cudaGetLastError());
        

        int* d_allnumvertices = nullptr;
        cudaMalloc(&d_allnumvertices, sizeof(int));
        CUDA_CHECK(cudaGetLastError());

        cudaMemcpy(d_allnumvertices, mesh->numvertices, sizeof(int), cudaMemcpyHostToDevice);
        CUDA_CHECK(cudaGetLastError());


        int* d_allnumindices = nullptr;
        cudaMalloc(&d_allnumindices, sizeof(int));
        CUDA_CHECK(cudaGetLastError());

        cudaMemcpy(d_allnumindices, mesh->numindices, sizeof(int), cudaMemcpyHostToDevice);
        CUDA_CHECK(cudaGetLastError());

        Mesh hmesh;
        hmesh.numindices = d_allnumindices;
        hmesh.numvertices = d_allnumvertices;
        hmesh.bbox = d_allbbox;
        hmesh.matrix = d_allmatrix;
        hmesh.vertices = d_allVertices;
        hmesh.normals = d_allNormals;
        hmesh.colors = d_allColor;
        hmesh.indices = d_allIndices;

        Mesh* dmesh = nullptr;
        cudaMalloc(&dmesh, sizeof(Mesh));
        cudaMemcpy(dmesh, &hmesh, sizeof(Mesh), cudaMemcpyHostToDevice);
		CUDA_CHECK(cudaGetLastError());

        meshArray[i] = dmesh;
    }

    cudaMemcpy(d_meshes, meshArray, meshes.size() * sizeof(Mesh*), cudaMemcpyHostToDevice);
    CUDA_CHECK(cudaGetLastError());
    delete[] meshArray;
}