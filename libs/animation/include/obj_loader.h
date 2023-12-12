#pragma once

#include <vector>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>

#include <GLFW/glfw3.h>
#include "indexed_model.h"


struct OBJIndex
{
    unsigned int vertexIndex;
    unsigned int uvIndex;
    unsigned int normalIndex;
    
    bool operator<(const OBJIndex& r) const { return vertexIndex < r.vertexIndex; }
};


class OBJModel
{
public:
    
    OBJModel(){};

    // Initialize
    void initialize(const std::string& fileName);

    // Getters 
    std::vector<OBJIndex> getObjIndices(void) { return m_objIndices; }
    std::vector<glm::vec3> getVertices(void) { return m_vertices; }
    std::vector<glm::vec2> getTextureCoords(void) { return m_uvs; }
    std::vector<glm::vec3> getNormals(void) { return m_normals; }
    unsigned int getVerticesNum(void) { return m_vertices.size(); }

    // Setters
    void setVertices(std::vector<glm::vec3> vert) { m_vertices = vert; }
    void setTextureCoords(std::vector<glm::vec2> texCoords) { m_uvs = texCoords; }
    void setNormals(std::vector<glm::vec3> normals) {m_normals = normals; }

    IndexedModel ToIndexedModel(void);

private:
    std::vector<OBJIndex> m_objIndices;
    std::vector<glm::vec3> m_vertices;
    std::vector<glm::vec2> m_uvs;
    std::vector<glm::vec3> m_normals;
    bool m_hasUVs, m_hasNormals;

private:
    unsigned int findLastVertexIndex(const std::vector<OBJIndex*>& indexLookup,
        const OBJIndex* currentIndex, const IndexedModel& result);

    void createOBJFace(const std::string& line);
    
    glm::vec2 parseOBJVec2(const std::string& line);
    glm::vec3 parseOBJVec3(const std::string& line);
    OBJIndex parseOBJIndex(const std::string& token, bool* hasUVs, bool* hasNormals);
};