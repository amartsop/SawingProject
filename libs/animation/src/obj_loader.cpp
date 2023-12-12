#include "obj_loader.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <map>

static bool compareOBJIndexPtr(const OBJIndex* a, const OBJIndex* b);
static inline unsigned int findNextChar(unsigned int start, const char* str, unsigned int length, char token);
static inline unsigned int parseOBJIndexValue(const std::string& token, unsigned int start, unsigned int end);
static inline float parseOBJFloatValue(const std::string& token, unsigned int start, unsigned int end);
static inline std::vector<std::string> SplitString(const std::string &s, char delim);

void OBJModel::initialize(const std::string& fileName)
{
	m_hasUVs = false;
	m_hasNormals = false;
    std::ifstream file;
    file.open(fileName.c_str());

    std::string line;
    if(file.is_open())
    {
        while(file.good())
        {
            getline(file, line);
        
            unsigned int lineLength = line.length();
            
            if(lineLength < 2)
                continue;
            
            const char* lineCStr = line.c_str();
            
            switch(lineCStr[0])
            {
                case 'v':
                    if(lineCStr[1] == 't')
                        m_uvs.push_back(parseOBJVec2(line));
                    else if(lineCStr[1] == 'n')
                        m_normals.push_back(parseOBJVec3(line));
                    else if(lineCStr[1] == ' ' || lineCStr[1] == '\t')
                        m_vertices.push_back(parseOBJVec3(line));
                break;
                case 'f':
                    createOBJFace(line);
                break;
                default: break;
            };
        }
    }
    else
    {
        std::cerr << "Unable to load mesh: " << fileName << std::endl;
    }
}


IndexedModel OBJModel::ToIndexedModel(void)
{
    IndexedModel result;
    IndexedModel normalModel;

    unsigned int numIndices = m_objIndices.size();
    std::vector<OBJIndex*> indexLookup;

    for(unsigned int i = 0; i < numIndices; i++)
        indexLookup.push_back(&m_objIndices[i]);
    
    std::sort(indexLookup.begin(), indexLookup.end(), compareOBJIndexPtr);
    
    std::map<OBJIndex, unsigned int> normalModelIndexMap;
    std::map<unsigned int, unsigned int> indexMap;
    
    for(unsigned int i = 0; i < numIndices; i++)
    {
        OBJIndex* currentIndex = &m_objIndices[i];
        
        glm::vec3 currentPosition = m_vertices[currentIndex->vertexIndex];
        glm::vec2 currentTexCoord;
        glm::vec3 currentNormal;
        
        if(m_hasUVs)
            currentTexCoord = m_uvs[currentIndex->uvIndex];
        else
            currentTexCoord = glm::vec2(0,0);
            
        if(m_hasNormals)
            currentNormal = m_normals[currentIndex->normalIndex];
        else
            currentNormal = glm::vec3(0,0,0);
        
        unsigned int normalModelIndex;
        unsigned int resultModelIndex;
        
        //Create model to properly generate normals on
        std::map<OBJIndex, unsigned int>::iterator it =
            normalModelIndexMap.find(*currentIndex);

        if(it == normalModelIndexMap.end())
        {
            normalModelIndex = normalModel.positions.size();
        
            normalModelIndexMap.insert(std::pair<OBJIndex,
                unsigned int>(*currentIndex, normalModelIndex));

            normalModel.positions.push_back(currentPosition);
            normalModel.texCoords.push_back(currentTexCoord);
            normalModel.normals.push_back(currentNormal);
        }
        else
            normalModelIndex = it->second;
        
        //Create model which properly separates texture coordinates
        unsigned int previousVertexLocation = findLastVertexIndex(indexLookup,
            currentIndex, result);
        
        if(previousVertexLocation == (unsigned int)-1)
        {
            resultModelIndex = result.positions.size();
        
            result.positions.push_back(currentPosition);
            result.texCoords.push_back(currentTexCoord);
            result.normals.push_back(currentNormal);
        }
        else
            resultModelIndex = previousVertexLocation;
        
        normalModel.indices.push_back(normalModelIndex);
        result.indices.push_back(resultModelIndex);
        indexMap.insert(std::pair<unsigned int, unsigned int>(resultModelIndex,
            normalModelIndex));
    }
    
    if(!m_hasNormals)
    {
        normalModel.calc_normals();
        
        for(unsigned int i = 0; i < result.positions.size(); i++)
            result.normals[i] = normalModel.normals[indexMap[i]];
    }
    
    return result;
};

unsigned int OBJModel::findLastVertexIndex(const std::vector<OBJIndex*>&
    indexLookup, const OBJIndex* currentIndex, const IndexedModel& result)
{
    unsigned int start = 0;
    unsigned int end = indexLookup.size();
    unsigned int current = (end - start) / 2 + start;
    unsigned int previous = start;
    
    while(current != previous)
    {
        OBJIndex* testIndex = indexLookup[current];
        
        if(testIndex->vertexIndex == currentIndex->vertexIndex)
        {
            unsigned int countStart = current;
        
            for(unsigned int i = 0; i < current; i++)
            {
                OBJIndex* possibleIndex = indexLookup[current - i];
                
                if(possibleIndex == currentIndex)
                    continue;
                    
                if(possibleIndex->vertexIndex != currentIndex->vertexIndex)
                    break;
                    
                countStart--;
            }
            
            for(unsigned int i = countStart; i < indexLookup.size() - countStart; i++)
            {
                OBJIndex* possibleIndex = indexLookup[current + i];
                
                if(possibleIndex == currentIndex)
                    continue;
                    
                if(possibleIndex->vertexIndex != currentIndex->vertexIndex)
                    break;
                else if((!m_hasUVs || possibleIndex->uvIndex ==
                    currentIndex->uvIndex) 
                    && (!m_hasNormals || possibleIndex->normalIndex ==
                    currentIndex->normalIndex))
                {
                    glm::vec3 currentPosition =
                        m_vertices[currentIndex->vertexIndex];
                    glm::vec2 currentTexCoord;
                    glm::vec3 currentNormal;
                    
                    if(m_hasUVs)
                        currentTexCoord = m_uvs[currentIndex->uvIndex];
                    else
                        currentTexCoord = glm::vec2(0,0);
                        
                    if(m_hasNormals)
                        currentNormal = m_normals[currentIndex->normalIndex];
                    else
                        currentNormal = glm::vec3(0,0,0);
                    
                    for(unsigned int j = 0; j < result.positions.size(); j++)
                    {
                        if(currentPosition == result.positions[j] 
                            && ((!m_hasUVs || currentTexCoord == result.texCoords[j])
                            && (!m_hasNormals || currentNormal == result.normals[j])))
                        {
                            return j;
                        }
                    }
                }
            }
        
            return -1;
        }
        else
        {
            if(testIndex->vertexIndex < currentIndex->vertexIndex)
                start = current;
            else
                end = current;
        }
    
        previous = current;
        current = (end - start) / 2 + start;
    }
    
    return -1;
}

void OBJModel::createOBJFace(const std::string& line)
{

    std::vector<std::string> tokens = SplitString(line, ' ');

    m_objIndices.push_back(parseOBJIndex(tokens[1], &m_hasUVs, &m_hasNormals));
    m_objIndices.push_back(parseOBJIndex(tokens[2], &m_hasUVs, &m_hasNormals));
    m_objIndices.push_back(parseOBJIndex(tokens[3], &m_hasUVs, &m_hasNormals));

    if((int)tokens.size() > 4)
    {
        m_objIndices.push_back(parseOBJIndex(tokens[1], &m_hasUVs, &m_hasNormals));
        m_objIndices.push_back(parseOBJIndex(tokens[3], &m_hasUVs, &m_hasNormals));
        m_objIndices.push_back(parseOBJIndex(tokens[4], &m_hasUVs, &m_hasNormals));
    }
}

OBJIndex OBJModel::parseOBJIndex(const std::string& token, bool* hasUVs, bool* hasNormals)
{
    unsigned int tokenLength = token.length();
    const char* tokenString = token.c_str();
    
    unsigned int vertIndexStart = 0;
    unsigned int vertIndexEnd = findNextChar(vertIndexStart, tokenString, tokenLength, '/');
    
    OBJIndex result;
    result.vertexIndex = parseOBJIndexValue(token, vertIndexStart, vertIndexEnd);
    result.uvIndex = 0;
    result.normalIndex = 0;
    
    if(vertIndexEnd >= tokenLength)
        return result;
    
    vertIndexStart = vertIndexEnd + 1;
    vertIndexEnd = findNextChar(vertIndexStart, tokenString, tokenLength, '/');
    
    result.uvIndex = parseOBJIndexValue(token, vertIndexStart, vertIndexEnd);
    *hasUVs = true;
    
    if(vertIndexEnd >= tokenLength)
        return result;
    
    vertIndexStart = vertIndexEnd + 1;
    vertIndexEnd = findNextChar(vertIndexStart, tokenString, tokenLength, '/');
    
    result.normalIndex = parseOBJIndexValue(token, vertIndexStart, vertIndexEnd);
    *hasNormals = true;
    
    return result;
}

glm::vec3 OBJModel::parseOBJVec3(const std::string& line) 
{
    unsigned int tokenLength = line.length();
    const char* tokenString = line.c_str();
    
    unsigned int vertIndexStart = 2;
    
    while(vertIndexStart < tokenLength)
    {
        if(tokenString[vertIndexStart] != ' ')
            break;
        vertIndexStart++;
    }
    
    unsigned int vertIndexEnd = findNextChar(vertIndexStart, tokenString, tokenLength, ' ');
    
    float x = parseOBJFloatValue(line, vertIndexStart, vertIndexEnd);
    
    vertIndexStart = vertIndexEnd + 1;
    vertIndexEnd = findNextChar(vertIndexStart, tokenString, tokenLength, ' ');
    
    float y = parseOBJFloatValue(line, vertIndexStart, vertIndexEnd);
    
    vertIndexStart = vertIndexEnd + 1;
    vertIndexEnd = findNextChar(vertIndexStart, tokenString, tokenLength, ' ');
    
    float z = parseOBJFloatValue(line, vertIndexStart, vertIndexEnd);
    
    return glm::vec3(x,y,z);

    //glm::vec3(atof(tokens[1].c_str()), atof(tokens[2].c_str()), atof(tokens[3].c_str()))
}

glm::vec2 OBJModel::parseOBJVec2(const std::string& line)
{
    unsigned int tokenLength = line.length();
    const char* tokenString = line.c_str();
    
    unsigned int vertIndexStart = 3;
    
    while(vertIndexStart < tokenLength)
    {
        if(tokenString[vertIndexStart] != ' ')
            break;
        vertIndexStart++;
    }
    
    unsigned int vertIndexEnd = findNextChar(vertIndexStart, tokenString, tokenLength, ' ');
    
    float x = parseOBJFloatValue(line, vertIndexStart, vertIndexEnd);
    
    vertIndexStart = vertIndexEnd + 1;
    vertIndexEnd = findNextChar(vertIndexStart, tokenString, tokenLength, ' ');
    
    float y = parseOBJFloatValue(line, vertIndexStart, vertIndexEnd);
    
    return glm::vec2(x,y);
}

static bool compareOBJIndexPtr(const OBJIndex* a, const OBJIndex* b)
{
    return a->vertexIndex < b->vertexIndex;
}

static inline unsigned int findNextChar(unsigned int start, const char* str, unsigned int length, char token)
{
    unsigned int result = start;
    while(result < length)
    {
        result++;
        if(str[result] == token)
            break;
    }
    
    return result;
}

static inline unsigned int parseOBJIndexValue(const std::string& token, unsigned int start, unsigned int end)
{
    return atoi(token.substr(start, end - start).c_str()) - 1;
}

static inline float parseOBJFloatValue(const std::string& token, unsigned int start, unsigned int end)
{
    return atof(token.substr(start, end - start).c_str());
}

static inline std::vector<std::string> SplitString(const std::string &s, char delim)
{
    std::vector<std::string> elems;
        
    const char* cstr = s.c_str();
    unsigned int strLength = s.length();
    unsigned int start = 0;
    unsigned int end = 0;
        
    while(end <= strLength)
    {
        while(end <= strLength)
        {
            if(cstr[end] == delim)
                break;
            end++;
        }
            
        elems.push_back(s.substr(start, end - start));
        start = end + 1;
        end = start;
    }
        
    return elems;
}
