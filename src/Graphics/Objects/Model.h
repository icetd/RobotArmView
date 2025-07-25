#ifndef MODEL_H
#define MODEL_H

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "../Renderer/Shader.h"
#include "../Renderer/Renderer.h"
#include "../Renderer/Texture.h"
#include "../../Utils/Utils.h"
#include "Object.h"

struct AABB {
    glm::vec3 min;
    glm::vec3 max;
};

class Model
{
public:
    Model(ObjectStructure *object);
    ~Model();
    void Draw(Shader &shader, GLenum mode);
    std::vector<Renderer *> meshes;

    AABB getAABB() const {
        return AABB{
            glm::vec3(xMin, yMin, zMin),
            glm::vec3(xMax, yMax, zMax)};
    }

    ObjectStructure *getmodelObj() { return modelObj; }

private:
    std::string directory;
    std::vector<Renderer*> meshesAABB;

    float xMin, yMin, zMin;
    float xMax, yMax, zMax;

    Utils t1;
    std::vector<float> aabbVertices;	
 
    std::vector<Texture> textures_loaded;

    ObjectStructure *modelObj;

    void loadModel(std::string path);
    void processNode(aiNode *node, const aiScene *scene);
    Renderer *processMesh(aiMesh *mesh, const aiScene *scene);
    Renderer* createAABB();

    std::vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type, std::string typeName);
};

#endif