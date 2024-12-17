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

class Model
{
public:
	Model(ObjectStructure* object);
	~Model();
	void Draw(Shader& shader, GLenum mode);
	std::vector<Renderer*> meshes;

private:
	std::string directory;
            
	std::vector<Texture> textures_loaded;

	ObjectStructure* modelObj;

	void loadModel(std::string path);
	void processNode(aiNode* node, const aiScene* scene);
	Renderer* processMesh(aiMesh* mesh, const aiScene* scene);
	
	std::vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type, std::string typeName);
};

#endif