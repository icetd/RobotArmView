#ifndef BULLET_COLLISION_MANAGER_H
#define BULLET_COLLISION_MANAGER_H

#include <btBulletCollisionCommon.h>
#include <vector>
#include <string>
#include <iostream>
#include <glm/glm.hpp>
#include "Graphics/Objects/Model.h"

class BulletCollisionManager {
public:
    struct ObjectInfo {
        std::string name;
        Model* model;
        btCollisionShape* shape = nullptr;
        btCollisionObject* collisionObject = nullptr;
        int index;
    };

    BulletCollisionManager();
    ~BulletCollisionManager();

    void addObject(const std::string& name, Model* model);
    void updateObjectTransform(int index, const glm::mat4& newTransform);
    bool checkCollisions();

    void clear();

private:
    std::vector<ObjectInfo> objects;

    btDefaultCollisionConfiguration* collisionConfig = nullptr;
    btCollisionDispatcher* dispatcher = nullptr;
    btBroadphaseInterface* broadphase = nullptr;
    btCollisionWorld* collisionWorld = nullptr;

    std::string getObjectName(const btCollisionObject* obj) const;
};

#endif // BULLET_COLLISION_MANAGER_H
