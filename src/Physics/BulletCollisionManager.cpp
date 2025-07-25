#include "BulletCollisionManager.h"

BulletCollisionManager::BulletCollisionManager() {
    collisionConfig = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfig);
    broadphase = new btDbvtBroadphase();
    collisionWorld = new btCollisionWorld(dispatcher, broadphase, collisionConfig);
}

BulletCollisionManager::~BulletCollisionManager() {
    for (auto& obj : objects) {
        collisionWorld->removeCollisionObject(obj.collisionObject);
        delete obj.collisionObject;
        delete obj.shape;
    }
    delete collisionWorld;
    delete broadphase;
    delete dispatcher;
    delete collisionConfig;
}

void BulletCollisionManager::addObject(const std::string& name, Model* model) {
    int index = objects.size();  // 自动编号
    AABB aabb = model->getAABB();
    btVector3 halfExtents(
        (aabb.max.x - aabb.min.x) / 2.0f,
        (aabb.max.y - aabb.min.y) / 2.0f,
        (aabb.max.z - aabb.min.z) / 2.0f);

    btBoxShape* boxShape = new btBoxShape(halfExtents);

    glm::vec3 center = (aabb.min + aabb.max) * 0.5f;

    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(center.x, center.y, center.z));

    btCollisionObject* colObj = new btCollisionObject();
    colObj->setCollisionShape(boxShape);
    colObj->setWorldTransform(startTransform);

    collisionWorld->addCollisionObject(colObj);

    objects.push_back({name, model, boxShape, colObj, index});
}

void BulletCollisionManager::updateObjectTransform(int index, const glm::mat4& newTransform) {
    if (index < 0 || index >= (int)objects.size()) return;

    btTransform btTrans;
    btTrans.setFromOpenGLMatrix(glm::value_ptr(newTransform));

    objects[index].collisionObject->setWorldTransform(btTrans);

    collisionWorld->updateSingleAabb(objects[index].collisionObject);
}

bool BulletCollisionManager::checkCollisions() {
    collisionWorld->performDiscreteCollisionDetection();

    int numManifolds = dispatcher->getNumManifolds();
    for (int i = 0; i < numManifolds; i++) {
        btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(i);
        const btCollisionObject* obA = manifold->getBody0();
        const btCollisionObject* obB = manifold->getBody1();

        std::string nameA = getObjectName(obA);
        std::string nameB = getObjectName(obB);

        int indexA = -1, indexB = -1;
        // 从 objects 列表中查找索引
        for (const auto& obj : objects) {
            if (obj.collisionObject == obA) 
                indexA = obj.index;
            if (obj.collisionObject == obB) 
                indexB = obj.index;
        }

        // 跳过相邻 link（例如 index 连续） // 跳过和base_link 的碰撞 dummy 的 base link AABB 太大;
        if (std::abs(indexA - indexB) == 1 || indexA == 0 || indexB == 0) {
            continue;
        }

        if (manifold->getNumContacts() > 0) {
            //std::cout << "Collision detected between " << nameA << " and " << nameB << std::endl;
            return true;
        }
    }

    return false;
}

std::string BulletCollisionManager::getObjectName(const btCollisionObject* obj) const {
    for (const auto& o : objects) {
        if (o.collisionObject == obj) return o.name;
    }
    return "Unknown";
}

void BulletCollisionManager::clear() {
    for (auto& obj : objects) {
        collisionWorld->removeCollisionObject(obj.collisionObject);
        delete obj.collisionObject;
        delete obj.shape;
    }
    objects.clear();
}