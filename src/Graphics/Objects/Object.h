#pragma once
#include <glm/glm.hpp>
#include <iostream>

#define M_PI    3.141592653589793
#define RADIANS_TO_ANGLE    180 / M_PI

struct ObjectStructure
{
    const glm::vec3 blue = glm::vec3(0.1, 0.1, 0.7);
    const glm::vec3 green = glm::vec3(0.0, 0.8, 0.0);
    const glm::vec3 grey = glm::vec3(0.6, 0.6, 0.6);
    const glm::vec3 lightGrey = glm::vec3(0.9, 0.9, 0.9);
    const glm::vec3 darkGrey = glm::vec3(0.2, 0.2, 0.2);
    const glm::vec3 red = glm::vec3(0.8, 0.0, 0.0);
    const glm::vec3 white = glm::vec3(1.0, 1.0, 1.0);
    const glm::vec3 orange = glm::vec3(1.0, 0.5, 0.0);
    const glm::vec3 brown = glm::vec3(0.9, 0.9, 0.85);
    const glm::vec3 black = glm::vec3(0.0, 0.0, 0.0);
    const glm::vec3 yellow = glm::vec3(0.0, 0.9, 0.9);
    const glm::vec3 silver = glm::vec3(0.8, 0.8, 0.8);

    bool modelDefined = false;
	std::string path;
	std::string name;
	glm::vec3 modelColor = glm::vec3(0.5f, 0.5f, 0.5f);
	glm::vec3 modelLineColor = glm::vec3(0.2f, 0.2f, 0.2f);

	glm::mat4 objModel = glm::mat4(1.0f);       // not ueed same to joint_transmat
	glm::mat3 objRotation = glm::mat3(0.0f);    // update in updateJointAngles
	glm::vec3 objTranslation = glm::vec3(0.0f); // update in updateJointAngles

    int joint_type;
    glm::mat4 joint_transmat; // 机械臂 model
    glm::mat4 vis_transmat;
    glm::vec3 scale;  // 机械臂缩放
    glm::vec3 axis;   // 当前关机旋转轴

    typedef struct {
        float offort_angle;
        float lower_angle;
        float upper_angle;
        float velocity_angle;
    } LimitsAngle_t;

    LimitsAngle_t limitAngle;
    float curAngle;
    
    ObjectStructure *parent;

    typedef enum {
        UNKNOWN,
        REVOLUTE,
        CONTINUOUS,
        PRISMATIC,
        FLOATING,
        PLANAR,
        FIXED
    } JointType;

    void setColor(std::string material)
    {
        std::string lowerMaterial = material;
        std::transform(lowerMaterial.begin(), lowerMaterial.end(), lowerMaterial.begin(), ::tolower);
        
        if (lowerMaterial == "blue") {
            this->modelColor = blue;
        } else if (lowerMaterial == "green") {
            this->modelColor = green;
        } else if (lowerMaterial == "grey") {
            this->modelColor = grey;
        } else if (lowerMaterial == "lightgrey") {
            this->modelColor = lightGrey;
        } else if (lowerMaterial == "darkgrey") {
            this->modelColor = darkGrey;
        } else if (lowerMaterial == "red") {
            this->modelColor = red;
        } else if (lowerMaterial == "white") {
            this->modelColor = white;
        } else if (lowerMaterial == "orange") {
            this->modelColor = orange;
        } else if (lowerMaterial == "brown") {
            this->modelColor = brown;
        } else if (lowerMaterial == "yellow") {
            this->modelColor = yellow;
        } else if (lowerMaterial == "silver") {
            this->modelColor = silver;
        } else {
            this->modelColor = white;
        }
    }

    void setAngle(float angle) {
        curAngle = angle;
        // objTranslation = glm::vec3(this->joint_transmat[3]); // already update in updateJointAngles
        glm::mat4 translationMat = glm::translate(glm::mat4(1.0f), objTranslation); // 创建平移矩阵
        glm::mat4 rotationMat = glm::rotate(glm::mat4(1.0f), glm::radians(angle), axis);
        this->joint_transmat = translationMat * rotationMat;
    }

    float getAngle() { return curAngle;}
};