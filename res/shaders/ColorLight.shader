#shader vertex
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
layout (location = 2) in vec3 aNormal;
layout (location = 3) in vec2 aTexCoords;

out vec3 FragPos;
out vec3 Color;
out vec3 Normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{	
	FragPos = aPos;
	Color = aColor;   
	Normal = aNormal;
	gl_Position = projection * view * model * vec4(aPos, 1.0);
}

#shader fragment
#version 330 core


in vec3 FragPos;
in vec3 Color;
in vec3 Normal;

uniform vec3 lightColor;
uniform vec3 lightPos;
uniform vec3 camPos;

out vec4 FragColor;

void main()
{
    //ambient
	float ambientStrength = 0.02f;
	vec3 ambient = ambientStrength * lightColor;


    // diffuse
	float diffStrength = 0.8f;
	vec3 norm = normalize(Normal);
	vec3 lightDir = normalize(lightPos - FragPos);
	float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse =  diffStrength * lightColor * diff * Color;

	// specular
	float specularStrength = 0.2;
	vec3 viewDir = normalize(camPos - FragPos);
	vec3 reflectDir = reflect(-lightDir, Normal);
	float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
	vec3 specular = specularStrength * lightColor * spec * Color;

	vec3 result = ambient + diffuse + specular;
	FragColor = vec4(result, 1.0);
}