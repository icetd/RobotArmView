#shader vertex
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
layout (location = 2) in vec3 aNormal;
layout (location = 3) in vec2 aTexCoords;

out vec3 Position;
out vec3 Normal;
out vec2 TexCoords;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{	
    Position = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
	TexCoords = aTexCoords;   
	gl_Position = projection * view * model * vec4(aPos, 1.0);
}

#shader fragment
#version 330 core



in vec3 Position;
in vec3 Color;
in vec3 Normal;
in vec2 TexCoords;

uniform vec3 lightColor;
uniform vec3 lightPos;
uniform vec3 camPos;

uniform sampler2D texture_diffuse1;
uniform sampler2D texture_specular1;
//uniform sampler2D texture_normal1;

out vec4 FragColor;

void main()
{
    //ambient
	float ambientStrength = 0.2f;
	vec3 ambient = ambientStrength * lightColor;


    // diffuse
	float diffStrength = 0.8f;
	vec3 norm = normalize(Normal);
	vec3 lightDir = normalize(lightPos - Position);
	float diff = max(dot(norm, lightDir), 0.5);
    vec3 diffuse =  diffStrength * lightColor * diff * texture(texture_diffuse1, TexCoords).rgb;

	// specular
	float specularStrength = 1.0;
	vec3 viewDir = normalize(camPos - Position);
	vec3 reflectDir = reflect(-lightDir, Normal);
	float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
	vec3 specular = specularStrength * lightColor * spec * texture(texture_specular1, TexCoords).rgb;

	vec3 result = ambient + diffuse + specular;
	FragColor = vec4(result, 1.0);
}