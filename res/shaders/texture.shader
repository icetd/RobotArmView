#shader vertex
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 cColor;
layout (location = 2) in vec3 aNormal;
layout (location = 3) in vec2 aTexCoords;

out vec2 TexCoords;


uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{	    
    TexCoords = aTexCoords;
	gl_Position = projection * view * model * vec4(aPos, 1.0);
}

#shader fragment
#version 330 core

in vec2 TexCoords;
uniform sampler2D texture_diffuse1;

out vec4 FragColor;

void main()
{
    FragColor = texture(texture_diffuse1, TexCoords);
}