#shader vertex
#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
layout (location = 2) in vec3 aNormal;
layout (location = 3) in vec2 aTexCoords;

out vec3 Color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    Color = aColor;
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}

#shader fragment
#version 330 core

in vec3 Color;
out vec4 FragColor;

void main()
{
    float patternLength = 20.0; // 虚线周期长度，单位是屏幕像素
    float coordSum = gl_FragCoord.x + gl_FragCoord.y;

    if (mod(coordSum, patternLength) > patternLength * 0.5)
        discard;  // 丢弃后半段像素，实现50%实线50%空白虚线

    FragColor = vec4(Color, 1.0);
}