#version 330 core
out vec4 FragColor;  // 输出片段颜色
in vec2 TexCoords;   //对应纹理坐标
in vec3 WorldPos;   //输入顶点位置
in vec3 Normal;   //输入顶点法向量

// 金属性
uniform sampler2D albedoMap;
uniform float metallic;
uniform float roughness;
uniform float ao;

// 光线位置
uniform vec3 lightPositions[4];
uniform vec3 lightColors[4];

uniform vec3 camPos;

const float PI = 3.14159265359;
// 正态分布函数s----------------------------------------------------------------------------
float DistributionGGX(vec3 N, vec3 H, float roughness)
{
    float a = roughness*roughness;
    float a2 = a*a;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH*NdotH;

    float nom   = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom = PI * denom * denom;

    return nom / max(denom, 0.001); //防止被除数是0
}
// 几何遮蔽函数----------------------------------------------------------------------------
float GeometrySchlickGGX(float NdotV, float roughness)
{
    float r = (roughness + 1.0);
    float k = (r*r) / 8.0;

    float nom   = NdotV;
    float denom = NdotV * (1.0 - k) + k;

    return nom / denom;
}
// ----------------------------------------------------------------------------
float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float ggx2 = GeometrySchlickGGX(NdotV, roughness);
    float ggx1 = GeometrySchlickGGX(NdotL, roughness);

    return ggx1 * ggx2;
}
// ----------------------------------------------------------------------------
vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}
// ----------------------------------------------------------------------------
void main()
{	
   //内建的texture函数来采样纹理的颜色
    vec3 albedo     = pow(texture(albedoMap, TexCoords).rgb, vec3(2.2));	
    vec3 N = normalize(Normal);
    vec3 V = normalize(camPos - WorldPos);


   //计算垂直入射时的反射率;塑料是0.04，金属是albedo
    vec3 F0 = vec3(0.04); 
    F0 = mix(F0, albedo , metallic);

    // reflectance equation
    vec3 Lo = vec3(0.0);
    for(int i = 0; i < 4; ++i) 
    {
        //计算光辐射入射出射
        vec3 L = normalize(lightPositions[i] - WorldPos);
        vec3 H = normalize(V + L);
        float distance = length(lightPositions[i] - WorldPos);
        float attenuation = 1.0 / (distance * distance);
        vec3 radiance = lightColors[i] * attenuation;

        // Cook-Torrance BRDF
        float NDF = DistributionGGX(N, H, roughness);   
        float G   = GeometrySmith(N, V, L, roughness);      
        vec3 F    = fresnelSchlick(clamp(dot(H, V), 0.0, 1.0), F0);
           
        vec3 nominator    = NDF * G * F; 
        float denominator = 4 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0);
        vec3 specular = nominator / max(denominator, 0.001); // prevent divide by zero for NdotV=0.0 or NdotL=0.0
        
        // kS  Fresnel菲涅耳
        vec3 kS = F;
        // 漫射光和反射光不能大于1.0(除非表面发光)，关系扩散分量(kD)应该等于1.0 - kS。
        vec3 kD = vec3(1.0) - kS;
        // 只有非金属才有漫射光，或者部分金属(纯金属没有漫射光)的线性混合。
        kD *= 1.0 - metallic;	  

        //标度灯by NdotL
        float NdotL = max(dot(N, L), 0.0);        

        // 出射光亮度 Lo
        Lo += (kD * albedo / PI + specular) * radiance * NdotL;  // note that we already multiplied the BRDF by the Fresnel (kS) so we won't multiply by kS again
    }   
    
    // 环境照明
    vec3 ambient = vec3(0.03) * albedo * ao;

    vec3 color = ambient + Lo;

    // HDR 色调映射
    color = color / (color + vec3(1.0));
    //伽马修正
    color = pow(color, vec3(1.0/2.2)); 

    FragColor = vec4(color, 1.0);
}