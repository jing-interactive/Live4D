#version 150

in vec4 ciPosition;
in vec2 ciTexCoord0;

uniform usampler2D  uDepthTexture;
uniform mat4        ciModelViewProjection;
out     float       vDepthColor;
uniform float		uDepthToMmScale;

/// convert raw shift value to metric depth (in mm)
float raw_to_mm(float raw)
{
    return uDepthToMmScale * raw;
}

///depth to world conversion
vec3 depth_mm_to_world( float cam_x, float cam_y, float depth_mm ){
    
    vec3  world;
    float factor    = 2. * depth_mm;
    world.x         = ( cam_x - 320. ) * factor;
    world.y         = ( cam_y - 240. ) * factor;
    world.z         = depth_mm;
    return          world;
    
}

void main()
{
    vec4 pos		= ciPosition;
    uint rawdepth	= texture( uDepthTexture, ciTexCoord0 ).r;
    vDepthColor     = float(rawdepth) / 65535.; //divide by max short bytes for % grey
    pos.xyz         = depth_mm_to_world( ciTexCoord0.x*640., (1.-ciTexCoord0.y)*480., raw_to_mm( float(rawdepth) ) );
	gl_Position		= ciModelViewProjection * pos;
}
