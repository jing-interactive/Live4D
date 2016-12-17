#version 150 core

uniform mat4		ciModelViewProjection;
uniform usampler2D	uTextureDepth;
uniform sampler2D	uTextureDepthToCameraTable;
uniform float uMinDistance;
uniform float uMaxDistance;
uniform bool		uFlipX;
uniform bool		uFlipY;

in vec2				ciPosition;

out float			vDepth;
out vec2			vTexCoord0;

void main( void )
{
	vTexCoord0	= ciPosition;

	vec2 uv		= vTexCoord0;
	uv.x		= uFlipX ? 1.0 - uv.x : uv.x;
	uv.y		= uFlipY ? 1.0 - uv.y : uv.y;

	vDepth		= texture( uTextureDepth, uv ).r;
	vec3 pos	= vec3( texture( uTextureDepthToCameraTable, vTexCoord0 ).rg * vDepth, vDepth );

	gl_Position = ciModelViewProjection * vec4( pos * 0.1, 1.0 );

	if ( vDepth <= uMinDistance || vDepth >= uMaxDistance )
		gl_Position.w = 0;

	vTexCoord0 = uv;
};
 