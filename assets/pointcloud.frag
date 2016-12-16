#version 150 core

uniform sampler2D	uTextureColor;
uniform sampler2D	uTextureDepthToColorTable;
uniform float uMinDistance;
uniform float uMaxDistance;

in float			vDepth;
in vec2				vTexCoord0;

out vec4			gl_FragColor;

void main( void )
{
	if ( vDepth <= uMinDistance || vDepth >= uMaxDistance ) {
		discard;
	}
	vec2 uv			= texture( uTextureDepthToColorTable, vTexCoord0 ).rg;

	gl_FragColor	= texture( uTextureColor, uv );
}
 