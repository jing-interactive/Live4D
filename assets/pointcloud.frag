#version 150 core

uniform sampler2D	uTextureColor;
uniform sampler2D	uTextureDepthToColorTable;

in float			vDepth;
in vec2				vTexCoord0;

out vec4			gl_FragColor;

void main( void )
{
#if 0
	if ( vDepth <= uMinDistance || vDepth >= uMaxDistance ) {
		discard;
	}
#endif

	vec2 uv			= texture( uTextureDepthToColorTable, vTexCoord0 ).rg;

	// TODO: discard invalid uv

	gl_FragColor	= texture( uTextureColor, uv );
}
 