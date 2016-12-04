//The MIT License(MIT)

//Copyright(c) 2016 Charles Greivelding Thomas

//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

#define PI 3.14159265359

uniform sampler2D	_MainTex,
					_ReflectionBuffer,
					_MipMapBuffer,
					_RayCast;

uniform sampler2D	_Noise;

uniform sampler2D	_CameraGBufferTexture0,
					_CameraGBufferTexture1,
					_CameraGBufferTexture2,
					_CameraReflectionsTexture;

uniform sampler2D	_CameraDepthTexture;
uniform sampler2D	_CameraDepthBuffer;

uniform sampler2D_half _CameraMotionVectorsTexture;

uniform float4		_MainTex_TexelSize;
uniform float4		_BufferSize;
uniform float4		_Project;
uniform float4		_GaussianDir;
uniform float4		_RayCastSize;
uniform float		_EdgeFactor; 
uniform float		_SmoothnessRange;

uniform int			_NumSteps;
uniform int			_MipMapCount;

uniform float4x4	_ProjectionMatrix;
uniform float4x4	_ViewProjectionMatrix;
uniform float4x4	_InverseProjectionMatrix;
uniform float4x4	_InverseViewProjectionMatrix;
uniform float4x4	_WorldToCameraMatrix;
uniform float4x4	_CameraToWorldMatrix;

//Debug Options
uniform int			_UseTemporal;

float sqr(float x)
{
	return x*x;
}
	
float fract(float x)
{
	return x - floor( x );
}

float4	GetSampleColor (sampler2D tex, float2 uv) { return tex2D(tex, uv); }
float4	GetCubeMap (float2 uv) { return tex2D(_CameraReflectionsTexture, uv); }
float4	GetAlbedo (float2 uv) { return tex2D(_CameraGBufferTexture0, uv); }
float4	GetSpecular (float2 uv) { return tex2D(_CameraGBufferTexture1, uv); }
float	GetRoughness (float smoothness) { return max(min(_SmoothnessRange, 1.0 - smoothness), 0.05); }
float	GetSmoothness (float smoothness) { return max(max(1 - _SmoothnessRange, smoothness), 0.05); }
float4	GetNormal (float2 uv) { float4 gbuffer2 = tex2D(_CameraGBufferTexture2, uv); return gbuffer2 * 2.0 - 1.0; }
float4	GetReflection(float2 uv)    { return tex2D(_ReflectionBuffer, uv); }
float4	GetVelocity(float2 uv)    { return tex2D(_CameraMotionVectorsTexture, uv); }

float3	GetViewNormal (float3 normal)
{
	float3 viewNormal =  mul((float3x3)_WorldToCameraMatrix, normal.rgb);
	return normalize(viewNormal);
}

float GetDepth (sampler2D tex, float2 uv)
{
	return UNITY_SAMPLE_DEPTH (tex2Dlod(tex, float4(uv, 0, 0)));
}

float GetDepth (sampler2D tex, float2 uv, float mip)
{
	return UNITY_SAMPLE_DEPTH(tex2Dlod(tex, float4(uv, 0, mip)));
}

float3 GetScreenPos (float2 uv, float depth)
{
	return float3(uv.x * 2 - 1, uv.y * 2 - 1, depth);
}

float3 GetWorlPos (float3 screenPos)
{
	float4 worldPos = mul(_InverseViewProjectionMatrix, float4(screenPos, 1));
	return worldPos.xyz / worldPos.w;
}

float3 GetViewPos (float3 screenPos)
{
	float4 viewPos = mul(_InverseProjectionMatrix, float4(screenPos, 1));
	return viewPos.xyz / viewPos.w;
}
	
float3 GetViewDir (float3 worldPos)
{
	return normalize(worldPos - _WorldSpaceCameraPos);
}

float RayAttenBorder (float2 pos, float value)
{
	float borderDist = min(1.0 - max(pos.x, pos.y), min(pos.x, pos.y));
	return saturate(borderDist > value ? 1.0 : borderDist / value);
}

float calcLOD(int cubeSize, float pdf)
{
	float lod = (0.5 * log2( (cubeSize*cubeSize) / cubeSize ) + 2.0) - 0.5*log2(pdf); 
	return lod;
}

float4 TangentToWorld(float3 N, float4 H)
{
	float3 UpVector = abs(N.z) < 0.999 ? float3(0,0,1) : float3(1,0,0);
	float3 T = normalize( cross( UpVector, N ) );
	float3 B = cross( N, T );
				 
	return float4((T * H.x) + (B * H.y) + (N * H.z), H.w);
}
	
static const int2 offsets[7] = {{-3, -3}, {-2, -2}, {-1, -1}, {0, 0}, {1, 1}, {2, 2}, {3, 3}};

static const float weights[7] = {0.001f, 0.028f, 0.233f, 0.474f, 0.233f, 0.028f, 0.001f};