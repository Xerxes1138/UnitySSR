//Copyright (c) 2015, Charles Greivelding Thomas
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//* Redistributions of source code must retain the above copyright notice, this
//  list of conditions and the following disclaimer.
//
//* Redistributions in binary form must reproduce the above copyright notice,
//  this list of conditions and the following disclaimer in the documentation
//  and/or other materials provided with the distribution.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Shader "Hidden/Unity SSR" 
{
	Properties 
	{
		_MainTex ("Base (RGB)", 2D) = "black" {}
	}
	
	CGINCLUDE
	
	#include "UnityCG.cginc"
	
	#define PI 3.14159265359
	
	uniform sampler2D	_MainTex;
	uniform sampler2D	_Mip; // SSR pass with mip map chain
	uniform sampler2D	_CameraGBufferTexture0; // Diffuse RGB and Occlusion A
	uniform sampler2D	_CameraGBufferTexture1; // Specular RGB and Roughness/Smoothness A
	uniform sampler2D	_CameraGBufferTexture2; // World Normal RGB
	
	uniform sampler2D	_Jitter;
	uniform sampler2D	_Dither;
	uniform sampler2D	_CameraDepthTexture;

	uniform float4	_CameraDepthTexture_ST;
	uniform float4 _MainTex_TexelSize;
	
	uniform float		_edgeFactor; 
	uniform float		_smoothnessRange;

	uniform int			_numSamples;
	uniform int			_numSteps;
	uniform int			_textureSize;

	uniform float4x4	_ProjectionMatrix;
	uniform float4x4	_ViewProjectionInverseMatrix;
	uniform float4x4	_WorldViewMatrix;
	uniform float4x4	_WorldViewInverseMatrix;

	// Schlick 1994, "An Inexpensive BRDF Model for Physically-Based Rendering"
	// Lagarde 2012, "Spherical Gaussian approximation for Blinn-Phong, Phong and Fresnel"
	inline float3 F_Schlick(float3 SpecularColor,float LdotH)
	{
	    return SpecularColor + ( 1.0f - SpecularColor ) * exp2( (-5.55473 * LdotH - 6.98316) * LdotH );
	}
	
	// Schlick 1994, "An Inexpensive BRDF Model for Physically-Based Rendering"
	// Lagarde 2012, "Spherical Gaussian approximation for Blinn-Phong, Phong and Fresnel"
	inline float F_SchlickMono(float SpecularColor,float LdotH)
	{
	    return SpecularColor + ( 1.0f - SpecularColor ) * exp2( (-5.55473 * LdotH - 6.98316) * LdotH );
	}
	
	inline float calcLOD(int cubeSize, float pdf, int NumSamples)
	{
		float lod = (0.5 * log2( (cubeSize*cubeSize)/float(NumSamples) ) + 2.0) - 0.5*log2(pdf); 
		return lod;
	}
	
//	uint ReverseBits32( uint bits )
//	{
//	#if SHADER_API_D3D11
//		return reversebits( bits );
//	#else
//		bits = ( bits << 16) | ( bits >> 16);
//		bits = ( (bits & 0x00ff00ff) << 8 ) | ( (bits & 0xff00ff00) >> 8 );
//		bits = ( (bits & 0x0f0f0f0f) << 4 ) | ( (bits & 0xf0f0f0f0) >> 4 );
//		bits = ( (bits & 0x33333333) << 2 ) | ( (bits & 0xcccccccc) >> 2 );
//		bits = ( (bits & 0x55555555) << 1 ) | ( (bits & 0xaaaaaaaa) >> 1 );
//		return bits;
//	#endif
//	}
//
//	float2 Hammersley( uint Index, uint NumSamples, uint2 Random )
//	{
//		float E1 = frac( (float)Index / NumSamples + float( Random.x & 0xffff ) / (1<<16) );
//		float E2 = float( ReverseBits32(Index) ^ Random.y ) * 2.3283064365386963e-10;
//		return float2( E1, E2 );
//	}

	inline  float RadicalInverse( int n, int base)
	{
		float bits = 0.0f;
		float invBase = 1.0f / base, invBi = invBase;
	 	while (n > 0) 
		{
		  	int d_i = (n % base);
			bits += d_i * invBi;
			n /= base;
			invBi *= invBase;
		}
		return (bits);
	}

	inline float2 Hammersley(int i, int N)
	{
		return float2(float(i) * (1.0/float( N )), RadicalInverse(i,3) );
	}

	inline float4 TangentToWorld(float3 N, float4 H)
	{
		float3 UpVector = abs(N.z) < 0.999 ? float3(0,0,1) : float3(1,0,0);
		float3 T = normalize( cross( UpVector, N ) );
		float3 B = cross( N, T );
				 
		return float4((T * H.x) + (B * H.y) + (N * H.z), H.w);
	}
	
	inline float4 ImportanceSampleBlinn( float2 Xi, float Roughness )
	{
		float m = Roughness*Roughness;
		float m2 = m*m;
		
		float n = 2 / m2 - 2;

		float Phi = 2 * PI * Xi.x;
		float CosTheta = pow( max(Xi.y, 0.001f), 1 / (n + 1) );
		float SinTheta = sqrt( 1 - CosTheta * CosTheta );

		float3 H;
		H.x = SinTheta * cos( Phi );
		H.y = SinTheta * sin( Phi );
		H.z = CosTheta;
		
		float D = (n+2)/ (2*PI) * saturate(pow( CosTheta, n ));
		float pdf = D * CosTheta;

		return float4(H, pdf); 
	}

	// Brian Karis, Epic Games "Real Shading in Unreal Engine 4"
	inline float4 ImportanceSampleGGX( float2 Xi, float Roughness )
	{
		//float m = Roughness;
		float m = Roughness*Roughness;
		float m2 = m*m;
		
		float Phi = 2 * PI * Xi.x;
				 
		float CosTheta = sqrt( (1 - Xi.y) / ( 1 + (m2 - 1) * Xi.y ) );
		float SinTheta = sqrt( 1 - CosTheta * CosTheta );  
				 
		float3 H;
		H.x = SinTheta * cos( Phi );
		H.y = SinTheta * sin( Phi );
		H.z = CosTheta;
		
		float d = ( CosTheta * m2 - CosTheta ) * CosTheta + 1;
		float D = m2 / ( PI*d*d );
		float pdf = D * CosTheta;

		return float4(H, pdf); 
	}
	
	float InterleavedGradientNoise(float2 pos, float2 scale)
	{
		float3 magic = float3(0.06711056, 0.00583715,52.9829189); 
		return -scale + 2.0 * scale * frac(magic.z * frac(dot(pos,magic.xy)));
	}
	
			static float2 kernel[64] = 
			{
				{0.254395, 0.385445},
				{-1.139177, 2.571074},
				{-0.453510, 0.962140},
				{1.482684, 0.346190},
				{0.889741, 0.724799},
				{-1.661047, -0.137344},
				{-2.193311, -0.313475},
				{1.363177, -0.960291},
				{-0.225866, -1.654962},
				{0.407875, 1.431522},
				{-1.543008, -1.992827},
				{-0.878493, -0.896999},
				{-0.641776, 1.130757},
				{-1.785279, -1.015190},
				{1.469054, 1.248633},
				{-1.820410, -0.562833},
				{0.174019, -0.618069},
				{-0.906449, -1.275099},
				{-0.655285, -0.956021},
				{0.453776, -0.085425},
				{-0.430788, -0.089530},
				{-0.587370, -1.851826},
				{1.149056, -2.474322},
				{1.228684, 1.321661},
				{-1.668752, -0.829400},
				{-0.729554, 0.438250},
				{-0.467018, -0.252882},
				{0.090398, 0.618760},
				{0.741840, 0.516401},
				{-0.983891, 0.248701},
				{-1.162010, -0.714455},
				{0.308825, 1.787175},
				{0.254395, 0.385445},
				{-1.139177, 2.571074},
				{-0.453510, 0.962140},
				{1.482684, 0.346190},
				{0.889741, 0.724799},
				{-1.661047, -0.137344},
				{-2.193311, -0.313475},
				{1.363177, -0.960291},
				{-0.225866, -1.654962},
				{0.407875, 1.431522},
				{-1.543008, -1.992827},
				{-0.878493, -0.896999},
				{-0.641776, 1.130757},
				{-1.785279, -1.015190},
				{1.469054, 1.248633},
				{-1.820410, -0.562833},
				{0.174019, -0.618069},
				{-0.906449, -1.275099},
				{-0.655285, -0.956021},
				{0.453776, -0.085425},
				{-0.430788, -0.089530},
				{-0.587370, -1.851826},
				{1.149056, -2.474322},
				{1.228684, 1.321661},
				{-1.668752, -0.829400},
				{-0.729554, 0.438250},
				{-0.467018, -0.252882},
				{0.090398, 0.618760},
				{0.741840, 0.516401},
				{-0.983891, 0.248701},
				{-1.162010, -0.714455},
				{0.308825, 1.787175}
			};
		
	inline float4 screenBRDF(float2 uv, int numSample)
	{
		float4 frag = tex2D(_MainTex,  uv);
		
		half4 gbuffer0 = tex2D (_CameraGBufferTexture0, uv); // Diffuse RGB and Occlusion A
		half4 gbuffer1 = tex2D (_CameraGBufferTexture1, uv); // Specular RGB and Roughness/Smoothness A
		half4 gbuffer2 = tex2D (_CameraGBufferTexture2, uv); // World Normal RGB

		float4 specular = gbuffer1;
		
		float occlusion = gbuffer0.a;

        float3 worldNormal =  gbuffer2 * 2.0 - 1.0;
 		float3 viewNormal =  mul(_WorldViewInverseMatrix, float4(worldNormal.rgb,1));
		
		viewNormal = normalize(viewNormal);
		worldNormal = normalize(worldNormal);
		
		float z = UNITY_SAMPLE_DEPTH(tex2D(_CameraDepthTexture, uv)); // we don't want linear Z here        	
		float3 scrPos = float3(uv*2-1, z);      
		float4 worldPos = mul(_ViewProjectionInverseMatrix, float4(scrPos,1)); // World position reconstruct by depth
		worldPos = worldPos / worldPos.w;
		
		float4 viewPos = mul(_WorldViewMatrix, worldPos); // World position to view position

		float3 viewDir = normalize(worldPos.rgb - _WorldSpaceCameraPos);
		
		float NdotV = saturate(dot( worldNormal ,-viewDir ));
		 
		float roughness = 1-specular.a;

		roughness = min(_smoothnessRange,roughness);
      	
      	float depth = F_SchlickMono(0.4f,NdotV); // Used depth at first but  it appears that using a NdotV gives a much more predictable effect

      	roughness = lerp(roughness,0.0f,depth);

		float4 sampleColor = float4(0,0,0,1);
		
		float2 jitter4x4 = tex2D( _Dither, uv * _ScreenParams.xy / 4 );
		//float2x2 rotationMatrix = float2x2(jitter4x4.x, jitter4x4.y, -jitter4x4.x, jitter4x4.y);
		//float2 jitter64x64 = tex2D( _Jitter, uv * _ScreenParams.xy / 64 );

		//float2 rotation = float2(0,0);
		//sincos(2.0 * PI * InterleavedGradientNoise(uv, 64),rotation.x, rotation.y);
		//float2x2 rotationMatrix = float2x2(rotation.x, rotation.y, -rotation.x, rotation.y);
		
		const int NumSamples = numSample;

		for( int i = 0; i < NumSamples; i++ )
		{	
		
			float2 Xi = float2(0,0);
			
			#ifdef _SAMPLING_HIGH
				Xi = Hammersley(i, NumSamples); // Much more better in quality but also much more slower
			#else
				Xi = jitter4x4; //A bit jittery but much more faster
			#endif

			float4 H = float4(0,0,0,0);
			
			#ifdef _BRDF_GGX
				H = ImportanceSampleGGX(Xi, roughness); // H expect to use a roughness value
			#else
				H = ImportanceSampleBlinn(Xi, roughness); // H expect to use a roughness value
			#endif

			sampleColor.rgb += tex2Dlod(_Mip, float4(uv.xy+H.xy,0,calcLOD(_textureSize,H.w,NumSamples)));
		}
		sampleColor.rgb /= NumSamples;

		return float4(frag.rgb + sampleColor.rgb * F_Schlick(specular.rgb,NdotV) * occlusion * (1-roughness), 1); // We mask the reflection with roughness to prevent too much blur bleeding on rough surfaces
		//return rotation.xyxy;
	}

	struct appdata 
	{
		float4 vertex : POSITION;
		float3 texcoord : TEXCOORD;
	};

	struct v2f 
	{
		float4 pos : POSITION;
		float2 uv : TEXCOORD0; 
	};

	v2f vert( appdata v ) 
	{
		v2f o;
		o.pos = mul(UNITY_MATRIX_MVP, v.vertex);
		o.uv = v.texcoord;
		return o;
	}

	float4 RayMarch(float3 R,int NumSteps, float4 viewPos, float3 screenPos, float2 coord, float3 stepOffset)
	{

		float4 rayPos = viewPos + float4(R,1);
		float4 rayUV = mul (_ProjectionMatrix, rayPos);
		rayUV.xyz /= rayUV.w;
					
		float3 rayDir = normalize( rayUV - screenPos );
		rayDir.xy *= 0.5;

		float sampleDepth;
		float sampleMask = 0;

	    float3 rayStart = float3(coord,screenPos.z);
                    
 		float stepSize = 1 / ( (float)NumSteps + 1);
		rayDir  *= stepOffset * stepSize + stepSize;
                                  
		float3 samplePos = rayStart + rayDir;

		for (int steps = 1;  steps< NumSteps; ++steps)
		{
			sampleDepth  = LinearEyeDepth(UNITY_SAMPLE_DEPTH(tex2Dlod (_CameraDepthTexture, float4(samplePos.xy,0,0))));
 
			if ( sampleDepth < LinearEyeDepth(samplePos.z) )  
			{  
			
				if (abs(sampleDepth - LinearEyeDepth(samplePos.z) ) < 0.3f)
				{
	                sampleMask = 1;
	                steps = NumSteps+8;
	                break;
				}
				else
				{
					rayDir *= 0.5;
					samplePos = rayStart + rayDir; 
				} 	                 
			}
			else
			{
		        rayStart = samplePos;
		        rayDir *= 1.1;
		        samplePos += rayDir;
			}
		}
		
		return float4(samplePos, sampleMask);

	}

	float4 fragSSR (v2f i) : SV_Target
	{	
		float2 uv = i.uv;
		
		float4 gbuffer2 = tex2D (_CameraGBufferTexture2, uv); // World Normal RGB
                   
		float3 worldNormal =  gbuffer2 * 2.0 - 1.0;
		float3 viewNormal =  mul(_WorldViewInverseMatrix, float4(worldNormal.rgb,1));
		
		viewNormal = normalize(viewNormal);
		
		float z = UNITY_SAMPLE_DEPTH(tex2D(_CameraDepthTexture, uv)); // we don't want linear Z here
                  	
		float3 scrPos = float3(uv*2-1, z);
                    
		float4 worldPos = mul(_ViewProjectionInverseMatrix, float4(scrPos,1)); // World position reconstruct by depth
		worldPos = worldPos / worldPos.w;
			     	
		float4 viewPos = mul(_WorldViewMatrix, worldPos); // World position to view position
 	
		float3 viewDir = normalize(worldPos.rgb-_WorldSpaceCameraPos);

		float4 sampleColor = float4(0,0,0,1);

		int NumSteps = _numSteps;
		
		float3 stepOffset = tex2D( _Dither, uv * _ScreenParams.xy / 4 );
		float3 R = reflect(viewPos,viewNormal);
		float4 ray = RayMarch(R, NumSteps, viewPos, scrPos, uv, stepOffset.x);
				
		float borderDist = min(1-max(ray.x, ray.y), min(ray.x, ray.y));
		float borderAtten = saturate(borderDist > _edgeFactor ? 1 : borderDist / _edgeFactor);

		sampleColor.rgb = tex2D(_MainTex, ray.xy);

		return  float4(sampleColor * ray.w * borderAtten);
	}
	
	float4 frag( v2f i ) : SV_Target
	{	 
		float2 uv = i.uv;
		float4 frag = tex2D(_MainTex, uv);

		return frag;
	}
	
	
	float4 fragScreenBRDFLow( v2f i ) : SV_Target
	{	 
		return screenBRDF(i.uv, 16);
	}
	
	float4 fragScreenBRDFMedium( v2f i ) : SV_Target
	{	 
		return screenBRDF(i.uv, 32);
	}
	
	float4 fragScreenBRDFHigh( v2f i ) : SV_Target
	{	 
		return screenBRDF(i.uv, 64);
	}

	ENDCG 
	
	Subshader 
	{
		//0
		Pass 
		{
			ZTest Always Cull Off ZWrite Off
			Fog { Mode off }

			CGPROGRAM
			#pragma target 3.0
			
			#ifdef SHADER_API_OPENGL
       			#pragma glsl
    		#endif

			#pragma fragmentoption ARB_precision_hint_fastest
			#pragma vertex vert
			#pragma fragment frag
			ENDCG
		}
		//1
		Pass 
		{
			ZTest Always Cull Off ZWrite Off
			Fog { Mode off }

			CGPROGRAM
			#pragma target 3.0
			
			#ifdef SHADER_API_OPENGL
       			#pragma glsl
    		#endif

			#pragma fragmentoption ARB_precision_hint_fastest
			#pragma vertex vert
			#pragma fragment fragSSR
			ENDCG
		}
		//2
		Pass 
		{
			ZTest Always Cull Off ZWrite Off
			Fog { Mode off }

			CGPROGRAM
			#pragma target 3.0
			
			#ifdef SHADER_API_OPENGL
       			#pragma glsl
    		#endif
    		
    		#pragma multi_compile _ _SAMPLING_HIGH
    		#pragma multi_compile _ _BRDF_GGX

			#pragma fragmentoption ARB_precision_hint_fastest
			#pragma vertex vert
			#pragma fragment fragScreenBRDFLow
			ENDCG
		}
		//3
		Pass 
		{
			ZTest Always Cull Off ZWrite Off
			Fog { Mode off }

			CGPROGRAM
			#pragma target 3.0
			
			#ifdef SHADER_API_OPENGL
       			#pragma glsl
    		#endif
	
    		#pragma multi_compile _ _SAMPLING_HIGH
    		#pragma multi_compile _ _BRDF_GGX
			
			#pragma fragmentoption ARB_precision_hint_fastest
			#pragma vertex vert
			#pragma fragment fragScreenBRDFMedium
			ENDCG
		}
		//4
		Pass 
		{
			ZTest Always Cull Off ZWrite Off
			Fog { Mode off }

			CGPROGRAM
			#pragma target 3.0
			
			#ifdef SHADER_API_OPENGL
       			#pragma glsl
    		#endif
    		
    		#pragma multi_compile _ _SAMPLING_HIGH
    		#pragma multi_compile _ _BRDF_GGX

			#pragma fragmentoption ARB_precision_hint_fastest
			#pragma vertex vert
			#pragma fragment fragScreenBRDFHigh
			ENDCG
		}
	}
	Fallback Off
}
