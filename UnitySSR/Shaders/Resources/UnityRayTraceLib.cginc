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

float4 RayMarch(sampler2D tex, float4x4 _ProjectionMatrix, float3 viewDir, int NumSteps, float3 viewPos, float3 screenPos, float2 uv, float stepOffset)
{
		float4 rayProj = mul (_ProjectionMatrix, float4(viewDir + viewPos,1));
		
		float3 rayDir = normalize( rayProj.xyz / rayProj.w - screenPos );
		rayDir.xy *= 0.5;

	    float3 rayStart = float3(uv, screenPos.z);
                    
 		float stepSize = 1.0 / ( (float)NumSteps + 1.0);
		rayDir  *= stepOffset * stepSize + stepSize;   
		                   
		float3 samplePos = rayStart + rayDir;

		float sampleMask = 0;
		for (int i = 0;  i < NumSteps; i++)
		{
			float sampleDepth  = UNITY_SAMPLE_DEPTH(tex2Dlod (tex, float4(samplePos.xy,0,0)));

			//float delta = abs(sampleDepth - samplePos.z); 

			if (sampleDepth < samplePos.z)  
			{  
				sampleMask = 1;// TO FIX !!!
				break;
			}
			else
				samplePos += rayDir;
		}
		
return float4(samplePos, sampleMask);
}