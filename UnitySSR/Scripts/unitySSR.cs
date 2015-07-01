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

using System;
using UnityEngine;

namespace unitySSR
{
	[ExecuteInEditMode]
	[RequireComponent(typeof (Camera))]
	[AddComponentMenu("Image Effects/Rendering/Unity 5 Screen Space Reflection")]

	public class unitySSR : MonoBehaviour
	{

		public bool useSSR = true;
		public  int numSteps = 128;

		public float reflectionEdgeFactor = 0.25f;

		public float smoothnessRange = 1.0f;

		public enum TextureSize
		{
			_128 = 128,
			_256 = 256,
			_512 = 512,
			_1024 = 1024,
			_2048 = 2048,
		};

		public TextureSize textureSize = TextureSize._512;

		public enum PointModel
		{
			Hammersley,
			Noise,
		}

		public PointModel pointModel = PointModel.Noise;

		public enum BRDFModel
		{
			Blinn,
			GGX,
		};

		public BRDFModel brdfModel = BRDFModel.Blinn;

		public enum SampleQuality  
		{
			Low = 2,
			Medium = 3,
			High = 4,
		};

		public SampleQuality sampleQuality = SampleQuality.Medium; // good value is 32/Medium but 64/High may gives better results but with perf impact

		private Texture jitter;
		private Texture dither;

		static Material m_rendererMaterial = null;
		protected Material rendererMaterial
		{
			get 
			{
				if (m_rendererMaterial == null) 
				{
					m_rendererMaterial = new Material(Shader.Find("Hidden/Unity SSR"));
					m_rendererMaterial.hideFlags = HideFlags.DontSave;
				}
				return m_rendererMaterial;
			} 
		}

		void goVariable()
		{
			jitter = Resources.Load("NOISE_64X64_JITTER",typeof(Texture)) as Texture;
			rendererMaterial.SetTexture("_Jitter",jitter);
			dither = Resources.Load("NOISE_4X4_JITTER",typeof(Texture)) as Texture;
			rendererMaterial.SetTexture("_Dither",dither);

			rendererMaterial.SetFloat ("_smoothnessRange", smoothnessRange);
			rendererMaterial.SetFloat("_edgeFactor",reflectionEdgeFactor);

			rendererMaterial.SetInt("_numSteps",numSteps);

			rendererMaterial.SetInt("_textureSize", (int)textureSize);
		}

		void Awake ()
		{
			InitCamera();
		}
		
		void InitCamera()
		{
			GetComponent<Camera>().depthTextureMode = DepthTextureMode.Depth;	
		}

		public void OnRenderImage(RenderTexture source, RenderTexture destination) 
		{

			if(useSSR == true )
			{
				goMatrix();
				goVariable();

				RenderTexture mip = RenderTexture.GetTemporary((int)textureSize ,(int)textureSize ,16,RenderTextureFormat.ARGBHalf); // Using a square texture to get mip map as Unity can't generate mip map on a non squared texture
				mip.isPowerOfTwo = true;
				mip.useMipMap = true;
				mip.generateMips = true;
				mip.filterMode = FilterMode.Trilinear;

				Graphics.Blit (source,mip, rendererMaterial,1); // Ray marching pass
						
				rendererMaterial.SetTexture ("_Mip", mip); // result of the ray marching pass is stored in the mip render texture

				if(brdfModel == BRDFModel.GGX)
					rendererMaterial.EnableKeyword("_BRDF_GGX");
				if(brdfModel == BRDFModel.Blinn)
					rendererMaterial.DisableKeyword("_BRDF_GGX");

				if(pointModel == PointModel.Hammersley)
					rendererMaterial.EnableKeyword("_SAMPLING_HIGH");
				if(pointModel == PointModel.Noise)
					rendererMaterial.DisableKeyword("_SAMPLING_HIGH");


				Graphics.Blit (source,destination, rendererMaterial,(int)sampleQuality); // low = pass 2; medium = pass 3, high = pass 4

				RenderTexture.ReleaseTemporary(mip);
			}
			else
			{
				Graphics.Blit (source,destination, rendererMaterial,0);
			}

			RenderTexture.active = null;
		}

		void goMatrix() 
		{
			rendererMaterial.SetMatrix("_WorldViewMatrix", GetComponent<Camera>().worldToCameraMatrix);
			rendererMaterial.SetMatrix("_WorldViewInverseMatrix", GetComponent<Camera>().worldToCameraMatrix.inverse.transpose);
			rendererMaterial.SetMatrix("_ViewProjectionInverseMatrix", (GetComponent<Camera>().projectionMatrix*GetComponent<Camera>().worldToCameraMatrix).inverse );
			Matrix4x4 projectionMatrix = GetComponent<Camera>().projectionMatrix;
			bool d3d = SystemInfo.graphicsDeviceVersion.IndexOf("Direct3D") > -1;
			if (d3d) 
			{
				// Scale and bias from OpenGL -> D3D depth range
				for (int i = 0; i < 4; i++) 
				{
					projectionMatrix[2,i] = projectionMatrix[2,i]*0.5f + projectionMatrix[3,i]*0.5f;
				}
			}
			rendererMaterial.SetMatrix("_ProjectionMatrix", projectionMatrix);
		}
	}
}
