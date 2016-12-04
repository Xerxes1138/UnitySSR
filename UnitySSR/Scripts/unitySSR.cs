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

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityStandardAssets.CinematicEffects;

namespace cCharkes
{
    [RequireComponent(typeof (Camera))]
	[AddComponentMenu("cCharkes/Image Effects/Rendering/Unity 5 Screen Space Reflection")]
	public class UnitySSR : MonoBehaviour
	{

        [SerializeField]
        bool animateJitter = false;

        [Range(20, 80)]
        [SerializeField]
		int numSteps = 80;

        [Range(0.00001f, 1.0f)]
        [SerializeField]
        float reflectionEdgeFactor = 0.1f;

        [Range(0.0f, 1.0f)]
        [SerializeField]
        float smoothnessRange = 1.0f;

        [SerializeField]
        bool useMipMap = true;

        Camera m_camera;

        RenderTexture mainBuffer0, mainBuffer1;
        RenderTexture mipMapBuffer0, mipMapBuffer1, mipMapBuffer2;

        private Matrix4x4 projectionMatrix;
        private Matrix4x4 viewProjectionMatrix;
        private Matrix4x4 inverseViewProjectionMatrix;
        private Matrix4x4 worldToCameraMatrix;
        private Matrix4x4 cameraToWorldMatrix;


        int maxMipMap = 5;
        Texture noise;

        float[] dist = new float[5] { 1.0f / 1024.0f, 1.0f / 256.0f, 1.0f / 128.0f, 1.0f / 64.0f, 1.0f / 32.0f };

        int[] mipLevel = new int[5] { 0, 2, 3, 4, 5 };

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

        public static RenderTexture CreateRenderTexture(int w, int h, int d, RenderTextureFormat f, bool useMipMap, bool generateMipMap, FilterMode filterMode)
        {
            RenderTexture r = new RenderTexture(w, h, d, f);
            r.filterMode = filterMode;
            r.wrapMode = TextureWrapMode.Clamp;
            r.useMipMap = useMipMap;
            r.autoGenerateMips = generateMipMap;
            r.Create();
            return r;
        }

        RenderTexture CreateTempBuffer(int x, int y, int depth, RenderTextureFormat format)
        {
            return RenderTexture.GetTemporary(x, y, depth, format);
        }

        void ReleaseTempBuffer(RenderTexture rt)
        {
            RenderTexture.ReleaseTemporary(rt);
        }

		void OnEnable ()
        {
            noise = Resources.Load("tex_BlueNoise_256x256_UNI") as Texture2D;


            InitCamera();
		}
		
		void InitCamera()
		{
            m_camera = GetComponent<Camera>();
            m_camera.depthTextureMode |= DepthTextureMode.Depth | DepthTextureMode.MotionVectors;
        }

        void OnDestroy()
        {
            Object.DestroyImmediate(rendererMaterial);
        }

        void OnDisable()
        {
            ReleaseRenderTargets();
        }

        void ReleaseRenderTargets()
        {
            if (mainBuffer0 != null || mainBuffer1 != null)
            {
                mainBuffer0.Release();
                mainBuffer0 = null;
                mainBuffer1.Release();
                mainBuffer1 = null;
            }

            if (mipMapBuffer0 != null || mipMapBuffer1 != null || mipMapBuffer2 != null)
            {
                mipMapBuffer0.Release();
                mipMapBuffer0 = null;
                mipMapBuffer1.Release();
                mipMapBuffer1 = null;
                mipMapBuffer2.Release();
                mipMapBuffer2 = null;
            }
        }

        void UpdateRenderTargets(int w, int h)
        {
            if (mainBuffer0 == null || !mainBuffer0.IsCreated())
            {
                mainBuffer0 = CreateRenderTexture(w, h, 0, RenderTextureFormat.DefaultHDR, false, false, FilterMode.Bilinear);
                mainBuffer1 = CreateRenderTexture(w, h, 0, RenderTextureFormat.DefaultHDR, false, false, FilterMode.Bilinear);
            }

            if (mipMapBuffer0 == null || !mipMapBuffer0.IsCreated() || mipMapBuffer1 == null || !mipMapBuffer1.IsCreated() || mipMapBuffer2 == null || !mipMapBuffer2.IsCreated())
            {
                mipMapBuffer0 = CreateRenderTexture(1024, 1024, 0, RenderTextureFormat.DefaultHDR, true, true, FilterMode.Bilinear);
                mipMapBuffer1 = CreateRenderTexture(1024, 1024, 0, RenderTextureFormat.DefaultHDR, true, true, FilterMode.Bilinear);
                mipMapBuffer2 = CreateRenderTexture(1024, 1024, 0, RenderTextureFormat.DefaultHDR, true, false, FilterMode.Bilinear);
            }
        }

        [ImageEffectOpaque]
        void OnRenderImage(RenderTexture source, RenderTexture destination)
        {

            rendererMaterial.SetPass(0);

            int width = m_camera.pixelWidth;
            int height = m_camera.pixelHeight;

            int rayWidth = width;
            int rayHeight = height ;

            UpdateRenderTargets(width, height);
            UpdateMatrices();
            UpdateVariables();

            Graphics.Blit(mainBuffer1, mainBuffer0, rendererMaterial, 0);

            RenderTexture rayCast = CreateTempBuffer(rayWidth, rayHeight, 0, RenderTextureFormat.ARGBHalf);
            rendererMaterial.SetVector("_RayCastSize", new Vector4(rayWidth, rayHeight, 0,0));
            rendererMaterial.SetTexture("_RayCast", rayCast);

            Graphics.SetRenderTarget(rayCast);
            rendererMaterial.SetPass(1);
            DrawFullScreenQuad();

            RenderTexture resolvePass = CreateTempBuffer(width, height, 0, RenderTextureFormat.ARGBHalf);

            if (useMipMap)
            {
                Graphics.Blit(mainBuffer0, resolvePass, rendererMaterial, 3);

                Graphics.Blit(resolvePass, mipMapBuffer0);

                for (int i = 0; i < maxMipMap; i++)
                {
                    rendererMaterial.SetVector("_GaussianDir", dist[i] * new Vector2(1.0f, 0.0f));
                    rendererMaterial.SetInt("_MipMapCount", mipLevel[i]);
                    Graphics.Blit(mipMapBuffer0, mipMapBuffer1, rendererMaterial, 2);

                    rendererMaterial.SetVector("_GaussianDir", dist[i] * new Vector2(0.0f, 1.0f));
                    rendererMaterial.SetInt("_MipMapCount", mipLevel[i]);
                    Graphics.Blit(mipMapBuffer1, mipMapBuffer0, rendererMaterial, 2);

                    Graphics.SetRenderTarget(mipMapBuffer2, i);
                    DrawFullScreenQuad();
                }

                rendererMaterial.SetTexture("_ReflectionBuffer", mipMapBuffer2);
            }
            else
            {
                Graphics.Blit(mainBuffer0, resolvePass, rendererMaterial, 3);

                rendererMaterial.SetTexture("_ReflectionBuffer", resolvePass);
            }

            ReleaseTempBuffer(rayCast);

            Graphics.Blit(source, mainBuffer1, rendererMaterial, 4);
            Graphics.Blit(mainBuffer1, destination);

            ReleaseTempBuffer(resolvePass);

        }

        void UpdateVariables()
        {
            if(animateJitter)
                rendererMaterial.SetInt("_UseTemporal", 1);
            else
                rendererMaterial.SetInt("_UseTemporal", 0);

            rendererMaterial.SetTexture("_Noise", noise);
            rendererMaterial.SetFloat("_SmoothnessRange", smoothnessRange);
            rendererMaterial.SetFloat("_EdgeFactor", reflectionEdgeFactor);
            rendererMaterial.SetInt("_NumSteps", numSteps);
        }

        void UpdateMatrices()
        {
            worldToCameraMatrix = m_camera.worldToCameraMatrix;
            cameraToWorldMatrix = worldToCameraMatrix.inverse;

            projectionMatrix = GL.GetGPUProjectionMatrix(m_camera.projectionMatrix, false);
            viewProjectionMatrix = projectionMatrix * worldToCameraMatrix;
            inverseViewProjectionMatrix = viewProjectionMatrix.inverse;

            rendererMaterial.SetMatrix("_ProjectionMatrix", projectionMatrix);
            rendererMaterial.SetMatrix("_ViewProjectionMatrix", viewProjectionMatrix);
            rendererMaterial.SetMatrix("_InverseProjectionMatrix", projectionMatrix.inverse);
            rendererMaterial.SetMatrix("_InverseViewProjectionMatrix", inverseViewProjectionMatrix);
            rendererMaterial.SetMatrix("_WorldToCameraMatrix", worldToCameraMatrix);
            rendererMaterial.SetMatrix("_CameraToWorldMatrix", cameraToWorldMatrix);
        }

        public void DrawFullScreenQuad()
        {
            GL.PushMatrix();
            GL.LoadOrtho();

            GL.Begin(GL.QUADS);
            GL.MultiTexCoord2(0, 0.0f, 0.0f);
            GL.Vertex3(0.0f, 0.0f, 0.0f); // BL

            GL.MultiTexCoord2(0, 1.0f, 0.0f);
            GL.Vertex3(1.0f, 0.0f, 0.0f); // BR

            GL.MultiTexCoord2(0, 1.0f, 1.0f);
            GL.Vertex3(1.0f, 1.0f, 0.0f); // TR

            GL.MultiTexCoord2(0, 0.0f, 1.0f);
            GL.Vertex3(0.0f, 1.0f, 0.0f); // TL

            GL.End();
            GL.PopMatrix();
        }
    }

}
