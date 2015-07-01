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

#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System.Collections;
using System.Collections.Generic;
using System.IO;

namespace unitySSR
{
	[CustomEditor(typeof(unitySSR))]
	public class unitySSREditor : Editor 
	{

		SerializedObject serObj;

		SerializedProperty numSteps;
		SerializedProperty reflectionEdgeFactor;
		SerializedProperty smoothnessRange;
		SerializedProperty textureSize;
		SerializedProperty sampleQuality;
		SerializedProperty brdfModel;
		SerializedProperty pointModel;
		//SerializedProperty highQualitySampling;

		void OnEnable()
		{
			serObj = new SerializedObject (target);
			numSteps = serObj.FindProperty ("numSteps");
			textureSize = serObj.FindProperty ("textureSize");
			smoothnessRange = serObj.FindProperty ("smoothnessRange");
			reflectionEdgeFactor = serObj.FindProperty ("reflectionEdgeFactor");
			sampleQuality = serObj.FindProperty ("sampleQuality");
			brdfModel = serObj.FindProperty ("brdfModel");
			pointModel = serObj.FindProperty ("pointModel");
		}

		public override void OnInspectorGUI()
		{
			GUILayout.Label ("General Parameters", EditorStyles.boldLabel);

			smoothnessRange.floatValue = EditorGUILayout.Slider (new GUIContent ("Smoothness Range","Specify the smoothness range ssr can read"),smoothnessRange.floatValue, 0.0f, 1.0f);
			reflectionEdgeFactor.floatValue = EditorGUILayout.Slider(new GUIContent ("Edge Mask Size", "Specify the size of the mask around screen edge"), reflectionEdgeFactor.floatValue, 0.1f, 0.8f);

			GUILayout.Label ("Ray Parameters", EditorStyles.boldLabel);

			numSteps.intValue = EditorGUILayout.IntSlider( new GUIContent ("Search Iteration","Specify how far the ray can iterate in the depth buffer"), numSteps.intValue, 1, 128);

			GUILayout.Label ("BRDF Parameters", EditorStyles.boldLabel);

			EditorGUILayout.PropertyField (pointModel, new GUIContent ("Sampling Method", "Hammersley gives better reflection blur but also increase cost"));
			EditorGUILayout.PropertyField (brdfModel, new GUIContent ("Sampling Model","Choose between Blinn and GGX"));
			EditorGUILayout.PropertyField (sampleQuality, new GUIContent ("Sampling Quality","Specify the number of samples used for BRDF blurring"));
			EditorGUILayout.PropertyField (textureSize, new GUIContent ("Texture Size","Specify the size of the mip mapped texture"));

			serObj.ApplyModifiedProperties();
		}
	}
}
#endif