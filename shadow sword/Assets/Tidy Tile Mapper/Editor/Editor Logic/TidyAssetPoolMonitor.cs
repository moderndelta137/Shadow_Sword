using System;
using UnityEngine;
using UnityEditor;
using DopplerInteractive.TidyTileMapper.Utilities;

namespace DopplerInteractive.TidyTileMapper.Editors{
	
	public class TidyAssetPoolMonitor : IEditorWindow
	{		
		EditorWindow parentWindow;
		
		#region IEditorWindow implementation
		public void Initialize (EditorWindow parentWindow)
		{
			this.parentWindow = parentWindow;
		}

		public void Update ()
		{
			
			parentWindow.Repaint();
		}
		
		Vector2 scrollPos = Vector2.zero;
		
		public void DrawWindow ()
		{			
			EditorGUILayout.BeginVertical();
			
			GUILayout.Label("Objects in pool:");
			
			if(AssetPool.IsPoolingEnabled()){
				
				scrollPos = EditorGUILayout.BeginScrollView(scrollPos);
				
				foreach(string s in AssetPool.pool.Keys){
					
					EditorGUILayout.BeginHorizontal();
					
					GUILayout.Label(s+":");
					
					GUILayout.Label(AssetPool.pool[s].Count + " objects in pool.");
					
					EditorGUILayout.EndHorizontal();
				}
				
				EditorGUILayout.EndScrollView();
					
			}
			else{
				
				GUILayout.Label("Asset pooling is disabled.");
				
			}
			
			EditorGUILayout.EndVertical();
		}

		public void DrawScene ()
		{
		}

		public void Destroy ()
		{
		}

		public void OnSelectionChange ()
		{
		}
		#endregion
	}
}

