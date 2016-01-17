using System;
using UnityEngine;
using UnityEditor;

namespace DopplerInteractive.TidyTileMapper.Editors{

	public interface IEditorWindow
	{
		void Initialize(EditorWindow parentWindow);
		void Update();
		void DrawWindow();
		void DrawScene();
		void Destroy();
		
		void OnSelectionChange();
	}
}

