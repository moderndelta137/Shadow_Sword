using System;
using UnityEditor;
using DopplerInteractive.TidyTileMapper.Editors;

public interface ITTMPlugin
{
	void InitializePlugin(EditorWindow parentWindow, TidyBlockMapCreator mapCreator);
	void DrawPlugin();
	void DrawScene(SceneView sceneView);
	void UpdatePlugin(float deltaTime);
	void FinalizePlugin();
	
	void DeactivateTool();
}


