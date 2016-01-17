using UnityEngine;
using UnityEditor;

public class UpdateWindow : EditorWindow
{
	//Hello friend!
	//
	//I've put this window into your project 
	//so that when you get an update, and it has important information
	//E.g: something may break your project
	//It will inform you so that you know to make provisions
	//
	//I realised a lot of people may not be on the forums
	//And I don't want to rudely shock you by breaking your project
	//
	//So hopefully this will make things a little easier
	
	public string updateInformation = "" +
		"Welcome to Tidy TileMapper v1.x\n"+
			"Hello friends! Unity 4.0 is here, and spring is in the air (figuratively)\n"+
			"In the spirit of free love, bees and flowers I have modified this tool to be\n"+
			"full source (as opposed to DLL).\n"+
			"NOTE PLEASE: IMPORTANT: If you are converting from an earlier version of TTM\n"+
			"You must do the following things:\n"+
			"1) Delete the Editor\\Editor Logic\\TidyTileMapper_Editor.dll file\n"+
			"2) Delete the Mapping\\TidyTileMapper.dll file\n"+
			"3) Fret momentarily - because all of your prefabs have lost their scripts!\n"+
			"4) Go to Window>Find and fix missing scripts\n"+
			"5) Done!\n"+
			"Note this script was found online - I have not modified it in any way,\n"+
			"and have retained the folder naming that points to the creator's website.\n"+
			"Give them some love.\n"+
			"-Joshua McGrath\n\n"+
			"(Hit OK to stop this message appearing again)";
	 
	public static string UPDATE_VERSION_KEY = "TTM_V_1_x";

	public static void Init () {
		
		if(EditorPrefs.HasKey(UPDATE_VERSION_KEY)){
			return;
		}
		
		EditorWindow w = EditorWindow.GetWindow(typeof(UpdateWindow),true,"Update: Tidy Tile Mapper",true); 
		w.position = new Rect(Screen.width*0.5f,Screen.height*0.5f,750.0f,300.0f);
	}
	
	Vector2 scrollPos = Vector2.zero;
	
	void OnGUI(){
				
		EditorGUILayout.BeginVertical();
		
		scrollPos = EditorGUILayout.BeginScrollView(scrollPos);
		
		EditorGUILayout.TextArea(updateInformation);
		
		EditorGUILayout.EndScrollView();
		
		EditorGUILayout.EndVertical();
		
		EditorGUILayout.BeginHorizontal();
		
		GUILayout.FlexibleSpace();
		
		if(GUILayout.Button("OK")){
			EditorPrefs.SetBool(UPDATE_VERSION_KEY,true);
			Close ();
		}
		
		EditorGUILayout.EndHorizontal();
		
	}
}
