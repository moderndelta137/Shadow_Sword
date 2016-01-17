using UnityEngine;
using UnityEditor;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

public class FindAndFixBrokenScripts : EditorWindow
{
	[MenuItem("Window/Find And Fix Missing Scripts")]
	static void Init()
	{
		// Get existing open window or if none, make a new one:
		FindAndFixBrokenScripts window = EditorWindow.GetWindow<FindAndFixBrokenScripts>("Missing Scripts");
		window.Show();
	}
	static FindAndFixBrokenScripts current;
	
	public List<GameObject> gameObjects;
	
	List<GameObject> processList = new List<GameObject>();
	
	List<GameObject> brokenList = null;
	bool scanned = false;
	
	bool showItems;
	
	
	void OnGUI()
	{
/*		var o = new SerializedObject(this);
		var p = o.FindProperty("gameObjects");
		var val = true;
		var canDo = false;
		do{
			EditorGUILayout.PropertyField(p);
			 canDo = p.NextVisible(val);
			val = false;
		}while(canDo);
		o.ApplyModifiedProperties();*/
		GUITitle("Missing Script Toolkit", new Color(0.7f,1f,0.7f,1f));
		GUILayout.BeginHorizontal();
		if(GUILayout.Button("Scan for broken scripts") || brokenList == null || !scanned)
		{
			scanned = true;
			brokenList =
				Resources.FindObjectsOfTypeAll(typeof(GameObject)).Cast<GameObject>().Where(c=>c.GetComponents<Component>().Any(o=>o==null)).ToList();
		}
		GUILayout.FlexibleSpace();
		GUILayout.EndHorizontal();
		
		if(brokenList.Count!=0)
		{
			GUITitle("Results", new Color(1,0.6f, 0.6f, 1f));
			if((showItems = EditorGUILayout.Foldout(showItems, "Missing Scripts")))
			{
				foreach(var script in brokenList.OrderBy(s=>s.name))
				{
					GUILayout.BeginHorizontal();
					GUILayout.Space(32);
					if(GUILayout.Button(script.name, "label"))
					{
						EditorGUIUtility.PingObject(script);
					}
					
					GUILayout.EndHorizontal();
				}
			}
			GUILayout.Space(10);
			if(GUILayout.Button("Fix Now", GUILayout.Width(80)))
			{
				FixMissingScripts.tried = false;
				EditorPrefs.SetBool("Fix", true);
				processList.AddRange(
					Resources.FindObjectsOfTypeAll(typeof(GameObject)).Cast<GameObject>().Where(c=>c.GetComponents<Component>().Any(o=>o==null))
					);
			}
		}
		else
		{
			GUILayout.Label("No broken scripts found");
		}
		
	}
	
	void GUITitle(string title, Color color)
	{
		int size = 25;
		
		var oldColor = GUI.color;
		GUI.color = color;
		
		var style =new GUIStyle("label");
		style.fontSize = size;
		
		GUILayout.Label(title, style); 
		GUILayout.Space(Mathf.CeilToInt(size/2));
		
		GUI.color = oldColor;
	}
	
	int nextTime=-1;
	bool trying;
	
	void Update()
	{
		if(nextTime > 0)
		{
			nextTime--;
			Repaint();
		} else if(nextTime ==0)
		{
		
			brokenList = null;
			scanned = false;
			nextTime = -1;
			Repaint();
		}
		if(!trying)
		{
			if(processList.Count > 0)
			{
				FixMissingScripts.tried = false;
				var first = processList[0];
				FixMissingScripts.tryThisObject = first;
				processList.RemoveAt(0);
				Selection.activeObject = first;
				if(processList.Count==0)
				{
					nextTime = 10;
				}
				Repaint();
				trying = true;
				
			}
		}
		if(trying && FixMissingScripts.tried)
			trying = false;
			
	}
	


}
